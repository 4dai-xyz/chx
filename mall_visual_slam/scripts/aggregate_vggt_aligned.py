#!/usr/bin/env python3
"""
基于"共享帧"对所有 VGGT 局部窗口做全局对齐，输出一张干净的全局点云。

为什么需要这个脚本：
VGGT 每次只在一个窗口内做多视图几何，输出的相机位姿和深度都只在
该窗口的局部坐标系里一致；如果在窗口之间不做对齐，直接把所有局部
点云堆在一起，就会看到一团混乱的点云（旧版 open_vggt_viser.py 里
那种"按 X 轴排开"也只是占位，并不是真正的全局重建）。

本脚本的做法：
1. 假设 run_vggt_video.py 跑的时候用了 window-size > window-stride，
   于是相邻窗口之间有若干共享帧；
2. 用每对相邻窗口里 *共享帧* 的相机中心做 Umeyama sim(3) 对齐，
   解出旋转 + 平移 + 尺度，把窗口 k+1 的局部坐标转到窗口 k 上；
3. 链式相乘后所有窗口都落在窗口 0 的坐标系里；
4. 对每个窗口的深度图按置信度百分位 + 边缘裁剪 + 深度上限做过滤，
   再反投影到全局；
5. 最后做 voxel 下采样，得到一张体素均匀的全局点云。

入口函数 `aggregate_aligned_scene` 可以被 run_vggt_video.py 调用，
也可以通过命令行单独跑（适合重新调聚合参数而不重跑 VGGT）。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


REPO = Path(__file__).resolve().parent.parent


# ---------------------------------------------------------------------------
# 基础工具
# ---------------------------------------------------------------------------


def closed_form_inverse_se3_single(extrinsic: np.ndarray) -> np.ndarray:
    """对一个 3x4 的 world-to-camera 外参求逆，返回 4x4 的 camera-to-world。"""
    rotation = extrinsic[:3, :3]
    translation = extrinsic[:3, 3]
    inv = np.eye(4, dtype=np.float64)
    inv[:3, :3] = rotation.T
    inv[:3, 3] = -rotation.T @ translation
    return inv


def umeyama_sim3(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    """对两组对应点做 Umeyama sim(3) 对齐：dst ≈ s * R * src + t。

    参数:
      src: (N, 3) 源点（窗口 k+1 局部坐标里）
      dst: (N, 3) 目标点（窗口 k 局部坐标里）

    返回:
      R (3,3), t (3,), s (float)，满足 dst ≈ s * R @ src.T + t
    """
    src = np.asarray(src, dtype=np.float64)
    dst = np.asarray(dst, dtype=np.float64)
    assert src.shape == dst.shape and src.ndim == 2 and src.shape[1] == 3

    num = src.shape[0]
    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_centered = src - src_mean
    dst_centered = dst - dst_mean

    # 协方差矩阵 H = (1/N) * src_centered^T @ dst_centered。
    cov = (src_centered.T @ dst_centered) / num
    U, sigma, Vt = np.linalg.svd(cov)

    # 修正反射的情况，保证 R 是正常旋转矩阵。
    d = np.sign(np.linalg.det(U @ Vt))
    S = np.diag([1.0, 1.0, d])
    R = Vt.T @ S @ U.T

    # 尺度估计：用源点的方差归一化。当所有源点都重合时（共享帧极少时可能），
    # 退化为 1.0，避免除 0。
    src_var = np.sum(src_centered ** 2) / num
    if src_var < 1e-12:
        scale = 1.0
    else:
        scale = float(np.sum(sigma * np.diag(S)) / src_var)

    t = dst_mean - scale * (R @ src_mean)
    return R, t, scale


def voxel_downsample(
    points: np.ndarray,
    colors: np.ndarray,
    weights: np.ndarray,
    voxel_size: float,
) -> tuple[np.ndarray, np.ndarray]:
    """简单的 voxel 下采样，按置信度加权平均，每个 voxel 输出一个点。"""
    if voxel_size <= 0 or len(points) == 0:
        return points, colors

    coords = np.floor(points / voxel_size).astype(np.int64)
    # 把三维 voxel 索引压成一维 key。这里用 cantor-like 的散列避免负数 / 越界。
    key = (
        coords[:, 0].astype(np.int64) * 73856093
        ^ coords[:, 1].astype(np.int64) * 19349663
        ^ coords[:, 2].astype(np.int64) * 83492791
    )

    order = np.argsort(key, kind="stable")
    key_sorted = key[order]
    points_sorted = points[order]
    colors_sorted = colors[order].astype(np.float64)
    weights_sorted = weights[order].astype(np.float64)

    # 找每个唯一 key 的起始位置。
    boundaries = np.flatnonzero(np.diff(key_sorted)) + 1
    starts = np.concatenate(([0], boundaries))
    ends = np.concatenate((boundaries, [len(key_sorted)]))

    out_points = np.zeros((len(starts), 3), dtype=np.float64)
    out_colors = np.zeros((len(starts), 3), dtype=np.float64)
    for i, (s, e) in enumerate(zip(starts, ends)):
        w = weights_sorted[s:e]
        w_sum = w.sum()
        if w_sum < 1e-9:
            out_points[i] = points_sorted[s:e].mean(axis=0)
            out_colors[i] = colors_sorted[s:e].mean(axis=0)
        else:
            out_points[i] = (points_sorted[s:e] * w[:, None]).sum(axis=0) / w_sum
            out_colors[i] = (colors_sorted[s:e] * w[:, None]).sum(axis=0) / w_sum

    return out_points.astype(np.float32), np.clip(out_colors, 0, 255).astype(np.uint8)


def save_ascii_ply(points: np.ndarray, colors: np.ndarray, path: Path) -> None:
    """保存 ASCII PLY 点云。和 run_vggt_video.py 用的是同一种格式。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for point, color in zip(points, colors):
            f.write(
                f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} "
                f"{int(color[0])} {int(color[1])} {int(color[2])}\n"
            )


# ---------------------------------------------------------------------------
# 单窗口点云生成
# ---------------------------------------------------------------------------


def load_window_meta(window_dir: Path) -> dict:
    """读一个窗口的 summary.json。需要里面带 extrinsics/intrinsics（新版 run 才会有）。"""
    summary_path = window_dir / "summary.json"
    if not summary_path.exists():
        raise FileNotFoundError(f"找不到 {summary_path}，请确认窗口已成功跑完。")
    meta = json.loads(summary_path.read_text(encoding="utf-8"))
    if "extrinsics" not in meta or "intrinsics" not in meta:
        raise KeyError(
            "summary.json 里没有完整的 extrinsics / intrinsics，"
            "请用新版 scripts/run_vggt_video.py 重跑 VGGT 才能做全局对齐。"
        )
    return meta


def load_window_npz(window_dir: Path) -> dict[str, np.ndarray]:
    """读一个窗口的 predictions.npz（新版保存了 depth/depth_conf/extrinsic/intrinsic）。"""
    npz_path = window_dir / "predictions.npz"
    if not npz_path.exists():
        raise FileNotFoundError(
            f"找不到 {npz_path}；请用新版 scripts/run_vggt_video.py 重跑，"
            "聚合脚本需要每个窗口的 depth 和 depth_conf。"
        )
    with np.load(npz_path, allow_pickle=False) as data:
        return {key: data[key] for key in data.files}


def unproject_window_points(
    depth: np.ndarray,
    depth_conf: np.ndarray,
    intrinsic: np.ndarray,
    extrinsic: np.ndarray,
    image_rgb_for_frame: np.ndarray | None,
    conf_threshold: float,
    edge_margin: int,
    depth_max: float | None,
    image_valid_mask: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """把一帧深度反投影到窗口局部世界坐标系，并按置信度 / 边缘 / 深度做过滤。

    返回 (points_world_local, colors_rgb_uint8, weights)。
    """
    if depth.ndim == 3 and depth.shape[-1] == 1:
        depth = depth[..., 0]
    height, width = depth.shape

    # 像素网格。
    u, v = np.meshgrid(np.arange(width), np.arange(height))

    # 基本有效性 mask：正深度 + 高于置信度阈值 + 远离边缘。
    valid = np.isfinite(depth) & (depth > 0)
    valid &= np.isfinite(depth_conf) & (depth_conf >= conf_threshold)
    if edge_margin > 0:
        edge_mask = np.zeros_like(valid)
        edge_mask[edge_margin:height - edge_margin, edge_margin:width - edge_margin] = True
        valid &= edge_mask
    if depth_max is not None and depth_max > 0:
        valid &= depth <= depth_max
    # VGGT pad 出来的白边里没有真实图像内容，VGGT 在那里推的深度是假的；
    # 用图像有效区域的 mask 把这些假点也剔掉。
    if image_valid_mask is not None and image_valid_mask.shape == valid.shape:
        valid &= image_valid_mask

    if not np.any(valid):
        return (
            np.empty((0, 3), dtype=np.float32),
            np.empty((0, 3), dtype=np.uint8),
            np.empty((0,), dtype=np.float32),
        )

    # 像素 → 相机坐标。
    fx = float(intrinsic[0, 0])
    fy = float(intrinsic[1, 1])
    cx = float(intrinsic[0, 2])
    cy = float(intrinsic[1, 2])
    z = depth[valid].astype(np.float64)
    x_cam = (u[valid].astype(np.float64) - cx) * z / fx
    y_cam = (v[valid].astype(np.float64) - cy) * z / fy
    cam_xyz = np.stack([x_cam, y_cam, z], axis=-1)

    # 相机 → 局部世界坐标：x_world = R^T (x_cam - t)。
    cam_to_world = closed_form_inverse_se3_single(extrinsic.astype(np.float64))
    world_xyz = cam_xyz @ cam_to_world[:3, :3].T + cam_to_world[:3, 3]

    # 颜色：images 数组是模型预处理后的 0-1 RGB；如果没传就用置信度上色，方便定位异常。
    if image_rgb_for_frame is not None:
        colors_float = image_rgb_for_frame[valid].astype(np.float64) * 255.0
        colors = np.clip(colors_float, 0, 255).astype(np.uint8)
    else:
        gray = np.clip((depth_conf[valid] / max(float(depth_conf[valid].max()), 1e-6)) * 255.0, 0, 255).astype(np.uint8)
        colors = np.stack([gray, gray, gray], axis=-1)

    weights = depth_conf[valid].astype(np.float32)
    return world_xyz.astype(np.float32), colors, weights


# ---------------------------------------------------------------------------
# 全局对齐
# ---------------------------------------------------------------------------


def _estimate_scale_from_depth_medians(
    depths_k: np.ndarray,
    confs_k: np.ndarray,
    shared_positions_k: list[int],
    depths_kp1: np.ndarray,
    confs_kp1: np.ndarray,
    shared_positions_kp1: list[int],
    conf_threshold: float,
) -> float:
    """用同一张共享帧在两个窗口里预测的深度中位数比值估计 sim(3) 尺度。

    为什么不直接从相机中心拟合：相邻 sample 在 1Hz 抽样下相机位移很小，
    相机中心几乎重合，Umeyama 拟合出来的尺度对噪声极敏感，常常给出 0.7-0.8
    这种系统性偏差。而 VGGT 对共享帧本身估的深度是同一面物理场景，深度比
    直接反映两个窗口的尺度比，鲁棒得多。
    """
    ratios: list[float] = []
    for pos_k, pos_kp1 in zip(shared_positions_k, shared_positions_kp1):
        depth_k = depths_k[pos_k].squeeze()
        depth_kp1 = depths_kp1[pos_kp1].squeeze()
        conf_k = confs_k[pos_k]
        conf_kp1 = confs_kp1[pos_kp1]

        mask = (
            np.isfinite(depth_k)
            & np.isfinite(depth_kp1)
            & (depth_k > 0)
            & (depth_kp1 > 0)
            & (conf_k >= conf_threshold)
            & (conf_kp1 >= conf_threshold)
        )
        if not np.any(mask):
            continue

        median_k = float(np.median(depth_k[mask]))
        median_kp1 = float(np.median(depth_kp1[mask]))
        if median_kp1 < 1e-9:
            continue
        ratios.append(median_k / median_kp1)

    if not ratios:
        return 1.0
    return float(np.median(ratios))


def compute_pairwise_alignment(
    extrinsics_k: np.ndarray,
    sample_indices_k: list[int],
    extrinsics_kp1: np.ndarray,
    sample_indices_kp1: list[int],
    depths_k: np.ndarray | None = None,
    confs_k: np.ndarray | None = None,
    depths_kp1: np.ndarray | None = None,
    confs_kp1: np.ndarray | None = None,
    conf_threshold: float = 1.0,
    scale_mode: str = "depth_median",
) -> tuple[np.ndarray, float, list[int]]:
    """根据相邻窗口的共享帧解出 sim(3) 对齐变换。

    参数:
      extrinsics_k / extrinsics_kp1: 两个窗口的全部 world-to-cam 外参，shape (N, 3, 4)。
      sample_indices_k / sample_indices_kp1: 两个窗口对应的全局 sample_index 列表。
      depths_k / confs_k / depths_kp1 / confs_kp1:
        scale_mode='depth_median' 时需要这些，用共享帧的 depth 中位数比估尺度。
        其他 mode 可以传 None。
      conf_threshold: depth_median 模式下过滤低置信度像素时的阈值。
      scale_mode:
        - 'depth_median': 推荐。尺度由共享帧深度中位数比给出，旋转+平移仍用 Umeyama 解。
        - 'umeyama':      纯 Umeyama 拟合尺度+旋转+平移（容易因相机微位移而漂移）。
        - 'se3':          强制 scale=1，不允许相对尺度漂移；适合 VGGT 输出已经接近真实尺度的场景。

    返回:
      (T_4x4, scale, shared_local_indices_in_kp1)。
      T 把窗口 k+1 的局部坐标变到窗口 k 的局部坐标。
    """
    # 找共享 sample_index：窗口 k+1 的帧落在窗口 k 的 sample 列表里。
    set_k = {idx: pos for pos, idx in enumerate(sample_indices_k)}
    shared_pairs: list[tuple[int, int]] = []
    for pos_kp1, idx in enumerate(sample_indices_kp1):
        if idx in set_k:
            shared_pairs.append((set_k[idx], pos_kp1))

    if len(shared_pairs) == 0:
        raise ValueError(
            "相邻窗口没有共享 sample_index，"
            "请把 run_vggt_video.py 的 --window-stride 设小于 --window-size。"
        )

    centers_k = []
    centers_kp1 = []
    for pos_k, pos_kp1 in shared_pairs:
        c_k = -extrinsics_k[pos_k, :, :3].T @ extrinsics_k[pos_k, :, 3]
        c_kp1 = -extrinsics_kp1[pos_kp1, :, :3].T @ extrinsics_kp1[pos_kp1, :, 3]
        centers_k.append(c_k)
        centers_kp1.append(c_kp1)
    centers_k = np.asarray(centers_k, dtype=np.float64)
    centers_kp1 = np.asarray(centers_kp1, dtype=np.float64)

    # 1) 先估尺度。
    if scale_mode == "se3":
        scale = 1.0
    elif scale_mode == "depth_median":
        if depths_k is None or depths_kp1 is None:
            raise ValueError("scale_mode='depth_median' 时必须传入 depths/confs。")
        scale = _estimate_scale_from_depth_medians(
            depths_k, confs_k, [p[0] for p in shared_pairs],
            depths_kp1, confs_kp1, [p[1] for p in shared_pairs],
            conf_threshold=conf_threshold,
        )
    elif scale_mode == "umeyama":
        if len(shared_pairs) >= 3:
            _R_tmp, _t_tmp, s_tmp = umeyama_sim3(centers_kp1, centers_k)
            scale = float(s_tmp)
        else:
            scale = 1.0
    else:
        raise ValueError(f"未知的 scale_mode: {scale_mode}")

    # 2) 用估好的尺度，把 src 缩放后再做 Kabsch (SE(3)) 解出旋转+平移。
    # 整个 transform 形式：x_k = (scale * R) @ x_kp1 + t，落到 4x4 矩阵就是 [scale*R | t]。
    src_scaled = scale * centers_kp1
    transform = np.eye(4, dtype=np.float64)
    R = np.eye(3, dtype=np.float64)
    t = np.zeros(3, dtype=np.float64)

    if len(shared_pairs) >= 3:
        src_mean = src_scaled.mean(axis=0)
        dst_mean = centers_k.mean(axis=0)
        H = (src_scaled - src_mean).T @ (centers_k - dst_mean)
        U, _S, Vt = np.linalg.svd(H)
        d = np.sign(np.linalg.det(Vt.T @ U.T))
        D = np.diag([1.0, 1.0, d])
        R = Vt.T @ D @ U.T
        t = dst_mean - R @ src_mean
    elif len(shared_pairs) == 2:
        v_k = centers_k[1] - centers_k[0]
        v_kp1 = src_scaled[1] - src_scaled[0]
        nrm_k = float(np.linalg.norm(v_k))
        nrm_kp1 = float(np.linalg.norm(v_kp1))
        if nrm_k > 1e-9 and nrm_kp1 > 1e-9:
            R = _rotation_between_vectors(v_kp1 / nrm_kp1, v_k / nrm_k)
        t = centers_k.mean(axis=0) - R @ src_scaled.mean(axis=0)
    else:
        # 单共享帧：尺度由 depth_median 提供，旋转用共享帧的 cam-to-world 关系。
        cam_to_world_k = closed_form_inverse_se3_single(extrinsics_k[shared_pairs[0][0]])
        cam_to_world_kp1 = closed_form_inverse_se3_single(extrinsics_kp1[shared_pairs[0][1]])
        R = cam_to_world_k[:3, :3] @ cam_to_world_kp1[:3, :3].T
        # t 取共享帧在两边的中心一致：x_k = scale * R @ x_kp1 + t，C_k = scale * R @ C_kp1 + t。
        c_k = cam_to_world_k[:3, 3]
        c_kp1_scaled = scale * cam_to_world_kp1[:3, 3]
        t = c_k - R @ c_kp1_scaled

    transform[:3, :3] = scale * R
    transform[:3, 3] = t

    shared_kp1_positions = [pos_kp1 for _, pos_kp1 in shared_pairs]
    return transform, scale, shared_kp1_positions


def _rotation_between_vectors(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """求把单位向量 a 旋到 b 的最小旋转矩阵（Rodrigues 公式）。"""
    a = a / max(np.linalg.norm(a), 1e-12)
    b = b / max(np.linalg.norm(b), 1e-12)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    s = float(np.linalg.norm(v))
    if s < 1e-9:
        # 平行或反平行：平行时返回单位阵；反平行时绕任意垂直轴 180 度。
        if c > 0:
            return np.eye(3)
        # 找一个与 a 垂直的轴。
        axis_candidates = np.eye(3)
        for axis in axis_candidates:
            if abs(np.dot(axis, a)) < 0.9:
                v = np.cross(a, axis)
                v /= max(np.linalg.norm(v), 1e-12)
                break
        K = np.array(
            [[0.0, -v[2], v[1]],
             [v[2], 0.0, -v[0]],
             [-v[1], v[0], 0.0]],
        )
        return np.eye(3) + 2.0 * K @ K
    K = np.array(
        [[0.0, -v[2], v[1]],
         [v[2], 0.0, -v[0]],
         [-v[1], v[0], 0.0]],
    )
    return np.eye(3) + K + K @ K * ((1 - c) / (s * s))


# ---------------------------------------------------------------------------
# 主聚合函数
# ---------------------------------------------------------------------------


def aggregate_aligned_scene(
    result_dir: Path,
    output_dir: Path,
    conf_percentile: float = 70.0,
    edge_margin: int = 8,
    depth_max_ratio: float = 5.0,
    voxel_size_ratio: float = 0.02,
    max_total_points: int = 0,
    scale_mode: str = "depth_median",
) -> dict:
    """读取一个 VGGT 运行结果目录，做全局对齐 + 过滤，输出干净的全局点云。

    参数:
      result_dir: run_vggt_video.py 的输出目录（要带 windows/window_*/predictions.npz）。
      output_dir: 聚合结果保存目录。
      conf_percentile: 丢弃低置信度像素的百分位（按整段视频的全部 depth_conf 统计）。
      edge_margin: 每张深度图四周裁掉的像素数。
      depth_max_ratio: 每个窗口按 depth 中位数的多少倍截断远点；0 表示不截断。
      voxel_size_ratio: voxel 下采样尺寸（相对场景包围盒最大边长的比例）；0 表示不下采样。
      max_total_points: 输出最多多少个点（voxel 下采样之后再随机抽稀）；0 表示不限制。
      scale_mode: 相邻窗口的尺度估计方式，见 compute_pairwise_alignment 的说明。

    返回:
      聚合摘要 dict，会写到 output_dir/aligned_full_summary.json。
    """
    result_dir = Path(result_dir)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    window_dirs = sorted((result_dir / "windows").glob("window_*"))
    if not window_dirs:
        raise FileNotFoundError(f"没有找到任何窗口目录: {result_dir / 'windows'}")

    # 第一遍：读所有窗口的 metadata + npz。这一步内存里只放 depth 和 conf；
    # 单个窗口大约 4-6MB，几十个窗口完全放得下。如果窗口太多可以改成流式。
    print(f"读取 {len(window_dirs)} 个窗口的 metadata 和 npz ...")
    window_metas: list[dict] = []
    window_npzs: list[dict[str, np.ndarray]] = []
    for window_dir in window_dirs:
        window_metas.append(load_window_meta(window_dir))
        window_npzs.append(load_window_npz(window_dir))

    # 计算全局置信度阈值。
    print("统计全局置信度分布 ...")
    all_conf = np.concatenate([npz["depth_conf"].reshape(-1) for npz in window_npzs])
    finite_conf = all_conf[np.isfinite(all_conf)]
    if finite_conf.size == 0:
        raise RuntimeError("所有窗口的 depth_conf 都是非有限值，无法做置信度过滤。")
    conf_threshold = float(np.percentile(finite_conf, conf_percentile))
    print(f"  置信度阈值 (p{conf_percentile:g}) = {conf_threshold:.4f}")

    # 第二遍：计算相邻窗口的两两对齐变换，并链式相乘成全局 T_k_to_0。
    print(f"解相邻窗口的 sim(3) 对齐 (scale_mode={scale_mode}) ...")
    transforms_to_global = [np.eye(4, dtype=np.float64)]
    pairwise_scales = []
    skipped_pairs = []
    for k in range(1, len(window_metas)):
        meta_prev = window_metas[k - 1]
        meta_curr = window_metas[k]
        npz_prev = window_npzs[k - 1]
        npz_curr = window_npzs[k]
        try:
            T_curr_to_prev, scale, _shared = compute_pairwise_alignment(
                extrinsics_k=np.asarray(meta_prev["extrinsics"], dtype=np.float64),
                sample_indices_k=meta_prev["sample_indices"],
                extrinsics_kp1=np.asarray(meta_curr["extrinsics"], dtype=np.float64),
                sample_indices_kp1=meta_curr["sample_indices"],
                depths_k=npz_prev["depth"],
                confs_k=npz_prev["depth_conf"],
                depths_kp1=npz_curr["depth"],
                confs_kp1=npz_curr["depth_conf"],
                conf_threshold=conf_threshold,
                scale_mode=scale_mode,
            )
        except ValueError as exc:
            # 共享帧为 0：链断了。把这之后的窗口都标记为漂浮，但仍尝试继续。
            skipped_pairs.append({"k": k, "reason": str(exc)})
            T_curr_to_prev = np.eye(4, dtype=np.float64)
            scale = 1.0
        pairwise_scales.append(float(scale))
        transforms_to_global.append(transforms_to_global[-1] @ T_curr_to_prev)

    # 第三遍：用每个窗口的每帧深度反投影 + 用 T_k_to_0 转到全局。
    # 这一步把 images 也读进来用作颜色。
    print("反投影每个窗口的深度并对齐到全局 ...")
    all_points: list[np.ndarray] = []
    all_colors: list[np.ndarray] = []
    all_weights: list[np.ndarray] = []
    all_camera_centers_global: list[dict] = []

    for k, (meta, npz) in enumerate(zip(window_metas, window_npzs)):
        extrinsics = np.asarray(meta["extrinsics"], dtype=np.float64)
        intrinsics = np.asarray(meta["intrinsics"], dtype=np.float64)
        depths = npz["depth"]
        confs = npz["depth_conf"]
        T = transforms_to_global[k]

        # 每个窗口先估一个深度中位数，用来做远点截断。
        depth_values = depths[np.isfinite(depths) & (depths > 0)]
        if depth_values.size == 0:
            continue
        depth_median = float(np.median(depth_values))
        depth_max = depth_median * depth_max_ratio if depth_max_ratio > 0 else None

        # 颜色：window 内 frame 数和 depth/conf 一致，从 result_dir/images 读对应原图。
        # 这里我们直接复用 images_dir 下 PNG，做一次按 518x518 的简单缩放对齐。
        for frame_pos, sample_idx in enumerate(meta["sample_indices"]):
            image_path = result_dir / "images" / f"frame_{sample_idx:05d}_src_{meta['source_frames'][frame_pos]:06d}.png"
            image_rgb, image_valid_mask = _load_image_to_depth_grid(image_path, depths.shape[1:3])

            points_local, colors, weights = unproject_window_points(
                depth=depths[frame_pos],
                depth_conf=confs[frame_pos],
                intrinsic=intrinsics[frame_pos],
                extrinsic=extrinsics[frame_pos],
                image_rgb_for_frame=image_rgb,
                conf_threshold=conf_threshold,
                edge_margin=edge_margin,
                depth_max=depth_max,
                image_valid_mask=image_valid_mask,
            )
            if len(points_local) == 0:
                continue

            # 把局部点云搬到全局。
            ones = np.ones((points_local.shape[0], 1), dtype=np.float64)
            homo = np.hstack([points_local.astype(np.float64), ones])
            points_global = (T @ homo.T).T[:, :3]
            all_points.append(points_global.astype(np.float32))
            all_colors.append(colors)
            all_weights.append(weights)

        # 同步把每个相机的全局位姿也保存下来，方便后续 viewer 画轨迹。
        for frame_pos, sample_idx in enumerate(meta["sample_indices"]):
            cam_to_world_local = closed_form_inverse_se3_single(extrinsics[frame_pos])
            cam_to_world_global = T @ cam_to_world_local
            all_camera_centers_global.append(
                {
                    "window_index": k,
                    "sample_index": int(sample_idx),
                    "source_frame": int(meta["source_frames"][frame_pos]),
                    "source_time_sec": float(meta["source_times_sec"][frame_pos]),
                    "position": cam_to_world_global[:3, 3].tolist(),
                    "rotation": cam_to_world_global[:3, :3].tolist(),
                }
            )

    if not all_points:
        raise RuntimeError("聚合后没有任何点，请放宽 conf_percentile / edge_margin / depth_max_ratio。")

    points = np.concatenate(all_points, axis=0)
    colors = np.concatenate(all_colors, axis=0)
    weights = np.concatenate(all_weights, axis=0)
    print(f"过滤后总点数: {len(points)}")

    # voxel 下采样。voxel_size 用包围盒最大边长 * voxel_size_ratio 算出来。
    if voxel_size_ratio > 0 and len(points) > 0:
        extent = float(np.ptp(points, axis=0).max())
        voxel_size = max(extent * voxel_size_ratio, 1e-6)
        print(f"  voxel 下采样, voxel_size = {voxel_size:.4g}")
        points, colors = voxel_downsample(points, colors, weights, voxel_size)
        print(f"  voxel 后剩余点数: {len(points)}")

    if max_total_points > 0 and len(points) > max_total_points:
        rng = np.random.default_rng(42)
        idx = rng.choice(len(points), size=max_total_points, replace=False)
        points = points[idx]
        colors = colors[idx]

    aligned_ply_path = output_dir / "aligned_full_scene.ply"
    aligned_traj_path = output_dir / "aligned_camera_trajectory.json"
    aligned_traj_txt_path = output_dir / "aligned_camera_centers.txt"
    aligned_summary_path = output_dir / "aligned_full_summary.json"

    save_ascii_ply(points, colors, aligned_ply_path)

    aligned_traj_path.write_text(
        json.dumps({"cameras": all_camera_centers_global}, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    with aligned_traj_txt_path.open("w", encoding="utf-8") as f:
        f.write("# window_index sample_index source_frame source_time_sec tx ty tz\n")
        for cam in all_camera_centers_global:
            t = cam["position"]
            f.write(
                f"{cam['window_index']} {cam['sample_index']} {cam['source_frame']} "
                f"{cam['source_time_sec']:.6f} {t[0]:.6f} {t[1]:.6f} {t[2]:.6f}\n"
            )

    summary = {
        "result_dir": str(result_dir),
        "output_dir": str(output_dir),
        "num_windows": len(window_metas),
        "num_aligned_windows": len(window_metas) - len(skipped_pairs),
        "conf_percentile": conf_percentile,
        "conf_threshold_used": conf_threshold,
        "edge_margin": edge_margin,
        "depth_max_ratio": depth_max_ratio,
        "voxel_size_ratio": voxel_size_ratio,
        "scale_mode": scale_mode,
        "aggregated_points": int(len(points)),
        "aligned_ply": str(aligned_ply_path),
        "aligned_trajectory_json": str(aligned_traj_path),
        "aligned_trajectory_txt": str(aligned_traj_txt_path),
        "pairwise_scales_window_k_to_k_minus_1": pairwise_scales,
        "skipped_pairs": skipped_pairs,
        "note": (
            "由相邻窗口共享帧做 sim(3) 链式对齐，仅依赖 VGGT 在窗口内的局部一致性。"
            "长视频可能存在累计漂移，可通过减小 frame-step、增大 window-size / 减小 stride 进一步降低漂移。"
        ),
    }
    aligned_summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    return summary


def _load_image_to_depth_grid(image_path: Path, depth_hw: tuple[int, int]) -> tuple[np.ndarray | None, np.ndarray | None]:
    """从原始 PNG 读图并对齐到 depth 的网格（VGGT 默认 518x518）。

    匹配 vggt/utils/load_fn.py 里 `mode='pad'` 的官方预处理：
    1. 等比缩放到长边等于 target_size，短边四舍五入到 14 的倍数；
    2. 短边以白色（1.0）pad 到方形；
    3. 输出 RGB float 0-1，shape (H, W, 3)。

    同时返回一个 valid mask（不是白色 padding 的真实图像区域），
    聚合脚本会用它把 padding 区域里 VGGT 误推的"假深度"剔掉。
    """
    try:
        import cv2
    except ImportError:
        return None, None

    image_bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image_bgr is None:
        return None, None

    target_h, target_w = depth_hw
    target_size = target_h  # VGGT 默认输出方形深度，长宽相等。
    h, w = image_bgr.shape[:2]
    if w >= h:
        new_w = target_size
        new_h = max(14, int(round(h * (new_w / w) / 14) * 14))
    else:
        new_h = target_size
        new_w = max(14, int(round(w * (new_h / h) / 14) * 14))
    new_w = min(new_w, target_w)
    new_h = min(new_h, target_h)

    # VGGT 用 PIL.BICUBIC；OpenCV 的 INTER_CUBIC 跟它接近，颜色差异对可视化够用。
    resized = cv2.resize(image_bgr, (new_w, new_h), interpolation=cv2.INTER_CUBIC)
    canvas = np.full((target_h, target_w, 3), 255, dtype=np.uint8)
    off_x = (target_w - new_w) // 2
    off_y = (target_h - new_h) // 2
    canvas[off_y:off_y + new_h, off_x:off_x + new_w] = resized

    valid_mask = np.zeros((target_h, target_w), dtype=bool)
    valid_mask[off_y:off_y + new_h, off_x:off_x + new_w] = True

    return cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0, valid_mask


# ---------------------------------------------------------------------------
# 命令行入口
# ---------------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="对 VGGT 多窗口输出做基于共享帧的全局对齐，输出干净的全局点云",
    )
    parser.add_argument(
        "--result-dir",
        default=str(REPO / "output" / "vggt_input_video_show"),
        help="run_vggt_video.py 的输出目录",
    )
    parser.add_argument(
        "--output-dir",
        default="",
        help="聚合结果保存目录；默认在 result-dir 下建 aligned_full 子目录",
    )
    parser.add_argument("--conf-percentile", type=float, default=70.0)
    parser.add_argument("--edge-margin", type=int, default=8)
    parser.add_argument("--depth-max-ratio", type=float, default=5.0)
    parser.add_argument("--voxel-size-ratio", type=float, default=0.02)
    parser.add_argument("--max-total-points", type=int, default=0)
    parser.add_argument(
        "--scale-mode",
        choices=["depth_median", "umeyama", "se3"],
        default="depth_median",
        help=(
            "相邻窗口的尺度估计方式："
            "depth_median 用共享帧深度中位数比（推荐，最稳）；"
            "umeyama 直接用相机中心拟合（1Hz 抽样时不稳）；"
            "se3 不估尺度，假设 VGGT 各窗口尺度天然一致。"
        ),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result_dir = Path(args.result_dir)
    output_dir = Path(args.output_dir) if args.output_dir else result_dir / "aligned_full"
    summary = aggregate_aligned_scene(
        result_dir=result_dir,
        output_dir=output_dir,
        conf_percentile=args.conf_percentile,
        edge_margin=args.edge_margin,
        depth_max_ratio=args.depth_max_ratio,
        voxel_size_ratio=args.voxel_size_ratio,
        max_total_points=args.max_total_points,
        scale_mode=args.scale_mode,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
