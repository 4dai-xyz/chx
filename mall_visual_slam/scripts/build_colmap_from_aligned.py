#!/usr/bin/env python3
"""
把 aggregate_vggt_aligned.py 输出的全局对齐结果写成 COLMAP sparse 格式，
给 gsplat 3DGS 训练直接用。

对齐数据里的内参是 VGGT 在 518×518 方形 canvas 上的内参，需要换算回
原始 1920×1080 图像下的内参才能配合原图训练。换算逻辑严格按照 VGGT
官方 `load_and_preprocess_images(mode='pad')` 的预处理：长边等比缩放
到 518、短边 round 到 14 的倍数、短边白色 pad 到 518×518。

输出目录结构（gsplat 的 datasets/colmap.py 直接能读）：
  <out>/images/                ← 软链接到原始 PNG（与训练 data_dir 共享）
  <out>/sparse/0/cameras.bin
  <out>/sparse/0/images.bin
  <out>/sparse/0/points3D.bin  ← 空文件，让 gsplat fallback 到 points.ply
  <out>/sparse/0/points.ply    ← 直接复用对齐后的 aligned_full_scene.ply
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import shutil
from pathlib import Path

import numpy as np


REPO = Path(__file__).resolve().parent.parent
THIRDPARTY_PACKAGES = REPO / "thirdparty" / "python_packages"


def _enable_pycolmap() -> None:
    """让 pycolmap 从 thirdparty 路径可见。"""
    if str(THIRDPARTY_PACKAGES) not in sys.path:
        sys.path.insert(0, str(THIRDPARTY_PACKAGES))


def vggt_to_original_intrinsics(
    K_vggt: np.ndarray,
    original_w: int,
    original_h: int,
    canvas_size: int = 518,
    divisor: int = 14,
) -> tuple[np.ndarray, int, int]:
    """把 VGGT 在 518x518 canvas 上的内参换算到原始分辨率。

    匹配 vggt/utils/load_fn.py 的 mode='pad' 逻辑：
      长边 -> canvas_size
      短边 -> round(短 * canvas_size / 长 / divisor) * divisor
      短边再白色 pad 到 canvas_size
    """
    if original_w >= original_h:
        scale = canvas_size / original_w
        scaled_h = max(divisor, int(round(original_h * scale / divisor)) * divisor)
        pad_top = (canvas_size - scaled_h) // 2
        pad_left = 0
        scaled_w = canvas_size
    else:
        scale = canvas_size / original_h
        scaled_w = max(divisor, int(round(original_w * scale / divisor)) * divisor)
        pad_left = (canvas_size - scaled_w) // 2
        pad_top = 0
        scaled_h = canvas_size

    fx_v = float(K_vggt[0, 0])
    fy_v = float(K_vggt[1, 1])
    cx_v = float(K_vggt[0, 2])
    cy_v = float(K_vggt[1, 2])

    fx_orig = fx_v / scale
    fy_orig = fy_v / scale
    cx_orig = (cx_v - pad_left) / scale
    cy_orig = (cy_v - pad_top) / scale

    K_orig = np.array(
        [[fx_orig, 0.0, cx_orig],
         [0.0, fy_orig, cy_orig],
         [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    return K_orig, int(original_w), int(original_h)


def rotation_matrix_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    """把 3x3 旋转矩阵转成 (x, y, z, w) 四元数（pycolmap.Rigid3d 用的约定）。"""
    R = np.asarray(R, dtype=np.float64)
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=np.float64)
    q /= max(np.linalg.norm(q), 1e-12)
    return q


def build_colmap_scene(
    result_dir: Path,
    aligned_subdir: str,
    output_dir: Path,
    canvas_size: int,
) -> dict:
    """主入口：读对齐结果 + 每窗口 summary，写 COLMAP sparse + 复制初始 PLY。"""
    _enable_pycolmap()
    import pycolmap as pc

    aligned_dir = result_dir / aligned_subdir
    aligned_ply = aligned_dir / "aligned_full_scene.ply"
    aligned_traj = aligned_dir / "aligned_camera_trajectory.json"
    if not aligned_ply.exists():
        raise FileNotFoundError(f"找不到对齐点云: {aligned_ply}")
    if not aligned_traj.exists():
        raise FileNotFoundError(f"找不到对齐轨迹: {aligned_traj}")

    # 1) 读对齐轨迹，按 source_frame 去重，留下 107 个相机的 (cam_to_world R, position) 全局位姿。
    traj = json.loads(aligned_traj.read_text(encoding="utf-8"))
    seen: dict[int, dict] = {}
    for cam in traj.get("cameras", []):
        sf = int(cam["source_frame"])
        # 共享帧会出现多次，保留第一次（属于较早窗口，位姿误差小一些）。
        if sf not in seen:
            seen[sf] = cam
    cams_unique = [seen[k] for k in sorted(seen)]
    print(f"对齐相机数: {len(cams_unique)} (按 source_frame 去重)")

    # 2) 从 frame_meta.json 找每张原图的路径和 sample_index -> source_frame 映射。
    frame_meta_path = result_dir / "frame_meta.json"
    if not frame_meta_path.exists():
        raise FileNotFoundError(f"找不到 frame_meta.json: {frame_meta_path}")
    frame_meta = json.loads(frame_meta_path.read_text(encoding="utf-8"))
    frames = frame_meta["frames"]
    original_w = int(frame_meta["width"])
    original_h = int(frame_meta["height"])
    print(f"原始分辨率: {original_w}x{original_h}")

    sf_to_image: dict[int, str] = {}
    for f in frames:
        sf_to_image[int(f["source_frame"])] = f["image_path"]

    # 3) 从任一窗口 summary 拿 VGGT 在 518x518 上的内参（VGGT 多次推理通常一致）。
    window_dirs = sorted((result_dir / "windows").glob("window_*"))
    if not window_dirs:
        raise FileNotFoundError(f"没有窗口结果: {result_dir / 'windows'}")
    first_summary = json.loads((window_dirs[0] / "summary.json").read_text(encoding="utf-8"))
    K_vggt = np.asarray(first_summary["first_intrinsic"], dtype=np.float64)
    print(f"VGGT 内参 (518x518): fx={K_vggt[0,0]:.2f} fy={K_vggt[1,1]:.2f} cx={K_vggt[0,2]:.2f} cy={K_vggt[1,2]:.2f}")

    K_orig, W, H = vggt_to_original_intrinsics(K_vggt, original_w, original_h, canvas_size)
    print(f"原图内参 ({W}x{H}): fx={K_orig[0,0]:.2f} fy={K_orig[1,1]:.2f} cx={K_orig[0,2]:.2f} cy={K_orig[1,2]:.2f}")

    # 4) 准备 COLMAP 输出结构。
    sparse_dir = output_dir / "sparse" / "0"
    images_link_dir = output_dir / "images"
    sparse_dir.mkdir(parents=True, exist_ok=True)
    images_link_dir.mkdir(parents=True, exist_ok=True)

    # 5) 构建 pycolmap.Reconstruction。SIMPLE_PINHOLE 单焦距：取 fx 和 fy 的均值。
    rec = pc.Reconstruction()
    focal = float((K_orig[0, 0] + K_orig[1, 1]) / 2.0)
    cam = pc.Camera(
        camera_id=1,
        model=pc.CameraModelId.SIMPLE_PINHOLE,
        width=W,
        height=H,
        params=np.array([focal, K_orig[0, 2], K_orig[1, 2]], dtype=np.float64),
    )
    rec.add_camera(cam)

    # 6) 把每张图的全局位姿写进 COLMAP（注意：traj.json 里存的 rotation 是 cam_to_world，
    #    COLMAP 要 world_to_cam）。
    next_image_id = 1
    for cam_dict in cams_unique:
        sf = int(cam_dict["source_frame"])
        if sf not in sf_to_image:
            print(f"  跳过 source_frame={sf}：frame_meta 找不到对应原图")
            continue

        original_path = REPO / sf_to_image[sf] if not Path(sf_to_image[sf]).is_absolute() else Path(sf_to_image[sf])
        image_name = original_path.name
        target_link = images_link_dir / image_name
        if not target_link.exists():
            # 软链接更省空间；失败就退回 copy。
            try:
                os.symlink(original_path.resolve(), target_link)
            except OSError:
                shutil.copy2(original_path, target_link)

        # cam_to_world = [R_c2w | t_c2w]; world_to_cam = inv = [R_c2w.T | -R_c2w.T @ t_c2w]
        R_c2w = np.asarray(cam_dict["rotation"], dtype=np.float64)
        t_c2w = np.asarray(cam_dict["position"], dtype=np.float64)
        R_w2c = R_c2w.T
        t_w2c = -R_w2c @ t_c2w

        # pycolmap.Rotation3d 接受 3x3 矩阵更直观（也避免本地实现四元数顺序错乱）。
        rigid = pc.Rigid3d(pc.Rotation3d(R_w2c), t_w2c)
        img = pc.Image(
            name=image_name,
            cam_from_world=rigid,
            camera_id=1,
            id=next_image_id,
        )
        # 关键：不显式 registered=True 的话，write_binary 不会持久化这张图。
        try:
            img.registered = True
        except Exception:  # noqa: BLE001
            # 极少数版本 registered 是只读，这种情况依赖 add_image 默认行为。
            pass
        rec.add_image(img)
        next_image_id += 1

    print(f"写入 {next_image_id - 1} 张相机到 COLMAP。")

    # 7) write_binary 会生成 cameras.bin / images.bin / points3D.bin（空也会有文件头）。
    rec.write_binary(str(sparse_dir))

    # 8) 把对齐 PLY 也复制成 sparse/0/points.ply，作为 3DGS 初始化点云。
    shutil.copy2(aligned_ply, sparse_dir / "points.ply")
    print(f"复制初始点云 -> {sparse_dir / 'points.ply'}")

    # 顺手保存一份摘要，方便排查。
    summary_out = {
        "result_dir": str(result_dir),
        "aligned_dir": str(aligned_dir),
        "output_dir": str(output_dir),
        "images_dir": str(images_link_dir),
        "sparse_dir": str(sparse_dir),
        "num_cameras_written": next_image_id - 1,
        "original_image_size": [W, H],
        "vggt_canvas_size": canvas_size,
        "K_vggt": K_vggt.tolist(),
        "K_orig_simple_pinhole": {
            "focal": focal,
            "cx": float(K_orig[0, 2]),
            "cy": float(K_orig[1, 2]),
        },
        "init_points_ply": str(sparse_dir / "points.ply"),
    }
    (output_dir / "build_colmap_summary.json").write_text(
        json.dumps(summary_out, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return summary_out


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--result-dir", default=str(REPO / "output" / "vggt_aligned_full_run"))
    parser.add_argument("--aligned-subdir", default="aligned_full")
    parser.add_argument(
        "--output-dir",
        default="",
        help="COLMAP 输出根目录；为空时默认 result-dir/aligned_colmap",
    )
    parser.add_argument("--canvas-size", type=int, default=518, help="VGGT 输入 canvas 边长")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result_dir = Path(args.result_dir)
    output_dir = Path(args.output_dir) if args.output_dir else result_dir / "aligned_colmap"
    summary = build_colmap_scene(
        result_dir=result_dir,
        aligned_subdir=args.aligned_subdir,
        output_dir=output_dir,
        canvas_size=args.canvas_size,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
