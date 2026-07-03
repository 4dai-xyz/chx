"""V3: BEV 坐标系方向校准。

问题背景:
--------
V2 的 static_map 用 world →  R_align (地面对齐) → select_bev_axes(["x","z"]) → 像素栅格
生成。这个数学显示是正确的俯视, 但**没有保证与真实世界方向一致**:

* DPVO 世界系没有 north/east 语义;
* R_align 只对齐了地面法向, 没有确定"从上往下看"时的水平朝向和手性;
* 单目 SLAM 有可能整个世界系相对真实商场做了一次镜像 (chirality/handedness);
* 用户观察到"BEV 像仰视图, 左右转都反了" → 大概率是**镜像**, 不是纯旋转。

本模块提供**统一的 2D BEV 坐标变换**, 作用在:

    aligned BEV coordinates
        after  select_bev_axes(R_align @ world_xyz, ["x","z"])
        before world_to_canvas 像素映射

一次配置, 到处生效 — 相机轨迹 / heading / 行人 / static_map grid / free mask
/ obstacle mask / dense pcd / 最终 mp4 都必须走同一 transform。

支持 9 种 transform (`identity` 是无变换, 其它对 (x, y) 做旋转 / 镜像):

    identity           [ x,  y]
    mirror_x           [-x,  y]     x 翻转 → 左右互换
    mirror_y           [ x, -y]     y 翻转 → 前后互换 (仰俯视切换)
    rotate_180         [-x, -y]     旋转 180° (不改手性)
    swap_xy            [ y,  x]     交换 x/y (等价于沿 y=x 对称)
    swap_xy_mirror_x   [-y,  x]     交换后再翻 x
    swap_xy_mirror_y   [ y, -x]     交换后再翻 y (等价 rotate_90_cw)
    rotate_90_cw       [ y, -x]     顺时针 90° (BEV 图像坐标下)
    rotate_90_ccw      [-y,  x]     逆时针 90° (BEV 图像坐标下)

数学备注:
    - 镜像 (mirror_x / mirror_y) 会改变手性, 也就是**改变左右转的方向**。
    - 单独旋转 (rotate_180, rotate_90_cw/ccw) 不改变手性, 只改整体朝向。
    - swap_xy 也会改变手性 (行列交换 = 沿对角线翻)。

因此, 如果"真实左转 → BEV 右转", 优先试 mirror_x / mirror_y /
swap_xy_mirror_x / swap_xy_mirror_y。
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# 1. 坐标层变换
# ---------------------------------------------------------------------------

TRANSFORMS = (
    "identity",
    "mirror_x",
    "mirror_y",
    "rotate_180",
    "swap_xy",
    "swap_xy_mirror_x",
    "swap_xy_mirror_y",
    "rotate_90_cw",
    "rotate_90_ccw",
)


def _xy_transform(name: str) -> Callable[[np.ndarray, np.ndarray], Tuple[np.ndarray, np.ndarray]]:
    """内部: 返回 (x, y) → (x', y') 变换的 numpy vectorised 函数。"""
    n = str(name).lower().strip()
    if n == "identity":
        return lambda x, y: (x, y)
    if n == "mirror_x":
        return lambda x, y: (-x, y)
    if n == "mirror_y":
        return lambda x, y: (x, -y)
    if n == "rotate_180":
        return lambda x, y: (-x, -y)
    if n == "swap_xy":
        return lambda x, y: (y, x)
    if n == "swap_xy_mirror_x":
        return lambda x, y: (-y, x)
    if n == "swap_xy_mirror_y":
        return lambda x, y: (y, -x)
    if n == "rotate_90_cw":
        return lambda x, y: (y, -x)
    if n == "rotate_90_ccw":
        return lambda x, y: (-y, x)
    raise ValueError(f"unknown bev_alignment transform: {name!r}. "
                     f"valid = {list(TRANSFORMS)}")


def _resolve_cfg(cfg_or_name) -> str:
    """接受 str 或 {'transform': str, ...} 形式的 cfg。"""
    if cfg_or_name is None:
        return "identity"
    if isinstance(cfg_or_name, str):
        return cfg_or_name
    if isinstance(cfg_or_name, dict):
        if not bool(cfg_or_name.get("enabled", True)):
            return "identity"
        return str(cfg_or_name.get("transform", "identity"))
    raise TypeError(f"unsupported bev_alignment cfg type: {type(cfg_or_name)}")


def apply_bev_alignment_xy(xy: np.ndarray, cfg) -> np.ndarray:
    """对 aligned BEV 坐标应用 transform (不是像素坐标)。

    输入:
      xy  形状 (..., 2), 最后一维 = [x, y]
      cfg 可以是 str, 或 {'enabled': bool, 'transform': str} dict

    输出:
      xy_transformed  同形状 (..., 2)
    """
    xy = np.asarray(xy, dtype=np.float64)
    if xy.size == 0:
        return xy.copy()
    name = _resolve_cfg(cfg)
    fn = _xy_transform(name)
    x = xy[..., 0]
    y = xy[..., 1]
    xn, yn = fn(x, y)
    return np.stack([xn, yn], axis=-1)


def apply_bev_alignment_heading(vec_xy: np.ndarray, cfg) -> np.ndarray:
    """对 heading / velocity 类向量应用旋转/镜像, **不做 translation**。

    在这个版本里 translation 恒为 0, 所以 heading 变换和 xy 变换公式一致。
    单独封装是为了将来引入非 0 translation 时不误伤 heading。
    """
    return apply_bev_alignment_xy(vec_xy, cfg)


# ---------------------------------------------------------------------------
# 2. 已经 rasterized 的 grid 图像变换 (debug 用)
# ---------------------------------------------------------------------------


def transform_grid_image(grid: np.ndarray, transform: str) -> np.ndarray:
    """对已经栅格化的 (H, W) 或 (H, W, C) 图像应用同名 transform。

    ⚠️ 只用于 debug 显示。**推荐做法**是在坐标层调用
    ``apply_bev_alignment_xy`` 之后重新 rasterize, 因为像素级 flip 会引入
    半像素误差, 也不能同时正确变换 canvas 中心点等语义。

    对应关系 (BEV 图像 py 向下, 世界 y 向上 → 像素 flip 语义是反的):

        坐标 mirror_x  → 像素 fliplr (左右翻)
        坐标 mirror_y  → 像素 flipud (上下翻)
        坐标 rotate_180 → cv2.rotate(ROTATE_180)
        坐标 rotate_90_cw (世界系顺时针) → 像素 cv2.rotate(ROTATE_90_COUNTERCLOCKWISE)
                                              (因为像素 py 向下)
        坐标 rotate_90_ccw → cv2.rotate(ROTATE_90_CLOCKWISE)
        坐标 swap_xy (沿 y=x 对称) → 转置
        坐标 swap_xy_mirror_x → 转置 + 上下翻 (先 swap 后再 mirror_x, 但像素 py 反向)
        坐标 swap_xy_mirror_y → 转置 + 左右翻
    """
    n = str(transform).lower().strip()
    if n == "identity":
        return grid.copy()
    if n == "mirror_x":
        return np.ascontiguousarray(grid[:, ::-1])
    if n == "mirror_y":
        return np.ascontiguousarray(grid[::-1, :])
    if n == "rotate_180":
        return cv2.rotate(grid, cv2.ROTATE_180)
    if n == "swap_xy":
        # 转置 (对角线翻转), 支持 2D + 3D
        if grid.ndim == 2:
            return np.ascontiguousarray(grid.T)
        return np.ascontiguousarray(np.transpose(grid, (1, 0, 2)))
    if n == "swap_xy_mirror_x":
        t = grid.T if grid.ndim == 2 else np.transpose(grid, (1, 0, 2))
        return np.ascontiguousarray(t[::-1, :])   # 先 swap 再上下翻
    if n == "swap_xy_mirror_y":
        t = grid.T if grid.ndim == 2 else np.transpose(grid, (1, 0, 2))
        return np.ascontiguousarray(t[:, ::-1])
    if n == "rotate_90_cw":
        # 世界系 rotate_90_cw → 像素 rotate_90_counterclockwise (py 向下)
        return cv2.rotate(grid, cv2.ROTATE_90_COUNTERCLOCKWISE)
    if n == "rotate_90_ccw":
        return cv2.rotate(grid, cv2.ROTATE_90_CLOCKWISE)
    raise ValueError(f"unknown transform: {transform!r}")


# ---------------------------------------------------------------------------
# 3. 候选图生成 (给 calibrate_bev_alignment.py 用)
# ---------------------------------------------------------------------------


def world_xy_to_grid_ij(xy: np.ndarray, meta: dict) -> np.ndarray:
    """(N, 2) BEV-aligned world (x, y) → (N, 2) int [px, py]。

    与 static_map / bev_canvas 保持一致:
       px = W/2 + (x - ox) / r
       py = H/2 - (y - oy) / r   (py 向下, 世界 y 向上)
    """
    W = int(meta["width_px"])
    H = int(meta["height_px"])
    r = float(meta["resolution_unit_per_px"])
    ox, oz = float(meta["origin_world"][0]), float(meta["origin_world"][1])
    xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    if xy.size == 0:
        return np.zeros((0, 2), dtype=np.int64)
    px = (W / 2 + (xy[:, 0] - ox) / r).astype(np.int64)
    py = (H / 2 - (xy[:, 1] - oz) / r).astype(np.int64)
    return np.stack([px, py], axis=1)


def _static_layer_tricolor(grid: np.ndarray) -> np.ndarray:
    """grid (0/127/255) → BGR 三值图 (occupied=黑, free=白, unknown=灰)。"""
    H, W = grid.shape
    img = np.full((H, W, 3), (160, 160, 160), dtype=np.uint8)
    img[grid == 127] = (255, 255, 255)
    img[grid == 255] = (0, 0, 0)
    return img


def render_alignment_candidate(
    transform: str,
    grid: np.ndarray,
    meta: dict,
    camera_bev_xy: np.ndarray,
    heading_bev_xy_list: List[Tuple[np.ndarray, np.ndarray]],
    people_final_positions: Optional[np.ndarray] = None,
    active_bbox: Optional[Dict[str, int]] = None,
) -> np.ndarray:
    """渲染单张候选图。

    输入所有 BEV 坐标都是"aligned world" (未经方向 transform)。
    本函数内部先做 transform, 再重新 rasterize + 绘制。

    参数:
      transform:              9 种之一
      grid:                   V2 static_map.npy (H, W) uint8
      meta:                   V2 static_map_meta.json (含 width/height/resolution/origin)
      camera_bev_xy:          (N, 2) 相机轨迹在 aligned BEV
      heading_bev_xy_list:    每 M 帧一个 (cam_xy, heading_vec) 元组用来画箭头
      people_final_positions: (K, 2) 或 None, 每人最终 BEV 位置
      active_bbox:            (可选) V2 active_bbox, 用来画一个虚线框帮助定位
    """
    # 1) 先变换 grid 到目标方向
    grid_t = transform_grid_image(grid, transform)
    static_bgr = _static_layer_tricolor(grid_t)
    H, W = grid_t.shape[:2]

    # 2) 若变换后画布长宽比改了 (rotate_90 / swap_xy), 建一个变换后 meta
    meta_t = dict(meta)
    if transform in ("swap_xy", "swap_xy_mirror_x", "swap_xy_mirror_y",
                     "rotate_90_cw", "rotate_90_ccw"):
        meta_t["width_px"] = W
        meta_t["height_px"] = H
        # origin 也要交换 (x ↔ y) — 因为 select_bev_axes 后 transform 到新 xy 空间,
        # 新 xy 的 origin 也应变换
    # 对 origin 应用 transform (点也走)
    ox, oy = float(meta["origin_world"][0]), float(meta["origin_world"][1])
    ox_t, oy_t = _xy_transform(transform)(np.array([ox]), np.array([oy]))
    meta_t["origin_world"] = [float(ox_t[0]), float(oy_t[0])]

    # 3) 对坐标层数据应用同一 transform
    cam_t = apply_bev_alignment_xy(camera_bev_xy, transform)
    cam_ij = world_xy_to_grid_ij(cam_t, meta_t)

    # 过滤画布外
    def _clip(ij):
        return ij[(ij[:, 0] >= 0) & (ij[:, 0] < W)
                  & (ij[:, 1] >= 0) & (ij[:, 1] < H)]
    cam_ij_in = _clip(cam_ij)

    # 画轨迹
    img = static_bgr.copy()
    if cam_ij_in.shape[0] >= 2:
        pts = cam_ij_in.astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(img, [pts], False, (255, 128, 0), thickness=3,
                      lineType=cv2.LINE_AA)

    # 起点绿, 终点红 (BGR)
    if cam_ij.shape[0] >= 1:
        start = tuple(int(v) for v in cam_ij[0])
        end = tuple(int(v) for v in cam_ij[-1])
        cv2.circle(img, start, 12, (0, 200, 0), -1, cv2.LINE_AA)   # green = start
        cv2.putText(img, "START", (start[0] + 14, start[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 160, 0), 2, cv2.LINE_AA)
        cv2.circle(img, end, 12, (0, 0, 220), -1, cv2.LINE_AA)     # red = end
        cv2.putText(img, "END", (end[0] + 14, end[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 180), 2, cv2.LINE_AA)

    # heading arrows
    for cam_xy, head_xy in heading_bev_xy_list:
        cxy = apply_bev_alignment_xy(np.asarray(cam_xy).reshape(1, 2), transform)
        hxy = apply_bev_alignment_heading(np.asarray(head_xy).reshape(1, 2), transform)
        cij = world_xy_to_grid_ij(cxy, meta_t)
        if cij.shape[0] == 0:
            continue
        cx, cy = int(cij[0, 0]), int(cij[0, 1])
        if not (0 <= cx < W and 0 <= cy < H):
            continue
        hx, hy = float(hxy[0, 0]), float(hxy[0, 1])
        nh = math.hypot(hx, hy)
        if nh < 1e-9:
            continue
        hx /= nh; hy /= nh
        arrow_px = 28
        # heading 的 y 也是"世界系向上", 所以像素 py 减去 hy
        ex = cx + int(round(hx * arrow_px))
        ey = cy - int(round(hy * arrow_px))
        cv2.arrowedLine(img, (cx, cy), (ex, ey), (255, 128, 0),
                        2, cv2.LINE_AA, tipLength=0.35)

    # 行人 (可选)
    if people_final_positions is not None and people_final_positions.shape[0]:
        ppl = apply_bev_alignment_xy(people_final_positions, transform)
        ppl_ij = world_xy_to_grid_ij(ppl, meta_t)
        for p in ppl_ij:
            if 0 <= p[0] < W and 0 <= p[1] < H:
                cv2.circle(img, tuple(int(v) for v in p), 6, (0, 120, 255),
                           -1, cv2.LINE_AA)

    # active_bbox (可选虚线框)
    if active_bbox is not None:
        x0, y0, x1, y1 = active_bbox["x0"], active_bbox["y0"], active_bbox["x1"], active_bbox["y1"]
        # bbox 是像素坐标, 也要跟着 grid transform 变;
        # 简化处理: 把 4 个角当点, 应用像素级变换
        # (仅做视觉辅助, 不用于任何几何计算)
        # skip: 现阶段不画 active_bbox, 避免混淆

    # HUD: 左上角 transform 名字 + 说明
    HUD_H = 44
    hud = np.zeros((HUD_H, W, 3), dtype=np.uint8)
    hud[:] = (20, 20, 20)
    cv2.putText(hud, f"BEV alignment candidate: {transform}",
                (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (240, 240, 240), 2, cv2.LINE_AA)
    img_with_hud = np.vstack([hud, img])
    return img_with_hud


# ---------------------------------------------------------------------------
# 4. 从 v2 outputs 采样出候选图需要的数据
# ---------------------------------------------------------------------------


def load_camera_bev_and_headings(
    camera_json_path: str,
    R_align_3x3: np.ndarray,
    bev_axes: Tuple[str, str] = ("x", "z"),
    heading_stride: int = 100,
) -> Tuple[np.ndarray, List[Tuple[np.ndarray, np.ndarray]]]:
    """读 V2 camera_trajectory_route_A_v2.json → (traj_bev, heading_samples)。

    注: V2 JSON 已经把 aligned BEV 保存在 poses[i].bev_xy 里。所以直接用就行,
    但为了保险 (以及支持没保存 bev_xy 的 case), 我们同时用 T_wc + R_align 重算 heading。
    """
    with open(camera_json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    poses = data.get("poses", [])
    if not poses:
        return np.zeros((0, 2)), []

    idx = {"x": 0, "y": 1, "z": 2}
    ax = idx[bev_axes[0].lower()]
    ay = idx[bev_axes[1].lower()]

    traj_bev: List[List[float]] = []
    headings: List[Tuple[np.ndarray, np.ndarray]] = []
    for i, pose in enumerate(poses):
        # 优先 bev_xy (V2 已算好); 兜底 T_wc
        if "bev_xy" in pose and pose["bev_xy"] is not None:
            xy = pose["bev_xy"]
            traj_bev.append([float(xy[0]), float(xy[1])])
        else:
            T = np.asarray(pose["T_wc"], dtype=np.float64)
            C_a = R_align_3x3 @ T[:3, 3]
            traj_bev.append([float(C_a[ax]), float(C_a[ay])])
        # heading: 每 stride 帧取一次
        if i % max(1, heading_stride) == 0:
            T = np.asarray(pose["T_wc"], dtype=np.float64)
            C_a = R_align_3x3 @ T[:3, 3]
            fwd_w = T[:3, :3] @ np.array([0.0, 0.0, 1.0])
            fwd_a = R_align_3x3 @ fwd_w
            headings.append((
                np.array([float(C_a[ax]), float(C_a[ay])]),
                np.array([float(fwd_a[ax]), float(fwd_a[ay])]),
            ))
    return np.asarray(traj_bev, dtype=np.float64), headings


def bake_alignment_into_map(
    grid: np.ndarray,
    meta: dict,
    transform: str,
    source: str = "manual",
) -> Tuple[np.ndarray, dict]:
    """把 transform 一次性烘焙到 grid + origin, 输出的 (new_grid, new_meta)
    再配合 pipeline 中对**坐标数据**同名 transform, 就能得到一致的对齐地图。

    做法:
      new_grid = transform_grid_image(grid, transform)
      new_origin = apply_bev_alignment_xy(origin, transform)
      width/height 若发生了旋转 (rotate_90 / swap_xy 家族), 交换

    烘焙后:
      * static_map.npy (new_grid) 已经是 aligned 状态
      * static_map_meta.json 里加 bev_alignment 记录
      * pipeline 拿到 new_meta, **仍需要对相机/行人坐标应用同名 transform**
        (因为 world_to_canvas 用的是 new_origin, 而 pipeline 输入的坐标
         还是 old-frame BEV, 必须先 transform 到 new frame 才能落到正确像素)

    参数:
      grid:       (H, W) uint8, 值 0/127/255
      meta:       V2 static_map_meta.json 内容
      transform:  9 种之一
      source:     用于记录到 meta.bev_alignment.source
    """
    new_grid = transform_grid_image(grid, transform)
    new_meta = json.loads(json.dumps(meta))   # deep copy via json

    # 1) origin 也 transform (关键)
    ox, oy = float(meta["origin_world"][0]), float(meta["origin_world"][1])
    ox_t, oy_t = _xy_transform(transform)(np.array([ox]), np.array([oy]))
    new_meta["origin_world"] = [float(ox_t[0]), float(oy_t[0])]

    # 2) 宽高: rotate_90 / swap_xy 系列会交换 W/H
    if transform in ("swap_xy", "swap_xy_mirror_x", "swap_xy_mirror_y",
                     "rotate_90_cw", "rotate_90_ccw"):
        new_meta["width_px"] = int(meta["height_px"])
        new_meta["height_px"] = int(meta["width_px"])

    # 3) 记录 bev_alignment 追溯信息
    new_meta["bev_alignment"] = {
        "enabled": True,
        "transform": transform,
        "source": source,
        "origin_before_bake": [ox, oy],
        "origin_after_bake": new_meta["origin_world"],
        "width_before_bake": int(meta["width_px"]),
        "height_before_bake": int(meta["height_px"]),
        "width_after_bake": int(new_meta["width_px"]),
        "height_after_bake": int(new_meta["height_px"]),
    }
    return new_grid, new_meta


def load_alignment_selected(path: str) -> dict:
    """读 alignment_selected.json, 返回 dict, 缺失 / 关闭 → identity。"""
    p = Path(path)
    if not p.exists():
        return {"selected_transform": "identity", "enabled": False,
                "source": "missing", "reason": f"{path} not found"}
    d = json.loads(p.read_text(encoding="utf-8"))
    return {
        "selected_transform": str(d.get("selected_transform", "identity")),
        "enabled": bool(d.get("enabled", True)),
        "source": str(d.get("source", "manual")),
        "reason": str(d.get("reason", "")),
        "confirmed_at": d.get("confirmed_at", None),
    }


def alignment_cfg_from_meta(meta: dict) -> dict:
    """从 static_map_meta.json 里读 bev_alignment 段, 转成
    apply_bev_alignment_xy 可用的 dict。"""
    a = meta.get("bev_alignment") or {}
    return {
        "enabled": bool(a.get("enabled", False)),
        "transform": str(a.get("transform", "identity")),
    }


def load_people_final_positions(
    people_json_path: str,
    use_last_frames: int = 50,
) -> np.ndarray:
    """最后 N 帧内出现的 active 人员的 filtered_bev_xy, 用于候选图叠加。"""
    with open(people_json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    frames = data.get("frames", [])
    if not frames:
        return np.zeros((0, 2))
    picked: Dict[int, list] = {}
    for f in frames[-max(1, use_last_frames):]:
        for p in f.get("people", []):
            if not p.get("projected"):
                continue
            xy = p.get("filtered_bev_xy") or p.get("bev_xy")
            if not xy:
                continue
            picked[int(p["track_id"])] = xy
    pts = np.asarray(list(picked.values()), dtype=np.float64) if picked \
        else np.zeros((0, 2))
    return pts
