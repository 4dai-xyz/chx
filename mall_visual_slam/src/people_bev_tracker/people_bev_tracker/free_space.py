"""V2: free-space 生成 (三源合并)。

三个来源, 按优先级 OR 合并:
  A. trajectory corridor  — 沿相机走过的路径画一条带状 buffer
  B. camera frustum       — 每 N 帧相机的视锥扇形投到 BEV
  C. semantic floor       — 从 SAM 处理视频 (input_video.mp4_bev.mp4) 里提取
                            黄色/绿色地面像素, 反投到 BEV

所有 mask 都在 "R_align 后的 aligned world" 坐标下的 BEV 栅格里 (与 static_map 一致)。
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# 通用 world → grid
# ---------------------------------------------------------------------------


def world_xy_to_grid_ij(xy: np.ndarray, meta: dict) -> np.ndarray:
    """(N, 2) BEV-aligned world (x, z) → (N, 2) int [px, py]."""
    W = int(meta["width_px"])
    H = int(meta["height_px"])
    r = float(meta["resolution_unit_per_px"])
    ox, oz = float(meta["origin_world"][0]), float(meta["origin_world"][1])
    px = (W / 2 + (xy[:, 0] - ox) / r).astype(np.int64)
    py = (H / 2 - (xy[:, 1] - oz) / r).astype(np.int64)
    return np.stack([px, py], axis=1)


def _in_bounds(ij: np.ndarray, W: int, H: int) -> np.ndarray:
    return (ij[:, 0] >= 0) & (ij[:, 0] < W) & (ij[:, 1] >= 0) & (ij[:, 1] < H)


# ---------------------------------------------------------------------------
# A. trajectory corridor
# ---------------------------------------------------------------------------


def build_trajectory_corridor(
    trajectory_aligned_xz: np.ndarray,
    meta: dict,
    radius_unit: float,
) -> np.ndarray:
    """沿轨迹画粗线, 返回 uint8 mask (0/255)。半径按 DPVO 单位配置。"""
    W = int(meta["width_px"])
    H = int(meta["height_px"])
    r = float(meta["resolution_unit_per_px"])
    thick = max(1, int(round(radius_unit / r)))

    mask = np.zeros((H, W), dtype=np.uint8)
    if trajectory_aligned_xz.shape[0] < 2:
        return mask
    ij = world_xy_to_grid_ij(trajectory_aligned_xz, meta)
    inb = _in_bounds(ij, W, H)
    ij = ij[inb]
    if ij.shape[0] < 2:
        return mask
    pts_np = ij.astype(np.int32).reshape(-1, 1, 2)
    cv2.polylines(mask, [pts_np], isClosed=False, color=255, thickness=thick)
    # 端点封头
    for p in [ij[0], ij[-1]]:
        cv2.circle(mask, tuple(int(v) for v in p), thick // 2 + 1, 255, -1)
    return mask


# ---------------------------------------------------------------------------
# B. camera frustum carving
# ---------------------------------------------------------------------------


def build_camera_frustum_mask(
    poses_T_wc: List[np.ndarray],
    R_align: np.ndarray,
    meta: dict,
    stride_frames: int = 15,
    half_fov_deg: float = 35.0,
    range_unit: float = 0.8,
    forward_sign_try: Tuple[int, ...] = (+1, -1),
) -> Tuple[np.ndarray, int]:
    """按 stride 取相机位姿, 在 BEV 平面画扇形 (三角形近似) mask。

    ``forward_sign_try = (+1, -1)`` 会分别尝试 +z 和 -z 作为相机前向,
    在 BEV 上生成扇形; 最后取两者并集 (以免相机坐标约定不确定导致扇形反向)。

    返回 (mask_uint8, n_frustums_drawn)。
    """
    W = int(meta["width_px"])
    H = int(meta["height_px"])
    r = float(meta["resolution_unit_per_px"])
    ox, oz = float(meta["origin_world"][0]), float(meta["origin_world"][1])

    bev_axes = meta.get("bev_axes", ["x", "z"])
    idx = {"x": 0, "y": 1, "z": 2}
    ax, az = idx[bev_axes[0].lower()], idx[bev_axes[1].lower()]

    mask = np.zeros((H, W), dtype=np.uint8)
    if not poses_T_wc:
        return mask, 0

    half_fov = math.radians(half_fov_deg)
    range_px = range_unit / r

    n_draw = 0
    poses_sel = poses_T_wc[::max(1, stride_frames)]
    for T_wc in poses_sel:
        R_wc = np.asarray(T_wc[:3, :3], dtype=np.float64)
        C_w = np.asarray(T_wc[:3, 3], dtype=np.float64)
        # aligned world
        C_a = R_align @ C_w
        cx_w, cz_w = float(C_a[ax]), float(C_a[az])

        # camera center → grid
        cu = W / 2 + (cx_w - ox) / r
        cv_ = H / 2 - (cz_w - oz) / r

        for s in forward_sign_try:
            fwd_c = np.array([0.0, 0.0, 1.0]) * float(s)
            fwd_w = R_wc @ fwd_c
            fwd_a = R_align @ fwd_w
            fx = float(fwd_a[ax])
            fz = float(fwd_a[az])
            norm = math.hypot(fx, fz)
            if norm < 1e-9:
                continue
            fx /= norm
            fz /= norm

            # 扇形近似 = 三角形 (顶点=cam, 两腰=±half_fov 方向)
            def rot2(dx, dz, theta):
                c = math.cos(theta); s = math.sin(theta)
                return dx * c - dz * s, dx * s + dz * c

            dx_l, dz_l = rot2(fx, fz, +half_fov)
            dx_r, dz_r = rot2(fx, fz, -half_fov)
            # BEV 图像坐标 (px, py) : py = H/2 - (z - oz)/r
            pl_u = cu + dx_l * range_px
            pl_v = cv_ - dz_l * range_px
            pr_u = cu + dx_r * range_px
            pr_v = cv_ - dz_r * range_px

            poly = np.array(
                [[cu, cv_], [pl_u, pl_v], [pr_u, pr_v]],
                dtype=np.int32,
            )
            cv2.fillConvexPoly(mask, poly, 255)
            n_draw += 1

    return mask, n_draw


# ---------------------------------------------------------------------------
# C. semantic floor mask (from SAM overlay video)
# ---------------------------------------------------------------------------


def _project_pixels_to_bev(
    pixels_uv: np.ndarray,        # (N, 2)
    K: np.ndarray,
    T_wc: np.ndarray,
    ground_normal: np.ndarray,
    ground_d: float,
    R_align: np.ndarray,
    meta: dict,
    max_range_unit: float,
    fallback_h: float = 0.1,
    fallback_pitch_deg: float = 15.0,
) -> Optional[np.ndarray]:
    """把一批像素反投到 aligned world 的 BEV grid, 返回 (M, 2) int [px, py]。

    优先用世界系地面; 若 lam<=0 (射线方向不对), 退化到相机系地面
    (fallback_h + fallback_pitch, 与 people 投影一致)。
    """
    if pixels_uv.size == 0:
        return None
    N = pixels_uv.shape[0]
    u = pixels_uv[:, 0]
    v = pixels_uv[:, 1]
    Kinv = np.linalg.inv(K)
    rays_c = (Kinv @ np.stack([u, v, np.ones(N)], axis=0))  # (3, N)
    rays_c = rays_c / (np.linalg.norm(rays_c, axis=0, keepdims=True) + 1e-12)

    R_wc = T_wc[:3, :3]
    C_w = T_wc[:3, 3]

    # 尝试世界系
    rays_w = R_wc @ rays_c        # (3, N)
    denom = ground_normal @ rays_w
    valid_w = np.abs(denom) > 1e-6
    lam_w = np.where(valid_w, -(ground_normal @ C_w + ground_d) / (denom + 1e-12), -1.0)
    # 世界系不合法就退化到相机系
    mask_ok_w = valid_w & (lam_w > 0)

    # 相机系
    pitch = math.radians(float(fallback_pitch_deg))
    g_c = np.array([0.0, math.cos(pitch), math.sin(pitch)])
    denom_c = g_c @ rays_c
    valid_c = denom_c > 1e-6
    lam_c = np.where(valid_c, float(fallback_h) / (denom_c + 1e-12), -1.0)

    # 逐像素选择: 优先世界系
    lam = np.where(mask_ok_w, lam_w, lam_c)
    ok = mask_ok_w | (valid_c & (lam_c > 0))
    ok &= lam <= float(max_range_unit)

    lam = np.clip(lam, 0.0, None)
    # 世界系时 X_w = C_w + lam * ray_w
    Xw_from_world = C_w[:, None] + lam[None, :] * rays_w
    # 相机系时 X_c = lam * ray_c, X_w = R_wc @ X_c + C_w
    Xw_from_camera = R_wc @ (lam[None, :] * rays_c) + C_w[:, None]

    Xw = np.where(mask_ok_w[None, :], Xw_from_world, Xw_from_camera)  # (3, N)
    Xw = Xw[:, ok]
    if Xw.shape[1] == 0:
        return None

    Xa = (R_align @ Xw).T   # (M, 3)
    bev_axes = meta.get("bev_axes", ["x", "z"])
    idx = {"x": 0, "y": 1, "z": 2}
    a = idx[bev_axes[0].lower()]
    b = idx[bev_axes[1].lower()]
    xy = np.stack([Xa[:, a], Xa[:, b]], axis=1)
    ij = world_xy_to_grid_ij(xy, meta)
    W = int(meta["width_px"])
    H = int(meta["height_px"])
    ij = ij[_in_bounds(ij, W, H)]
    return ij


def build_semantic_floor_mask(
    semantic_mask_video_path: str,
    K: np.ndarray,
    poses_T_wc_by_frame: Dict[int, np.ndarray],
    R_align: np.ndarray,
    meta: dict,
    ground_normal: np.ndarray,
    ground_d: float,
    stride_frames: int = 30,
    hsv_lower: Tuple[int, int, int] = (15, 60, 60),
    hsv_upper: Tuple[int, int, int] = (45, 255, 255),
    sample_step_px: int = 12,
    max_range_unit: float = 1.5,
    fallback_h: float = 0.1,
    fallback_pitch_deg: float = 15.0,
) -> Tuple[np.ndarray, int]:
    """读 SAM 处理视频, HSV 抽 floor 像素, 反投到 BEV mask。

    返回 (mask_uint8, n_frames_processed)。
    """
    W = int(meta["width_px"])
    H = int(meta["height_px"])
    mask = np.zeros((H, W), dtype=np.uint8)
    if not Path(semantic_mask_video_path).exists():
        return mask, 0
    if not poses_T_wc_by_frame:
        return mask, 0

    cap = cv2.VideoCapture(semantic_mask_video_path)
    if not cap.isOpened():
        return mask, 0

    frame_idx = 0
    n_ok = 0
    while True:
        ret, bgr = cap.read()
        if not ret:
            break
        if frame_idx % max(1, stride_frames) == 0 and frame_idx in poses_T_wc_by_frame:
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            m = cv2.inRange(hsv, np.asarray(hsv_lower), np.asarray(hsv_upper))
            # 只取图像下半部分, 避免天花板反光被当成地面
            m[: bgr.shape[0] // 2, :] = 0
            ys, xs = np.where(m > 0)
            if xs.size > 0:
                # 下采样
                step = max(1, int(sample_step_px))
                idxs = np.arange(0, xs.size, step)
                pix = np.stack([xs[idxs], ys[idxs]], axis=1).astype(np.float64)
                T_wc = poses_T_wc_by_frame[frame_idx]
                ij = _project_pixels_to_bev(
                    pix, K, T_wc, ground_normal, ground_d, R_align, meta,
                    max_range_unit=max_range_unit,
                    fallback_h=fallback_h,
                    fallback_pitch_deg=fallback_pitch_deg,
                )
                if ij is not None and ij.shape[0]:
                    mask[ij[:, 1], ij[:, 0]] = 255
                    n_ok += 1
        frame_idx += 1
    cap.release()

    # 补小洞 (地面反投常有孔)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
    return mask, n_ok


# ---------------------------------------------------------------------------
# 合并 + 后处理
# ---------------------------------------------------------------------------


def merge_free_masks(
    masks: List[np.ndarray],
    close_kernel: int = 15,
    min_component_area_px: int = 100,
) -> np.ndarray:
    """三源 OR + morphological close + 去小连通域。"""
    if not masks:
        return None
    out = np.zeros_like(masks[0])
    for m in masks:
        if m is None:
            continue
        out = out | m
    if close_kernel and close_kernel > 1:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_kernel, close_kernel))
        out = cv2.morphologyEx(out, cv2.MORPH_CLOSE, k, iterations=1)
    if min_component_area_px > 0:
        n_cc, cc = cv2.connectedComponents((out > 0).astype(np.uint8))
        keep = np.zeros_like(out)
        for lab in range(1, n_cc):
            area = int((cc == lab).sum())
            if area >= min_component_area_px:
                keep[cc == lab] = 255
        out = keep
    return out
