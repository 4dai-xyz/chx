"""点云 → 2D BEV occupancy grid.

流程:
1. 把点云 + 相机轨迹 rotate 到 "地面对齐" 坐标 (地面法向 → 世界 +Y)。
2. 高度过滤: 保留 obstacle band (h_min < y < h_max) 的点。
3. 2D 直方图 (x, z) → 每个 cell 累积点数。
4. 阈值化: cell 点数 > τ 视为 occupied。
5. free 只来自相机走过的路径附近 buffer (对未观测区域不推断为 free)。
6. 剩下 = unknown。
7. 形态学 dilate 让墙"厚"一点; 可选 Gaussian blur 平滑 cost。
8. 渲染成 uint8 三色图 + 保存 meta。

栅格坐标约定:
  cell (i, j)  ↔  world (x, z) = (origin_x + (i - W/2) * r,  origin_z + (j - H/2) * r)
  图像像素 (px, py) = (i,  H - 1 - j)   # py 向下, 世界 z 向上

因此 static_map 与 bev_canvas.world_to_canvas 需要坐标一致 (bev_axes=["x","z"] 时)。
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np


def _rot_matrix_align_a_to_b(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Rodrigues: 找 R 使 R a = b (单位向量)."""
    a = a / (np.linalg.norm(a) + 1e-12)
    b = b / (np.linalg.norm(b) + 1e-12)
    c = float(np.dot(a, b))
    if c > 0.9999:
        return np.eye(3)
    if c < -0.9999:
        # 180 度: 随便选一个和 a 不共线的 axis
        axis = np.array([1.0, 0.0, 0.0])
        if abs(a[0]) > 0.9:
            axis = np.array([0.0, 1.0, 0.0])
        axis = axis - a * np.dot(axis, a)
        axis /= np.linalg.norm(axis)
        K = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0],
        ])
        return -np.eye(3) + 2 * (K @ K)
    axis = np.cross(a, b)
    axis /= np.linalg.norm(axis)
    s = float(np.linalg.norm(np.cross(a, b)))
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0],
    ])
    return np.eye(3) + s * K + (1 - c) * (K @ K)


def _select_bev_axes(pts_aligned: np.ndarray, bev_axes: Tuple[str, str]) -> np.ndarray:
    """从 (N, 3) aligned world 选两个分量当 BEV (x, z)."""
    idx = {"x": 0, "y": 1, "z": 2}
    a = idx[bev_axes[0].lower()]
    b = idx[bev_axes[1].lower()]
    return np.stack([pts_aligned[:, a], pts_aligned[:, b]], axis=1)


def build_static_map(
    points_world: np.ndarray,
    trajectory_world_xyz: np.ndarray,
    ground_plane: Dict,
    cfg: dict,
) -> Tuple[np.ndarray, Dict]:
    """点云 + 相机轨迹 → occupancy grid (uint8 W×H, 值 0=unknown, 127=free, 255=occupied).

    ``points_world``:            (N, 3) DPVO 世界系点云
    ``trajectory_world_xyz``:    (M, 3) DPVO 世界系相机位置
    ``ground_plane``:            {normal: [nx,ny,nz], d: float, ...}
    ``cfg``:                     route_A.yaml -> static_map 段

    返回:
        grid_uint8, meta
    """
    resolution = float(cfg["resolution_unit_per_px"])
    W = int(cfg["width_px"])
    H = int(cfg["height_px"])
    origin_world = np.asarray(cfg["origin_world"], dtype=np.float64).reshape(2)  # (ox, oz)
    bev_axes = tuple(cfg.get("bev_axes", ["x", "z"]))
    hmin, hmax = cfg["obstacle_height_range"]
    hmin = float(hmin); hmax = float(hmax)
    auto_height = bool(cfg.get("auto_height", False))
    count_thresh = int(cfg["count_thresh"])
    dilate_kernel = int(cfg.get("dilate_kernel", 3))
    blur_sigma = float(cfg.get("gaussian_blur_sigma", 0.0))
    corridor_r = int(cfg.get("free_corridor_radius_px", 12))

    # ---------- Step 1: 变到 "地面对齐" 坐标 ----------
    n = np.asarray(ground_plane["normal"], dtype=np.float64).reshape(3)
    d = float(ground_plane["d"])
    n = n / (np.linalg.norm(n) + 1e-12)
    R_align = _rot_matrix_align_a_to_b(n, np.array([0.0, 1.0, 0.0]))

    pts = (R_align @ points_world.T).T                       # (N, 3)
    # 高度 = 变换后的 Y 分量; 因为 R_align n = +Y, 所以 n·X = (R_align.T @ +Y) · X = +Y · (R_align X)
    # 即高度 h_align = pts[:,1] + d (d 在原系里的偏移, R 不改 d 的物理含义? 严格上应该也旋转 d)
    # 更严格: h = n·X_world + d; 而 R_align n = e_y, R_align X_world = pts, 所以
    # n·X_world = (R_align.T e_y)·X_world = e_y·(R_align X_world) = pts[:,1]
    # 所以 h = pts[:,1] + d
    h_align = pts[:, 1] + d

    traj = (R_align @ trajectory_world_xyz.T).T              # (M, 3)

    # ---------- Step 2: 高度过滤 (障碍带) ----------
    if auto_height:
        # 用高度直方图找 ground level = h 中众数, obstacle band = [gl + hmin*scale, gl + hmax*scale]
        # 但 h_align 已经是相对地面的 signed distance, ground level 应该在 0 附近。
        # 更稳: 我们用 abs(h_align) 的 5% 分位当"极靠近地面"厚度, 障碍带用
        # [3 * base, 30 * base] 之类的相对宽度。
        abs_h = np.abs(h_align)
        base = float(np.percentile(abs_h, 5)) if abs_h.size else 0.01
        base = max(base, 1e-4)
        hmin_used = 3.0 * base
        hmax_used = 30.0 * base
    else:
        hmin_used = hmin
        hmax_used = hmax
    mask_band = (h_align >= hmin_used) & (h_align <= hmax_used)
    pts_obst = pts[mask_band]
    n_obst_pts = int(pts_obst.shape[0])

    # 拿 (x, z) 做 BEV
    xz_obst = _select_bev_axes(pts_obst, bev_axes)           # (n_obst, 2)
    xz_traj = _select_bev_axes(traj, bev_axes)               # (M, 2)

    # ---------- Step 3: 世界 (x, z) → 图像 (px, py) ----------
    ox, oz = float(origin_world[0]), float(origin_world[1])

    def world_to_grid(xy: np.ndarray) -> np.ndarray:
        """返回 (K, 2) int, [px, py]."""
        px = (W / 2 + (xy[:, 0] - ox) / resolution).astype(np.int64)
        py = (H / 2 - (xy[:, 1] - oz) / resolution).astype(np.int64)
        return np.stack([px, py], axis=1)

    obst_ij = world_to_grid(xz_obst) if len(xz_obst) else np.zeros((0, 2), np.int64)
    traj_ij = world_to_grid(xz_traj) if len(xz_traj) else np.zeros((0, 2), np.int64)

    # 剔除超出画布
    def inside(ij):
        return (ij[:, 0] >= 0) & (ij[:, 0] < W) & (ij[:, 1] >= 0) & (ij[:, 1] < H)
    obst_ij = obst_ij[inside(obst_ij)]
    traj_ij = traj_ij[inside(traj_ij)]

    # ---------- Step 4: 累积计数 ----------
    count = np.zeros((H, W), dtype=np.int32)
    if len(obst_ij):
        np.add.at(count, (obst_ij[:, 1], obst_ij[:, 0]), 1)

    occupied = (count >= count_thresh)

    # ---------- Step 5: free corridor from trajectory ----------
    import cv2
    free_mask = np.zeros((H, W), dtype=np.uint8)
    if len(traj_ij):
        # 画连续 polyline
        pts_np = traj_ij.astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(free_mask, [pts_np], isClosed=False, color=255,
                      thickness=max(1, corridor_r))
    free = free_mask > 0

    # 冲突: 相机走过但障碍点也堆积 → 保留 free
    occupied = occupied & (~free)

    # ---------- Step 6: 形态学 ----------
    if dilate_kernel and dilate_kernel > 1:
        k = np.ones((dilate_kernel, dilate_kernel), dtype=np.uint8)
        occupied_u8 = occupied.astype(np.uint8) * 255
        occupied_u8 = cv2.dilate(occupied_u8, k, iterations=1)
        occupied = occupied_u8 > 0

    # ---------- Step 7: 组装 grid (0/127/255) ----------
    grid = np.zeros((H, W), dtype=np.uint8)      # 0 = unknown
    grid[free] = 127
    grid[occupied] = 255

    total = H * W
    stats = {
        "occupied_count": int(occupied.sum()),
        "free_count": int(free.sum() & ~occupied.sum() if False else int(free.sum())),
        "unknown_count": int(total - int(occupied.sum()) - int(free.sum())),
        "occupied_ratio": float(occupied.sum() / total),
        "free_ratio": float(free.sum() / total),
        "unknown_ratio": float(1.0 - (occupied.sum() + free.sum()) / total),
        "n_obstacle_points_after_height_filter": n_obst_pts,
        "n_points_input": int(points_world.shape[0]),
    }

    # cost (0-1 float) for 可选高斯 blur
    if blur_sigma > 0:
        cost = (occupied.astype(np.float32))
        cost = cv2.GaussianBlur(cost, (0, 0), sigmaX=blur_sigma, sigmaY=blur_sigma)
        stats["cost_max"] = float(cost.max())
    else:
        cost = occupied.astype(np.float32)

    meta = {
        "resolution_unit_per_px": resolution,
        "width_px": W,
        "height_px": H,
        "origin_world": [ox, oz],
        "bev_axes": list(bev_axes),
        "obstacle_height_range": [hmin_used, hmax_used],
        "obstacle_height_range_config": [hmin, hmax],
        "auto_height_used": bool(auto_height),
        "count_thresh": count_thresh,
        "dilate_kernel": dilate_kernel,
        "gaussian_blur_sigma": blur_sigma,
        "free_corridor_radius_px": corridor_r,
        "ground_plane": {"normal": n.tolist(), "d": d},
        "R_align": R_align.tolist(),
        "colors": {
            "unknown": list(cfg.get("colors", {}).get("unknown", [35, 35, 35])),
            "free":    list(cfg.get("colors", {}).get("free",    [205, 205, 205])),
            "occupied":list(cfg.get("colors", {}).get("occupied",[70, 70, 70])),
        },
        "statistics": stats,
    }
    return grid, meta


def render_static_map(grid: np.ndarray, meta: dict) -> np.ndarray:
    """把 (H, W) uint8 grid (0/127/255) 渲染成 (H, W, 3) BGR 图。"""
    H, W = grid.shape
    img = np.zeros((H, W, 3), dtype=np.uint8)
    c = meta.get("colors", {})
    unk = np.array(c.get("unknown", [35, 35, 35]), dtype=np.uint8)
    fre = np.array(c.get("free",    [205, 205, 205]), dtype=np.uint8)
    occ = np.array(c.get("occupied",[70, 70, 70]), dtype=np.uint8)
    img[:] = unk
    img[grid == 127] = fre
    img[grid == 255] = occ
    return img


def save_static_map(
    grid: np.ndarray,
    meta: dict,
    npy_path: str,
    png_path: str,
    meta_path: str,
) -> None:
    import cv2, json
    Path(npy_path).parent.mkdir(parents=True, exist_ok=True)
    np.save(npy_path, grid)
    img = render_static_map(grid, meta)
    cv2.imwrite(png_path, img)
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)


# ===========================================================================
#  V2: build_static_map_v2 + multi-mode renders + intermediate debug images
# ===========================================================================


def _cv2_close(mask_u8: np.ndarray, ksize: int) -> np.ndarray:
    if ksize <= 1:
        return mask_u8
    import cv2
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
    return cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, k, iterations=1)


def _remove_small_components(mask_bool: np.ndarray, min_area: int) -> np.ndarray:
    if min_area <= 0:
        return mask_bool
    import cv2
    n_cc, cc = cv2.connectedComponents(mask_bool.astype(np.uint8))
    keep = np.zeros_like(mask_bool)
    for lab in range(1, n_cc):
        if int((cc == lab).sum()) >= min_area:
            keep[cc == lab] = True
    return keep


def build_static_map_v2(
    points_world: np.ndarray,
    trajectory_world_xyz: np.ndarray,
    ground_plane: Dict,
    cfg: dict,
    poses_T_wc_by_frame: Optional[Dict[int, np.ndarray]] = None,
    K: Optional[np.ndarray] = None,
    semantic_mask_video: Optional[str] = None,
    return_debug: bool = False,
) -> Tuple[np.ndarray, Dict]:
    """V2 版: 点云 → 障碍 (密度 + 平滑 + 闭运算 + 去小连通域 + 膨胀);
    trajectory + camera frustum + semantic-floor → free; 冲突消解 → grid.

    ``cfg`` 应传入 route_A_v2.yaml 里的 ``static_map_v2`` 段, **加上以下**
    单值字段 (由 tune 脚本决定):
        resolution_unit_per_px, width_px, height_px, origin_world, bev_axes,
        obstacle_height_range,
        obstacle_count_thresh, obstacle_gaussian_sigma, obstacle_close_kernel,
        obstacle_dilate_kernel, obstacle_density_percentile,
        free_corridor_radius_unit, free_frustum_*, free_close_kernel,
        use_semantic_floor_mask, floor_*
    """
    import cv2
    from .free_space import (
        build_trajectory_corridor,
        build_camera_frustum_mask,
        build_semantic_floor_mask,
        merge_free_masks,
        world_xy_to_grid_ij,
    )

    resolution = float(cfg["resolution_unit_per_px"])
    W = int(cfg["width_px"])
    H = int(cfg["height_px"])
    origin_world = np.asarray(cfg["origin_world"], dtype=np.float64).reshape(2)
    bev_axes = tuple(cfg.get("bev_axes", ["x", "z"]))

    hmin, hmax = cfg["obstacle_height_range"]
    hmin = float(hmin); hmax = float(hmax)
    count_thresh = int(cfg.get("obstacle_count_thresh", 1))
    density_percentile = float(cfg.get("obstacle_density_percentile", 0) or 0)
    gauss_sigma = float(cfg.get("obstacle_gaussian_sigma", 1.0))
    close_kernel = int(cfg.get("obstacle_close_kernel", 9))
    dilate_kernel = int(cfg.get("obstacle_dilate_kernel", 5))
    obst_min_comp = int(cfg.get("obstacle_min_component_area_px", 20))

    corridor_r_unit = float(cfg.get("free_corridor_radius_unit", 0.20))
    frustum_enable = bool(cfg.get("free_frustum_enable", True))
    frustum_stride = int(cfg.get("free_frustum_stride_frames", 15))
    frustum_range = float(cfg.get("free_frustum_range_unit", 0.80))
    frustum_hfov = float(cfg.get("free_frustum_half_fov_deg", 35.0))
    free_close_k = int(cfg.get("free_close_kernel", 15))
    free_min_comp = int(cfg.get("free_min_component_area_px", 100))

    use_semantic = bool(cfg.get("use_semantic_floor_mask", False)) and \
        semantic_mask_video is not None and Path(semantic_mask_video).exists()

    # ---------- Step 1: R_align + aligned coords ----------
    n = np.asarray(ground_plane["normal"], dtype=np.float64).reshape(3)
    d = float(ground_plane["d"])
    n = n / (np.linalg.norm(n) + 1e-12)
    R_align = _rot_matrix_align_a_to_b(n, np.array([0.0, 1.0, 0.0]))
    pts_a = (R_align @ points_world.T).T
    traj_a = (R_align @ trajectory_world_xyz.T).T
    h_align = pts_a[:, 1] + d

    # ---------- Step 2: obstacle height band ----------
    mask_band = (h_align >= hmin) & (h_align <= hmax)
    pts_obst = pts_a[mask_band]

    # ---------- Step 3: obstacle count histogram ----------
    xz_obst = _select_bev_axes(pts_obst, bev_axes)
    xz_traj = _select_bev_axes(traj_a, bev_axes)

    meta_for_grid = {
        "width_px": W, "height_px": H,
        "resolution_unit_per_px": resolution,
        "origin_world": [float(origin_world[0]), float(origin_world[1])],
        "bev_axes": list(bev_axes),
    }

    obst_ij = world_xy_to_grid_ij(xz_obst, meta_for_grid) if len(xz_obst) else np.zeros((0, 2), np.int64)
    traj_ij = world_xy_to_grid_ij(xz_traj, meta_for_grid) if len(xz_traj) else np.zeros((0, 2), np.int64)

    def _inb(ij):
        return (ij[:, 0] >= 0) & (ij[:, 0] < W) & (ij[:, 1] >= 0) & (ij[:, 1] < H)
    obst_ij = obst_ij[_inb(obst_ij)]
    traj_ij = traj_ij[_inb(traj_ij)]

    count = np.zeros((H, W), dtype=np.int32)
    if len(obst_ij):
        np.add.at(count, (obst_ij[:, 1], obst_ij[:, 0]), 1)

    # ---------- Step 4: density → mask ----------
    if gauss_sigma > 0:
        density = cv2.GaussianBlur(count.astype(np.float32), (0, 0),
                                   sigmaX=gauss_sigma, sigmaY=gauss_sigma)
    else:
        density = count.astype(np.float32)

    obst_raw = np.zeros((H, W), dtype=bool)
    if density_percentile > 0:
        # 只对非零 density 求 percentile, 避免 P90 = 0
        nz = density[density > 0]
        if nz.size > 0:
            thr = float(np.percentile(nz, density_percentile))
            obst_raw = density >= max(thr, 1e-6)
    else:
        obst_raw = count >= count_thresh
    obst_raw_u8 = obst_raw.astype(np.uint8) * 255

    # ---------- Step 5: obstacle regularization ----------
    obst_closed = _cv2_close(obst_raw_u8, close_kernel)
    obst_bool = _remove_small_components(obst_closed > 0, obst_min_comp)
    if dilate_kernel > 1:
        k = np.ones((dilate_kernel, dilate_kernel), dtype=np.uint8)
        obst_bool = cv2.dilate(obst_bool.astype(np.uint8) * 255, k, iterations=1) > 0
    obstacle_mask = obst_bool.astype(np.uint8) * 255

    # ---------- Step 6: free (trajectory corridor + frustum + semantic) ----------
    free_masks: List[np.ndarray] = []

    # A. trajectory corridor
    corridor_mask = build_trajectory_corridor(
        xz_traj, meta_for_grid, radius_unit=corridor_r_unit,
    )
    free_masks.append(corridor_mask)

    # B. camera frustum
    n_frustum = 0
    if frustum_enable and poses_T_wc_by_frame:
        # 有序序列 by frame_index
        poses_sorted = [poses_T_wc_by_frame[k] for k in sorted(poses_T_wc_by_frame.keys())]
        frustum_mask, n_frustum = build_camera_frustum_mask(
            poses_T_wc=poses_sorted,
            R_align=R_align,
            meta=meta_for_grid,
            stride_frames=frustum_stride,
            half_fov_deg=frustum_hfov,
            range_unit=frustum_range,
        )
        free_masks.append(frustum_mask)

    # C. semantic floor
    n_semantic_frames = 0
    semantic_mask = None
    if use_semantic and K is not None and poses_T_wc_by_frame:
        semantic_mask, n_semantic_frames = build_semantic_floor_mask(
            semantic_mask_video_path=semantic_mask_video,
            K=K,
            poses_T_wc_by_frame=poses_T_wc_by_frame,
            R_align=R_align,
            meta=meta_for_grid,
            ground_normal=n,
            ground_d=d,
            stride_frames=int(cfg.get("semantic_stride_frames", 30)),
            hsv_lower=tuple(cfg.get("floor_hsv_lower", [15, 60, 60])),
            hsv_upper=tuple(cfg.get("floor_hsv_upper", [45, 255, 255])),
            sample_step_px=int(cfg.get("floor_sample_step_px", 12)),
            max_range_unit=float(cfg.get("floor_projection_max_range_unit", 1.5)),
        )
        if semantic_mask is not None:
            free_masks.append(semantic_mask)

    merged_free = merge_free_masks(
        [m for m in free_masks if m is not None],
        close_kernel=free_close_k,
        min_component_area_px=free_min_comp,
    )
    if merged_free is None:
        merged_free = np.zeros((H, W), dtype=np.uint8)
    free_mask_bool = merged_free > 0

    # ---------- Step 7: 冲突消解 ----------
    obstacle_mask_bool = obstacle_mask > 0
    # 相机走过的路径 = free, occupied 让路 (trajectory corridor 优先级最高)
    # (但保留 frustum/semantic 内出现的 occupied — 那些是真的墙)
    corridor_bool = corridor_mask > 0
    obstacle_mask_bool = obstacle_mask_bool & (~corridor_bool)

    # free 也让开 occupied (最终二值化)
    free_final = free_mask_bool & (~obstacle_mask_bool)

    # ---------- Step 8: 组装 grid ----------
    grid = np.zeros((H, W), dtype=np.uint8)
    grid[free_final] = 127
    grid[obstacle_mask_bool] = 255

    stats = {
        "occupied_count": int(obstacle_mask_bool.sum()),
        "free_count": int(free_final.sum()),
        "unknown_count": int((grid == 0).sum()),
        "occupied_ratio": float(obstacle_mask_bool.sum() / (H * W)),
        "free_ratio": float(free_final.sum() / (H * W)),
        "unknown_ratio": float((grid == 0).sum() / (H * W)),
        "n_obstacle_points_after_height_filter": int(pts_obst.shape[0]),
        "n_points_input": int(points_world.shape[0]),
        "n_frustum_drawn": int(n_frustum),
        "n_semantic_frames_used": int(n_semantic_frames),
        "semantic_enabled": bool(use_semantic),
    }

    meta = {
        "resolution_unit_per_px": resolution,
        "width_px": W,
        "height_px": H,
        "origin_world": [float(origin_world[0]), float(origin_world[1])],
        "bev_axes": list(bev_axes),
        "obstacle_height_range": [hmin, hmax],
        "obstacle_count_thresh": count_thresh,
        "obstacle_gaussian_sigma": gauss_sigma,
        "obstacle_density_percentile": density_percentile,
        "obstacle_close_kernel": close_kernel,
        "obstacle_dilate_kernel": dilate_kernel,
        "obstacle_min_component_area_px": obst_min_comp,
        "free_corridor_radius_unit": corridor_r_unit,
        "free_frustum_enable": frustum_enable,
        "free_frustum_stride_frames": frustum_stride,
        "free_frustum_range_unit": frustum_range,
        "free_frustum_half_fov_deg": frustum_hfov,
        "free_close_kernel": free_close_k,
        "free_min_component_area_px": free_min_comp,
        "use_semantic_floor_mask": use_semantic,
        "ground_plane": {"normal": n.tolist(), "d": d},
        "R_align": R_align.tolist(),
        "colors": {
            "unknown": [35, 35, 35],
            "free":    [205, 205, 205],
            "occupied":[70, 70, 70],
        },
        "statistics": stats,
    }

    if return_debug:
        debug = {
            "count": count,
            "density": density,
            "obst_raw": obst_raw_u8,
            "obst_regularized": obstacle_mask,
            "free_corridor": corridor_mask,
            "free_frustum": free_masks[1] if len(free_masks) > 1 else None,
            "free_semantic": semantic_mask,
            "free_merged": merged_free,
            "trajectory_ij": traj_ij,
        }
        return grid, meta, debug
    return grid, meta


# ---------------------------------------------------------------------------
# Multi-mode renders
# ---------------------------------------------------------------------------


def render_nav_binary(grid: np.ndarray, colors: Optional[dict] = None) -> np.ndarray:
    """黑白二值: free = 白, else = 黑 (occupied + unknown 都算不可通行)。"""
    colors = colors or {}
    free_bgr = tuple(colors.get("free", [255, 255, 255]))
    not_free_bgr = tuple(colors.get("not_free", [0, 0, 0]))
    H, W = grid.shape
    img = np.zeros((H, W, 3), dtype=np.uint8)
    img[:] = not_free_bgr
    img[grid == 127] = free_bgr
    return img


def render_tricolor(grid: np.ndarray, colors: Optional[dict] = None) -> np.ndarray:
    """三值: occupied = 黑, free = 白, unknown = 灰。"""
    colors = colors or {}
    H, W = grid.shape
    img = np.zeros((H, W, 3), dtype=np.uint8)
    img[:] = tuple(colors.get("unknown", [160, 160, 160]))
    img[grid == 127] = tuple(colors.get("free", [255, 255, 255]))
    img[grid == 255] = tuple(colors.get("occupied", [0, 0, 0]))
    return img


def render_paper_style(
    grid: np.ndarray,
    trajectory_ij: Optional[np.ndarray] = None,
    people_ij: Optional[np.ndarray] = None,
    colors: Optional[dict] = None,
) -> np.ndarray:
    """论文式全局俯视: 柔和底图 + 相机轨迹 + 行人点。"""
    import cv2
    colors = colors or {}
    H, W = grid.shape
    bg = tuple(colors.get("background", [245, 245, 245]))
    occ_c = tuple(colors.get("occupied", [20, 20, 20]))
    free_c = tuple(colors.get("free", [255, 255, 255]))
    unk_c = tuple(colors.get("unknown", [220, 220, 220]))
    traj_c = tuple(colors.get("camera_traj", [255, 120, 0]))     # BGR order
    ppl_c = tuple(colors.get("people", [0, 120, 255]))

    img = np.zeros((H, W, 3), dtype=np.uint8)
    img[:] = bg
    img[grid == 0] = unk_c
    img[grid == 127] = free_c
    img[grid == 255] = occ_c

    if trajectory_ij is not None and trajectory_ij.shape[0] >= 2:
        pts = trajectory_ij.astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(img, [pts], False, traj_c, thickness=3, lineType=cv2.LINE_AA)
        # 起点终点圆
        cv2.circle(img, tuple(int(v) for v in trajectory_ij[0]), 8, traj_c, -1, cv2.LINE_AA)
        cv2.circle(img, tuple(int(v) for v in trajectory_ij[-1]), 8, traj_c, 2, cv2.LINE_AA)

    if people_ij is not None and people_ij.shape[0]:
        for p in people_ij:
            cv2.circle(img, tuple(int(v) for v in p), 5, ppl_c, -1, cv2.LINE_AA)

    return img


def save_static_map_v2(
    grid: np.ndarray,
    meta: dict,
    out_dir: str,
    debug: Optional[dict] = None,
    render_cfg: Optional[dict] = None,
    trajectory_ij: Optional[np.ndarray] = None,
) -> Dict[str, str]:
    """把 grid + 多种渲染 + debug 图一次性写盘。返回 file paths dict。"""
    import cv2, json
    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
    render_cfg = render_cfg or {}
    paths: Dict[str, str] = {}

    np.save(str(out / "static_map.npy"), grid)
    with open(out / "static_map_meta.json", "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    paths["npy"] = str(out / "static_map.npy")
    paths["meta"] = str(out / "static_map_meta.json")

    # 三种 render
    nav = render_nav_binary(grid, render_cfg.get("nav_binary"))
    cv2.imwrite(str(out / "nav_binary_map.png"), nav)
    paths["nav_binary"] = str(out / "nav_binary_map.png")

    tri = render_tricolor(grid, render_cfg.get("tricolor"))
    cv2.imwrite(str(out / "static_map_tricolor.png"), tri)
    paths["tricolor"] = str(out / "static_map_tricolor.png")

    paper = render_paper_style(
        grid,
        trajectory_ij=trajectory_ij,
        colors=render_cfg.get("paper_style"),
    )
    cv2.imwrite(str(out / "paper_style_global_view.png"), paper)
    paths["paper"] = str(out / "paper_style_global_view.png")

    # 保留旧的 tricolor 通道 (0/127/255 renderer, 用来给 pipeline_A 当 static_layer)
    cv2.imwrite(str(out / "static_map.png"), render_static_map(grid, meta))

    # debug
    if debug:
        for name, arr in debug.items():
            if name == "trajectory_ij" or arr is None:
                continue
            if arr.dtype in (np.float32, np.float64):
                # normalize to uint8 for viewing
                arr_min = float(np.min(arr))
                arr_max = float(np.max(arr))
                if arr_max > arr_min:
                    a8 = ((arr - arr_min) / (arr_max - arr_min) * 255).astype(np.uint8)
                else:
                    a8 = np.zeros_like(arr, dtype=np.uint8)
                cv2.imwrite(str(out / f"debug_{name}.png"), a8)
            else:
                a8 = arr.astype(np.uint8)
                # for count uint8-view: scale
                if arr.dtype == np.int32:
                    a8 = np.clip(arr, 0, 255).astype(np.uint8)
                cv2.imwrite(str(out / f"debug_{name}.png"), a8)
    return paths
