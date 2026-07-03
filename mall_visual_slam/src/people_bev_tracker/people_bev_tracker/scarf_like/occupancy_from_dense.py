"""V3.1 阶段 7: 稠密点云 → 2D occupancy grid。

输入稠密静态点云 (DPVO 单位, 世界系) + DPVO 轨迹 + 地面平面 + mirror_y。

三层:
  free:     floor points 投影 + trajectory corridor (相机走过永远 free)
  occupied: obstacle_height_range 内的多帧一致点 (submap fusion 已保证 >= min_obs)
  unknown:  其余 (二值导航图里当障碍)

统一坐标层:
  X_w (DPVO world)
    → R_align (地面法向 → +Y)
    → select (x, z)
    → apply_bev_alignment_xy(mirror_y)   ← 与 V3 前置校准一致
    → 栅格化
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from ..bev_alignment import apply_bev_alignment_xy
from ..static_map import _rot_matrix_align_a_to_b


def _world_to_grid(xy: np.ndarray, W, H, res, ox, oz) -> np.ndarray:
    px = (W / 2 + (xy[:, 0] - ox) / res).astype(np.int64)
    py = (H / 2 - (xy[:, 1] - oz) / res).astype(np.int64)
    return np.stack([px, py], axis=1)


def _draw_frustum_free(
    mask: np.ndarray,
    cam_bev: np.ndarray,           # (M, 2) aligned+mirrored BEV 相机位置
    fwd_bev: np.ndarray,           # (M, 2) aligned+mirrored BEV 前向 (未归一)
    W: int, H: int, res: float, ox: float, oz: float,
    range_unit: float, half_fov_deg: float,
) -> int:
    """沿相机视锥画扇形 free (ray carving 近似)。返回画了几个扇形。"""
    import math
    range_px = range_unit / res
    half = math.radians(half_fov_deg)
    n = 0
    for c, f in zip(cam_bev, fwd_bev):
        cu = W / 2 + (c[0] - ox) / res
        cv_ = H / 2 - (c[1] - oz) / res
        fn = math.hypot(f[0], f[1])
        if fn < 1e-9:
            continue
        fx, fy = f[0] / fn, f[1] / fn
        def rot(dx, dy, th):
            cc, ss = math.cos(th), math.sin(th)
            return dx * cc - dy * ss, dx * ss + dy * cc
        dlx, dly = rot(fx, fy, half)
        drx, dry = rot(fx, fy, -half)
        poly = np.array([
            [cu, cv_],
            [cu + dlx * range_px, cv_ - dly * range_px],
            [cu + drx * range_px, cv_ - dry * range_px],
        ], dtype=np.int32)
        cv2.fillConvexPoly(mask, poly, 255)
        n += 1
    return n


def build_occupancy_from_dense(
    dense_pts_world: np.ndarray,
    trajectory_world_xyz: np.ndarray,
    ground_plane: Dict,
    cfg: dict,
    transform: str = "mirror_y",
    camera_poses_T_wc: Optional[np.ndarray] = None,
    dense_obs: Optional[np.ndarray] = None,
) -> Tuple[np.ndarray, dict, dict]:
    """返回 (grid uint8 0/127/255, meta, debug)。

    ``camera_poses_T_wc``: (M, 4, 4) 用于视锥 ray carving 生成 free (可选)。
    ``dense_obs``: (N,) 每个稠密点被几个关键帧观测。障碍层只保留
        obs >= obstacle_min_observations 的点 (多帧一致的稳定墙, 剔除单帧噪声/反光)。
    """
    res = float(cfg["resolution_unit_per_px"])
    W = int(cfg["width_px"]); H = int(cfg["height_px"])
    bev_axes = ("x", "z")
    hmin, hmax = cfg["obstacle_height_range_unit"]
    hmin = float(hmin); hmax = float(hmax)
    floor_abs = float(cfg["floor_height_abs_thresh_unit"])
    corridor_r_unit = float(cfg.get("free_corridor_radius_unit", 0.25))
    obst_close = int(cfg.get("obstacle_close_kernel", 9))
    obst_dilate = int(cfg.get("obstacle_dilate_kernel", 3))
    obst_min_area = int(cfg.get("obstacle_min_component_area_px", 25))
    free_close = int(cfg.get("free_close_kernel", 17))
    free_dilate = int(cfg.get("free_dilate_kernel", 3))

    # ---- 地面对齐 R_align ----
    n = np.asarray(ground_plane["normal"], dtype=np.float64).reshape(3)
    d = float(ground_plane["d"])
    n = n / (np.linalg.norm(n) + 1e-12)
    R_align = _rot_matrix_align_a_to_b(n, np.array([0.0, 1.0, 0.0]))

    align_cfg = {"enabled": True, "transform": transform}

    def to_bev(pw: np.ndarray) -> np.ndarray:
        """world (N,3) → aligned+mirrored BEV (N,2) 以及 height (N,)。"""
        pa = (R_align @ pw.T).T                 # (N,3)
        h = pa[:, 1] + d
        xz = np.stack([pa[:, 0], pa[:, 2]], axis=1)
        xz = apply_bev_alignment_xy(xz, align_cfg)
        return xz, h

    # ---- 轨迹 (定 origin) ----
    traj_xz, _ = to_bev(trajectory_world_xyz)
    ox = float((traj_xz[:, 0].min() + traj_xz[:, 0].max()) / 2)
    oz = float((traj_xz[:, 1].min() + traj_xz[:, 1].max()) / 2)

    # 相机 h 符号: 若相机在 -h 侧, 翻转平面 (与 build_route_A 一致)
    pa_traj = (R_align @ trajectory_world_xyz.T).T
    Ch = pa_traj[:, 1] + d
    if float(np.median(Ch)) < 0:
        # 翻转: 让 h>0 表示 "地面之上到相机"
        d = -d
        n = -n
        # 重新算 (R_align 不变, 因为 mirror in +Y ambiguous). 简化: 重算 h
        # 注意 R_align 是把 n->+Y; 翻转 n 后应重算 R_align
        R_align = _rot_matrix_align_a_to_b(n, np.array([0.0, 1.0, 0.0]))
        traj_xz, _ = to_bev(trajectory_world_xyz)
        ox = float((traj_xz[:, 0].min() + traj_xz[:, 0].max()) / 2)
        oz = float((traj_xz[:, 1].min() + traj_xz[:, 1].max()) / 2)

    # ---- 稠密点分层 ----
    pts_xz, pts_h = to_bev(dense_pts_world)

    obst_min_obs = int(cfg.get("obstacle_min_observations", 2))
    keep_obs2_if_near_obs3 = bool(cfg.get("keep_obs2_if_near_obs3", False))
    keep_obs2_near_radius_unit = float(cfg.get("keep_obs2_near_radius_unit", 0.10))
    floor_mask_pts = np.abs(pts_h) <= floor_abs
    obst_band = (pts_h >= hmin) & (pts_h <= hmax)
    obst_mask_pts = obst_band.copy()
    # 障碍层多帧一致性
    if dense_obs is not None and obst_min_obs > 1:
        obs_arr = np.asarray(dense_obs).reshape(-1)
        if obs_arr.shape[0] == obst_band.shape[0]:
            base_stable = obst_band & (obs_arr >= obst_min_obs)
            if keep_obs2_if_near_obs3 and obst_min_obs >= 3:
                # 恢复 obs=2 中"紧邻已有 obs>=3 稳定墙"的点 (端点/边缘)
                candidate2 = obst_band & (obs_arr == 2)
                if base_stable.any() and candidate2.any():
                    stable_xz = pts_xz[base_stable]
                    cand_xz = pts_xz[candidate2]
                    # 用 grid bucket 做半径查询, 避免 O(N*M)
                    r = keep_obs2_near_radius_unit
                    # bucket by (round(x/r), round(z/r))
                    keys = set()
                    for p in stable_xz:
                        cx, cy = int(p[0] / r), int(p[1] / r)
                        for dx in (-1, 0, 1):
                            for dy in (-1, 0, 1):
                                keys.add((cx + dx, cy + dy))
                    near_flag = np.zeros(cand_xz.shape[0], dtype=bool)
                    for i, p in enumerate(cand_xz):
                        if (int(p[0] / r), int(p[1] / r)) in keys:
                            near_flag[i] = True
                    recovered = candidate2.copy()
                    recovered_idxs = np.where(candidate2)[0]
                    recovered[recovered_idxs[~near_flag]] = False
                    obst_mask_pts = base_stable | recovered
                else:
                    obst_mask_pts = base_stable
            else:
                obst_mask_pts = base_stable

    def rasterize(xz):
        ij = _world_to_grid(xz, W, H, res, ox, oz)
        inb = (ij[:, 0] >= 0) & (ij[:, 0] < W) & (ij[:, 1] >= 0) & (ij[:, 1] < H)
        return ij[inb]

    # occupied 计数
    occ = np.zeros((H, W), dtype=np.int32)
    oij = rasterize(pts_xz[obst_mask_pts])
    if oij.shape[0]:
        np.add.at(occ, (oij[:, 1], oij[:, 0]), 1)
    obstacle_raw = (occ >= 1).astype(np.uint8) * 255

    # floor → free 计数
    floor_free = np.zeros((H, W), dtype=np.uint8)
    fij = rasterize(pts_xz[floor_mask_pts])
    if fij.shape[0]:
        floor_free[fij[:, 1], fij[:, 0]] = 255

    # trajectory corridor → free
    corridor = np.zeros((H, W), dtype=np.uint8)
    tij = rasterize(traj_xz)
    if tij.shape[0] >= 2:
        thick = max(1, int(round(corridor_r_unit / res)))
        cv2.polylines(corridor, [tij.astype(np.int32).reshape(-1, 1, 2)], False, 255, thick)
        for p in [tij[0], tij[-1]]:
            cv2.circle(corridor, tuple(int(v) for v in p), thick // 2 + 1, 255, -1)

    # camera frustum ray carving → free (相机看得见的地面区域)
    frustum = np.zeros((H, W), dtype=np.uint8)
    n_frustum = 0
    if bool(cfg.get("free_from_ray_carving", True)) and camera_poses_T_wc is not None \
            and len(camera_poses_T_wc) > 0:
        poses = np.asarray(camera_poses_T_wc, dtype=np.float64)
        stride = max(1, poses.shape[0] // 400)   # 最多 ~400 个视锥
        cam_w = poses[::stride, :3, 3]
        fwd_w = (poses[::stride, :3, :3] @ np.array([0.0, 0.0, 1.0]))
        cam_xz, _ = to_bev(cam_w)
        # forward: 只旋转不平移 → 用 to_bev 差分
        fwd_xz, _ = to_bev(cam_w + fwd_w)
        fwd_vec = fwd_xz - cam_xz
        n_frustum = _draw_frustum_free(
            frustum, cam_xz, fwd_vec, W, H, res, ox, oz,
            range_unit=float(cfg.get("frustum_range_unit", 0.8)),
            half_fov_deg=float(cfg.get("frustum_half_fov_deg", 33.0)))

    # ---- 形态学正则化 ----
    # obstacle: close → 去小连通域 → dilate
    obst = obstacle_raw
    if obst_close > 1:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (obst_close, obst_close))
        obst = cv2.morphologyEx(obst, cv2.MORPH_CLOSE, k)
    if obst_min_area > 0:
        n_cc, cc = cv2.connectedComponents((obst > 0).astype(np.uint8))
        keep = np.zeros_like(obst)
        for lab in range(1, n_cc):
            if int((cc == lab).sum()) >= obst_min_area:
                keep[cc == lab] = 255
        obst = keep
    if obst_dilate > 1:
        k = np.ones((obst_dilate, obst_dilate), np.uint8)
        obst = cv2.dilate(obst, k)

    # free: (floor ∪ corridor ∪ frustum) close + dilate
    free = ((floor_free > 0) | (corridor > 0) | (frustum > 0)).astype(np.uint8) * 255
    if free_close > 1:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (free_close, free_close))
        free = cv2.morphologyEx(free, cv2.MORPH_CLOSE, k)
    if free_dilate > 1:
        k = np.ones((free_dilate, free_dilate), np.uint8)
        free = cv2.dilate(free, k)

    obst_bool = obst > 0
    corridor_bool = corridor > 0
    # trajectory 永远 free: occupied 让开 corridor
    obst_bool = obst_bool & (~corridor_bool)
    free_bool = (free > 0) & (~obst_bool)

    grid = np.zeros((H, W), dtype=np.uint8)
    grid[free_bool] = 127
    grid[obst_bool] = 255

    total = H * W
    stats = {
        "occupied_ratio": float(obst_bool.sum() / total),
        "free_ratio": float(free_bool.sum() / total),
        "unknown_ratio": float((grid == 0).sum() / total),
        "n_dense_points": int(dense_pts_world.shape[0]),
        "n_floor_points": int(floor_mask_pts.sum()),
        "n_obstacle_points": int(obst_mask_pts.sum()),
        "n_frustum_carved": int(n_frustum),
    }
    meta = {
        "resolution_unit_per_px": res,
        "width_px": W, "height_px": H,
        "origin_world": [ox, oz],
        "bev_axes": list(bev_axes),
        "obstacle_height_range": [hmin, hmax],
        "floor_height_abs_thresh_unit": floor_abs,
        "ground_plane": {"normal": n.tolist(), "d": d},
        "R_align": R_align.tolist(),
        "bev_alignment": {
            "enabled": True,
            "transform": transform,
            "source": "output/route_A_v3_scarf/alignment_selected.json",
        },
        "colors": {"unknown": [35, 35, 35], "free": [205, 205, 205], "occupied": [70, 70, 70]},
        "source": "scarf_like_dense",
        "statistics": stats,
    }
    debug = {
        "obstacle_raw": obstacle_raw,
        "obstacle_regularized": obst,
        "floor_free": floor_free,
        "corridor": corridor,
        "frustum_free": frustum,
        "free_merged": free,
        "trajectory_ij": tij,
    }
    return grid, meta, debug
