"""V2: 静态地图质量指标 + 综合评分。

给定 grid (0=unknown, 127=free, 255=occupied) + meta + 轨迹在栅格里的位置,
计算:

  * occupied_ratio / free_ratio / unknown_ratio                (全图)
  * active_bbox                                                (轨迹周围裁剪)
  * active_free_ratio / active_unknown_ratio / active_occupied_ratio (裁剪后)
  * trajectory_collision_ratio                                 (轨迹落到 occupied 的比例)
  * obstacle_component_count / obstacle_small_component_ratio  (障碍连通域)
  * largest_free_component_ratio                               (最大 free 连通域 / active area)
  * score                                                      (综合评分, 越大越好)
"""

from __future__ import annotations

from typing import Dict, Optional

import cv2
import numpy as np


def compute_active_bbox(
    trajectory_ij: np.ndarray,
    W: int,
    H: int,
    margin_px: int = 120,
) -> Dict[str, int]:
    """轨迹 BEV 位置的 bbox + 边距, 限制到画布内。"""
    if trajectory_ij is None or trajectory_ij.shape[0] == 0:
        return {"x0": 0, "y0": 0, "x1": W, "y1": H}
    x_min = max(0, int(trajectory_ij[:, 0].min()) - margin_px)
    y_min = max(0, int(trajectory_ij[:, 1].min()) - margin_px)
    x_max = min(W, int(trajectory_ij[:, 0].max()) + margin_px)
    y_max = min(H, int(trajectory_ij[:, 1].max()) + margin_px)
    if x_max <= x_min:
        x_max = min(W, x_min + 1)
    if y_max <= y_min:
        y_max = min(H, y_min + 1)
    return {"x0": x_min, "y0": y_min, "x1": x_max, "y1": y_max}


def evaluate_grid_quality(
    grid: np.ndarray,
    meta: dict,
    trajectory_ij: Optional[np.ndarray] = None,
    small_component_area_px: int = 20,
    active_margin_px: int = 120,
    score_weights: Optional[Dict[str, float]] = None,
) -> Dict:
    """返回质量报告 dict。"""
    H, W = grid.shape
    total = H * W
    occ = (grid == 255)
    fre = (grid == 127)
    unk = (grid == 0)

    active_bbox = compute_active_bbox(trajectory_ij, W, H, margin_px=active_margin_px)
    ax0, ay0, ax1, ay1 = active_bbox["x0"], active_bbox["y0"], active_bbox["x1"], active_bbox["y1"]
    active_grid = grid[ay0:ay1, ax0:ax1]
    a_total = max(1, active_grid.size)
    a_occ = int((active_grid == 255).sum())
    a_fre = int((active_grid == 127).sum())
    a_unk = int((active_grid == 0).sum())

    # 轨迹碰撞
    traj_collision = 0
    traj_n = 0
    if trajectory_ij is not None and trajectory_ij.shape[0]:
        ins = (
            (trajectory_ij[:, 0] >= 0) & (trajectory_ij[:, 0] < W)
            & (trajectory_ij[:, 1] >= 0) & (trajectory_ij[:, 1] < H)
        )
        tj = trajectory_ij[ins]
        traj_n = int(tj.shape[0])
        if traj_n:
            occ_hits = occ[tj[:, 1], tj[:, 0]]
            traj_collision = int(occ_hits.sum())
    collision_ratio = float(traj_collision / traj_n) if traj_n else 0.0

    # 障碍连通域
    n_cc, cc = cv2.connectedComponents(occ.astype(np.uint8))
    n_components = max(0, n_cc - 1)
    small_ratio = 0.0
    if n_components > 0:
        areas = np.array([int((cc == lab).sum()) for lab in range(1, n_cc)])
        small = int((areas < small_component_area_px).sum())
        small_ratio = float(small / n_components)

    # 最大 free 连通域 (裁剪到 active bbox 内)
    active_free = (active_grid == 127).astype(np.uint8)
    largest_free_ratio = 0.0
    if active_free.sum() > 0:
        n_free_cc, ccf = cv2.connectedComponents(active_free)
        max_area = 0
        for lab in range(1, n_free_cc):
            a = int((ccf == lab).sum())
            if a > max_area:
                max_area = a
        largest_free_ratio = float(max_area / a_total)

    # 综合评分
    w = {"w_free": 1.5, "w_unknown": -0.5, "w_collision": -3.0,
         "w_small_comp": -0.8, "w_largest_free": 1.0}
    if score_weights:
        w.update(score_weights)
    active_free_ratio = a_fre / a_total
    active_unknown_ratio = a_unk / a_total
    score = (
        w["w_free"] * active_free_ratio
        + w["w_unknown"] * active_unknown_ratio
        + w["w_collision"] * collision_ratio
        + w["w_small_comp"] * small_ratio
        + w["w_largest_free"] * largest_free_ratio
    )

    return {
        "occupied_ratio": float(occ.sum() / total),
        "free_ratio": float(fre.sum() / total),
        "unknown_ratio": float(unk.sum() / total),
        "active_bbox": active_bbox,
        "active_size_px": int(active_grid.size),
        "active_occupied_ratio": float(a_occ / a_total),
        "active_free_ratio": float(active_free_ratio),
        "active_unknown_ratio": float(active_unknown_ratio),
        "trajectory_pixel_count": int(traj_n),
        "trajectory_collision_pixel_count": int(traj_collision),
        "trajectory_collision_ratio": float(collision_ratio),
        "obstacle_component_count": int(n_components),
        "obstacle_small_component_ratio": float(small_ratio),
        "largest_free_component_ratio": float(largest_free_ratio),
        "score": float(score),
    }


def check_thresholds(quality: Dict, thresholds: Dict) -> Dict[str, bool]:
    """按 route_A_v2.yaml 里的 quality_thresholds 校验, 返回每项 pass/fail。"""
    return {
        "active_free_ratio_ok":
            quality["active_free_ratio"] >= thresholds.get("active_free_ratio_min", 0.15),
        "active_unknown_ratio_ok":
            quality["active_unknown_ratio"] <= thresholds.get("active_unknown_ratio_max", 0.60),
        "trajectory_collision_ratio_ok":
            quality["trajectory_collision_ratio"]
            <= thresholds.get("trajectory_collision_ratio_max", 0.01),
        "obstacle_small_component_ratio_ok":
            quality["obstacle_small_component_ratio"]
            <= thresholds.get("obstacle_small_component_ratio_max", 0.35),
        "largest_free_component_ratio_ok":
            quality["largest_free_component_ratio"]
            >= thresholds.get("largest_free_component_ratio_min", 0.50),
    }
