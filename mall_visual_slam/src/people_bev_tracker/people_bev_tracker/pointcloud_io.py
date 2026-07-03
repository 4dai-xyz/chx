"""点云 I/O + 稳健过滤 (支持 PLY 二进制/ASCII + .npy)。"""

from __future__ import annotations

from pathlib import Path
from typing import Optional, Tuple

import numpy as np


def load_points(path: str, max_points: int = 0) -> np.ndarray:
    """读点云返回 (N, 3) float32。

    支持:
    * ``.ply`` 二进制 / ASCII: 用 Open3D 读, 只取 xyz
    * ``.npy``:
        - 形状 (N, 3): 直接取
        - 形状 (K, H, W, 3): 展平所有点 (KV-Tracker pcd.npy 就是这种)
        - object dtype (list of (H, W, 3)): 逐个展平

    ``max_points > 0`` 时随机降采样到该数量。
    """
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"pointcloud not found: {path}")
    suf = p.suffix.lower()
    if suf == ".ply":
        import open3d as o3d
        pcd = o3d.io.read_point_cloud(str(p))
        pts = np.asarray(pcd.points, dtype=np.float32)
    elif suf == ".npy":
        arr = np.load(str(p), allow_pickle=True)
        if arr.dtype == object:
            chunks = [np.asarray(x, dtype=np.float32).reshape(-1, 3) for x in arr]
            pts = np.concatenate(chunks, axis=0) if chunks else np.zeros((0, 3), np.float32)
        else:
            a = np.asarray(arr, dtype=np.float32)
            if a.ndim >= 2 and a.shape[-1] == 3:
                pts = a.reshape(-1, 3)
            else:
                raise ValueError(f"unsupported .npy shape: {a.shape}")
    else:
        raise ValueError(f"unsupported pointcloud extension: {suf}")

    pts = pts[np.isfinite(pts).all(axis=1)]

    if max_points and pts.shape[0] > max_points:
        rng = np.random.default_rng(0)
        idx = rng.choice(pts.shape[0], size=max_points, replace=False)
        pts = pts[idx]

    return pts


def robust_filter_points(
    points: np.ndarray,
    percentile: Tuple[float, float] = (1.0, 99.0),
) -> np.ndarray:
    """按每维 percentile 剔除极端离群点。"""
    if points.size == 0:
        return points
    pts = np.asarray(points, dtype=np.float32)
    lo, hi = percentile
    q = np.percentile(pts, [lo, hi], axis=0)  # (2, 3)
    mask = np.all((pts >= q[0]) & (pts <= q[1]), axis=1)
    return pts[mask]


def trajectory_proximity_filter(
    points: np.ndarray,
    trajectory_xyz: np.ndarray,
    max_ratio: float = 10.0,
    min_radius: float = 1.0,
) -> np.ndarray:
    """只保留 "离轨迹不太远" 的点。

    半径 = max(min_radius, max_ratio * traj_extent),
    其中 traj_extent = max_side_length(bbox(trajectory))。

    这一步专治单目 SLAM 点云里"暴走"的离群 patch 点 (DPVO 常见)。
    """
    if points.size == 0 or trajectory_xyz.size == 0:
        return points
    pts = np.asarray(points, dtype=np.float32)
    tj = np.asarray(trajectory_xyz, dtype=np.float32)
    traj_min = tj.min(axis=0)
    traj_max = tj.max(axis=0)
    extent = float(np.max(traj_max - traj_min))
    radius = max(float(min_radius), float(max_ratio) * extent)
    center = tj.mean(axis=0)
    d = np.linalg.norm(pts - center, axis=1)
    return pts[d <= radius]


def save_points_npy(path: str, points: np.ndarray) -> None:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    np.save(str(p), np.asarray(points, dtype=np.float32))


def summarize_points(points: np.ndarray) -> dict:
    if points.size == 0:
        return {"N": 0}
    pts = np.asarray(points, dtype=np.float64)
    q = np.percentile(pts, [1, 25, 50, 75, 99], axis=0)
    return {
        "N": int(pts.shape[0]),
        "p1":  q[0].tolist(),
        "p25": q[1].tolist(),
        "p50": q[2].tolist(),
        "p75": q[3].tolist(),
        "p99": q[4].tolist(),
        "min": pts.min(axis=0).tolist(),
        "max": pts.max(axis=0).tolist(),
        "mean": pts.mean(axis=0).tolist(),
    }
