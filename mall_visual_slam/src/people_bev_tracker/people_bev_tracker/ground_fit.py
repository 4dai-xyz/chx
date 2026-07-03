"""地面拟合: RANSAC / bottom-percentile PCA / camera-frame fallback。

统一约定: 地面方程 ``n^T X + d = 0``, ``n`` 单位向量, "地面之上" 侧
``n^T X + d > 0`` 表示相机所在侧 (法向指向相机, 便于用它算相机离地高度)。

若 axis_hint = "y" (相机初始 +Y ~= 向下), 那么"地面在相机下方", 相机侧
``n^T X + d > 0``: 相机的 Y < ground_y, 而 n ~= (0, 1, 0) → n^T C < 0 → 需要
让 n^T C + d 与相机在 hint 方向的坐标一致 (小于 0 或大于 0 都行, 只要 sign 稳定)。

代码里我们统一把 n 的方向调成: ``n · axis_hint_vec > 0`` (法向与 axis_hint 同向)。
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional

import numpy as np


AXIS_HINT_VEC = {
    "x": np.array([1.0, 0.0, 0.0]),
    "y": np.array([0.0, 1.0, 0.0]),
    "z": np.array([0.0, 0.0, 1.0]),
    "-x": np.array([-1.0, 0.0, 0.0]),
    "-y": np.array([0.0, -1.0, 0.0]),
    "-z": np.array([0.0, 0.0, -1.0]),
}


def _plane_normalize(plane: np.ndarray, axis_hint: np.ndarray) -> np.ndarray:
    """把 (a,b,c,d) 归一化, 并保证 n 与 axis_hint 同向。返回 (a,b,c,d)."""
    n = plane[:3]
    d = plane[3]
    norm = np.linalg.norm(n)
    if norm < 1e-12:
        return plane
    n = n / norm
    d = d / norm
    if float(np.dot(n, axis_hint)) < 0:
        n = -n
        d = -d
    return np.array([n[0], n[1], n[2], d], dtype=np.float64)


def _inlier_stats(points: np.ndarray, plane: np.ndarray, thresh: float) -> Dict:
    n = plane[:3]
    d = plane[3]
    dist = np.abs(points @ n + d)
    inl = dist < thresh
    inl_count = int(inl.sum())
    total = points.shape[0]
    return {
        "inlier_ratio": inl_count / max(total, 1),
        "rmse": float(np.sqrt((dist[inl] ** 2).mean())) if inl_count else float("inf"),
        "inlier_count": inl_count,
        "total": total,
    }


def _angle_deg_between(a: np.ndarray, b: np.ndarray) -> float:
    a = a / (np.linalg.norm(a) + 1e-12)
    b = b / (np.linalg.norm(b) + 1e-12)
    c = float(np.clip(a @ b, -1.0, 1.0))
    return math.degrees(math.acos(c))


def _adaptive_ransac_thresh(points: np.ndarray, base_thresh: float) -> float:
    """RANSAC 距离阈值随点云自身尺度自适应。

    ``base_thresh`` 只对小尺度 (e.g. metric) 点云成立。如果点云 IQR 明显大于
    ``base_thresh`` 的 100 倍, 就按 IQR 的一小部分放大它。
    """
    iqr = np.percentile(points, 75, axis=0) - np.percentile(points, 25, axis=0)
    scale = float(np.mean(iqr))
    if scale <= 0 or not np.isfinite(scale):
        return float(base_thresh)
    # 目标: threshold ≈ 3% 的中位 IQR, 但不小于 base_thresh
    adaptive = max(float(base_thresh), 0.03 * scale)
    return adaptive


def fit_ground_ransac(
    points: np.ndarray,
    cfg: dict,
) -> Optional[Dict]:
    """Open3D RANSAC. 失败返回 None。"""
    if points.shape[0] < 100:
        return None
    import open3d as o3d
    thresh = _adaptive_ransac_thresh(
        points, float(cfg.get("ransac_distance_threshold", 0.015))
    )
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    try:
        plane, _ = pcd.segment_plane(
            distance_threshold=thresh,
            ransac_n=3,
            num_iterations=int(cfg.get("ransac_iterations", 2000)),
        )
    except Exception as e:
        return {"error": str(e)}
    axis_hint = AXIS_HINT_VEC.get(str(cfg.get("axis_hint", "y")).lower(), AXIS_HINT_VEC["y"])
    p = _plane_normalize(np.asarray(plane, dtype=np.float64), axis_hint)
    stats = _inlier_stats(points, p, thresh)
    return {
        "method": "ransac",
        "normal": p[:3].tolist(),
        "d": float(p[3]),
        "axis_hint": str(cfg.get("axis_hint", "y")),
        "angle_vs_axis_hint_deg": _angle_deg_between(p[:3], axis_hint),
        "distance_threshold_used": thresh,
        **stats,
    }


def fit_ground_bottom_pca(
    points: np.ndarray,
    cfg: dict,
) -> Optional[Dict]:
    """按 axis_hint 方向, 保留"底部" percentile 的点做 PCA, 最小奇异值方向 = 法向。"""
    if points.shape[0] < 100:
        return None
    axis_hint = AXIS_HINT_VEC.get(str(cfg.get("axis_hint", "y")).lower(), AXIS_HINT_VEC["y"])
    proj = points @ axis_hint    # (N,)  正向大 = 更贴近 axis_hint 方向
    pct = float(cfg.get("bottom_percentile", 20.0))
    thresh = np.percentile(proj, 100.0 - pct)  # 保留 proj 较大的一端 (贴近 axis_hint)
    cand = points[proj >= thresh]
    if cand.shape[0] < 30:
        return None
    mean = cand.mean(axis=0)
    C = (cand - mean).T @ (cand - mean) / cand.shape[0]
    U, S, _ = np.linalg.svd(C)
    n = U[:, -1]                 # 最小奇异值对应的方向 = 平面法向
    p = np.array([n[0], n[1], n[2], -float(n @ mean)], dtype=np.float64)
    p = _plane_normalize(p, axis_hint)
    thresh = _adaptive_ransac_thresh(points, float(cfg.get("ransac_distance_threshold", 0.015)))
    stats = _inlier_stats(points, p, thresh)
    return {
        "method": "bottom_pca",
        "normal": p[:3].tolist(),
        "d": float(p[3]),
        "axis_hint": str(cfg.get("axis_hint", "y")),
        "angle_vs_axis_hint_deg": _angle_deg_between(p[:3], axis_hint),
        "distance_threshold_used": thresh,
        **stats,
    }


def make_camera_ground_fallback(cfg: dict) -> Dict:
    """相机系地面 fallback: 把 axis_hint 当地面法向 (先验), d 由高度 h 决定。

    在 axis_hint = "y" 时, 假设相机在 world 原点朝 +z 前进, 世界 +Y 向下.
    地面在相机下方 h_dpvo 处 (法向指向 +Y, +Y·X + d = 0 → X_y = -d.)
    我们让 d = -h, 使得 (n · C + d)|C=0 = -h < 0, 即相机上方到地面的距离为 -h ??
    换个角度: 我们只是提供一个稳定的地面, 具体符号不敏感, ``trajectory_flatten``
    只关心 median(h(t)) 做归零, 不关心绝对值。
    """
    axis_hint_str = str(cfg.get("axis_hint", "y")).lower()
    axis_hint = AXIS_HINT_VEC.get(axis_hint_str, AXIS_HINT_VEC["y"])
    h = float(cfg.get("fallback_camera_height", 0.1))
    p = np.array([axis_hint[0], axis_hint[1], axis_hint[2], -h], dtype=np.float64)
    return {
        "method": "camera_fallback",
        "normal": p[:3].tolist(),
        "d": float(p[3]),
        "axis_hint": axis_hint_str,
        "angle_vs_axis_hint_deg": 0.0,
        "inlier_ratio": 0.0,
        "rmse": float("nan"),
        "inlier_count": 0,
        "total": 0,
        "fallback_camera_height": h,
    }


def choose_best_ground(
    candidates: List[Dict],
    cfg: dict,
) -> Dict:
    """从 candidates 里挑一个。

    评分策略 (每个可选权重):
      * inlier_ratio 越大越好
      * angle_vs_axis_hint_deg 越小越好
      * rmse 越小越好

    综合分:
      score = inlier_ratio - alpha_angle * (angle / 90) - alpha_rmse * min(rmse, 1)

    默认: alpha_angle=0.5, alpha_rmse=0.5
    (angle 从 0 到 90 度线性惩罚, rmse 也线性惩罚)

    要求 (硬约束):
      * inlier_ratio >= min_inlier_ratio (默认 0.02, 之前 0.3 太严)
      * angle <= max_angle (默认 35)

    都不满足 → camera_fallback。
    """
    min_ratio = float(cfg.get("min_inlier_ratio", 0.02))
    max_angle = float(cfg.get("normal_max_angle_deg", 35.0))
    alpha_angle = float(cfg.get("score_alpha_angle", 0.5))
    alpha_rmse = float(cfg.get("score_alpha_rmse", 0.5))

    ok = []
    for c in candidates:
        if not c or c.get("method") == "camera_fallback":
            continue
        ratio = float(c.get("inlier_ratio", 0.0))
        angle = float(c.get("angle_vs_axis_hint_deg", 999.0))
        rmse = float(c.get("rmse", float("inf")))
        if ratio < min_ratio or angle > max_angle:
            continue
        score = ratio - alpha_angle * (angle / 90.0) - alpha_rmse * min(rmse, 1.0)
        c["score"] = score
        ok.append(c)

    if ok:
        best = max(ok, key=lambda c: c["score"])
        best["selected_reason"] = (
            f"best score {best['score']:.3f} "
            f"(inlier {best['inlier_ratio']:.3f}, "
            f"angle {best['angle_vs_axis_hint_deg']:.1f}°, "
            f"rmse {best.get('rmse', 0):.4f})"
        )
        return best

    for c in candidates:
        if c and c.get("method") == "camera_fallback":
            c["selected_reason"] = "no valid RANSAC/PCA candidate; using camera fallback"
            return c
    return {**make_camera_ground_fallback(cfg), "selected_reason": "hard fallback"}


def fit_ground_all_methods(points: np.ndarray, cfg: dict) -> Dict:
    """跑 RANSAC + bottom_pca + camera_fallback, 返回 {candidates, best}."""
    method = str(cfg.get("method", "auto")).lower()
    candidates: List[Dict] = []

    if method in ("auto", "ransac"):
        c = fit_ground_ransac(points, cfg)
        if c and "error" not in c:
            candidates.append(c)
    if method in ("auto", "bottom_pca"):
        c = fit_ground_bottom_pca(points, cfg)
        if c:
            candidates.append(c)
    if method in ("auto", "camera_fallback", "camera"):
        candidates.append(make_camera_ground_fallback(cfg))

    if method == "auto":
        best = choose_best_ground(candidates, cfg)
    else:
        best = candidates[0] if candidates else make_camera_ground_fallback(cfg)
        best["selected_reason"] = f"user forced method={method}"

    return {"candidates": candidates, "best": best}
