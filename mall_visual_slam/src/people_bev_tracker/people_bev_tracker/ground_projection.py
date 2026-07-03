"""脚底像素射线与地面相交。

支持两种参数化方式：

* **世界系地面 (world)**: ``n_w · X_w + d_w = 0``。
  适合 DPVO 世界轴恰好与重力对齐的情况。

* **相机系地面 (camera-frame)**: 假设相机相对地面是刚性的 (头戴/胸戴相机俯仰
  角不变)。在相机系里，地面方程为 ``g_c · X_c = h``，其中 ``g_c`` 是相机系下
  的重力方向单位向量，``h`` 是相机离地高度。
  这种方式不依赖 DPVO 世界轴是否对齐重力，对存在固定俯仰角的第一人称
  视频更稳定。第一版默认用这个。
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

from .camera_model import pixel_to_ray


def gravity_in_camera(pitch_deg: float) -> np.ndarray:
    """相机俯仰 (向下为正) 时，重力方向在相机系的单位向量。

    标准约定: 相机 +X 右, +Y 下 (图像 y 向下), +Z 前.
    俯仰 alpha 度 (相机机头向下) 后，世界重力 ``(0, 1, 0)`` 在相机系下变成
    ``(0, cos(alpha), sin(alpha))`` (前方多了一个分量, 因为低头时正前方
    出现地面)。
    """
    a = math.radians(float(pitch_deg))
    return np.array([0.0, math.cos(a), math.sin(a)], dtype=np.float64)


def intersect_footpoint_with_ground(
    foot_uv: np.ndarray,
    K: np.ndarray,
    T_wc: np.ndarray,
    ground_normal: np.ndarray,
    ground_d: float,
    max_distance: float = 100.0,
) -> Optional[np.ndarray]:
    """**世界系** 地面相交。

    地面: ``n · X_w + d = 0``。返回世界坐标 (3,) 或 None。
    """
    ray_c = pixel_to_ray(foot_uv, K)
    R_wc = T_wc[:3, :3]
    C_w = T_wc[:3, 3]
    ray_w = R_wc @ ray_c

    n = np.asarray(ground_normal, dtype=np.float64).reshape(3)
    n_norm = np.linalg.norm(n)
    if n_norm < 1e-12:
        return None
    n = n / n_norm
    denom = float(n @ ray_w)
    if abs(denom) < 1e-6:
        return None

    lam = -(float(n @ C_w) + ground_d) / denom
    if lam <= 0 or lam > max_distance:
        return None

    X_w = C_w + lam * ray_w
    return X_w


def intersect_footpoint_with_camera_ground(
    foot_uv: np.ndarray,
    K: np.ndarray,
    T_wc: np.ndarray,
    camera_height: float,
    camera_pitch_deg: float = 0.0,
    gravity_cam: Optional[np.ndarray] = None,
    max_distance: float = 100.0,
) -> Optional[np.ndarray]:
    """**相机系** 地面相交，再变到世界系。

    相机系下地面: ``g_c · X_c = h`` (``g_c`` 是相机系下重力单位向量,
    ``h`` 是相机离地高度，单位与 DPVO 平移一致)。

    返回世界坐标 (3,) 或 None。
    """
    if gravity_cam is None:
        g_c = gravity_in_camera(camera_pitch_deg)
    else:
        g_c = np.asarray(gravity_cam, dtype=np.float64).reshape(3)
        n = np.linalg.norm(g_c)
        if n < 1e-12:
            return None
        g_c = g_c / n

    ray_c = pixel_to_ray(foot_uv, K)
    denom = float(g_c @ ray_c)
    if denom <= 1e-6:
        return None
    lam = float(camera_height) / denom
    if lam <= 0 or lam > max_distance:
        return None

    X_c = lam * ray_c
    R_wc = T_wc[:3, :3]
    C_w = T_wc[:3, 3]
    return R_wc @ X_c + C_w


def select_bev_axes(world_xyz: np.ndarray, bev_axes: list[str]) -> np.ndarray:
    """从 world XYZ 中选两轴当 BEV 平面坐标 (例如 ``["x", "z"]``)。"""
    axis_idx = {"x": 0, "y": 1, "z": 2}
    a, b = bev_axes[0].lower(), bev_axes[1].lower()
    return np.array([world_xyz[axis_idx[a]], world_xyz[axis_idx[b]]], dtype=np.float64)
