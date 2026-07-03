"""DPVO 相机轨迹 I/O。

DPVO 的 ``demo.py --save_trajectory`` 会写出 TUM 8 列:
    timestamp tx ty tz qx qy qz qw
其中 timestamp 是 DPVO 内部的 tick (0, 1, 2, ...)，对应原始视频里
第 ``tick * stride`` 帧附近的位姿。

第一版按 timestamp 做最近邻匹配。``timestamp`` 既可以是秒、也可以是
"DPVO tick"，配置 ``timestamp_unit`` 决定如何换算到视频帧的 timestamp。
"""

from __future__ import annotations

from typing import List, Optional

import numpy as np

from .types import CameraPose


def _quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def load_tum_trajectory(
    path: str,
    pose_is_twc: bool = True,
    scale: float = 1.0,
    timestamp_unit: str = "dpvo_tick",
    dpvo_stride: int = 2,
    video_fps: float = 30.0,
) -> List[CameraPose]:
    """读取 TUM 轨迹文件，返回 ``CameraPose`` 列表 (按时间戳排序)。

    参数:
        pose_is_twc: True 表示文件存的是 T_wc (相机到世界)。
            False 表示存的是 T_cw (世界到相机)，会自动求逆。
        scale: 平移部分整体乘的尺度因子 (单目 DPVO 没有真实米制尺度)。
        timestamp_unit:
            * ``"seconds"`` 直接当成秒；
            * ``"dpvo_tick"`` 文件第一列是 DPVO 内部 tick；
              换算到原始视频时间戳 ``t_sec = tick * dpvo_stride / video_fps``。
        dpvo_stride: DPVO 运行时的 ``--stride`` 参数。
        video_fps: 原始视频帧率，用于把 DPVO tick 转成秒。
    """
    poses: List[CameraPose] = []
    with open(path, "r", encoding="utf-8") as f:
        for line_no, raw in enumerate(f):
            s = raw.strip()
            if not s or s.startswith("#"):
                continue
            parts = s.split()
            if len(parts) < 8:
                continue
            try:
                vals = [float(x) for x in parts[:8]]
            except ValueError:
                continue
            t_raw, tx, ty, tz, qx, qy, qz, qw = vals
            if timestamp_unit == "dpvo_tick":
                timestamp = t_raw * dpvo_stride / max(video_fps, 1e-6)
                frame_index = int(round(t_raw * dpvo_stride))
            else:
                timestamp = t_raw
                frame_index = int(round(t_raw * video_fps))

            R = _quat_to_rot(qx, qy, qz, qw)
            t = np.array([tx, ty, tz], dtype=np.float64) * scale
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = t

            if not pose_is_twc:
                # T_cw -> T_wc
                Rwc = R.T
                twc = -Rwc @ t
                T = np.eye(4)
                T[:3, :3] = Rwc
                T[:3, 3] = twc

            poses.append(CameraPose(timestamp=timestamp, frame_index=frame_index, T_wc=T))

    poses.sort(key=lambda p: p.timestamp)
    return poses


def nearest_pose(
    poses: List[CameraPose], timestamp: float, tolerance: float
) -> Optional[CameraPose]:
    """二分查找最接近 ``timestamp`` 的位姿，超出 ``tolerance`` 返回 None。"""
    if not poses:
        return None
    ts = np.array([p.timestamp for p in poses])
    idx = int(np.searchsorted(ts, timestamp))
    candidates = []
    if idx > 0:
        candidates.append(idx - 1)
    if idx < len(ts):
        candidates.append(idx)
    best = None
    best_dt = float("inf")
    for c in candidates:
        dt = abs(ts[c] - timestamp)
        if dt < best_dt:
            best_dt = dt
            best = c
    if best is None or best_dt > tolerance:
        return None
    return poses[best]


def make_mock_trajectory(frame_count: int, fps: float) -> List[CameraPose]:
    """调试用的假轨迹: 沿 +z 缓慢前进的相机。"""
    poses: List[CameraPose] = []
    for i in range(frame_count):
        T = np.eye(4)
        # 让相机沿 +z 走一点，方便看 BEV 显示。
        T[:3, 3] = np.array([0.0, 0.0, i * 0.01])
        poses.append(
            CameraPose(timestamp=i / max(fps, 1e-6), frame_index=i, T_wc=T)
        )
    return poses
