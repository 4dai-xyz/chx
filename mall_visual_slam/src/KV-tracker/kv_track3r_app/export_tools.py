"""把官方落盘的 traj.npy / pcd.npy / kf_poses.npy 转成 TUM / JSON / PLY / CSV。

不修改官方代码；只在 src/KV-tracker/ 调用官方输出做转换。
"""

from __future__ import annotations

import csv
import json
from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import Any, Iterable, List, Optional, Sequence

import numpy as np


# ---------------------------------------------------------------------------
# 矩阵 / 四元数工具
# ---------------------------------------------------------------------------


def _rot_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    """旋转矩阵 -> (qx, qy, qz, qw) (Hamilton 约定)"""
    R = np.asarray(R, dtype=np.float64)
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
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
    return np.array([x, y, z, w], dtype=np.float64)


def pose_to_tum_line(timestamp: float, T_wc: np.ndarray) -> str:
    """返回 TUM 单行: ``timestamp tx ty tz qx qy qz qw``."""
    T_wc = np.asarray(T_wc, dtype=np.float64)
    t = T_wc[:3, 3]
    q = _rot_to_quat_xyzw(T_wc[:3, :3])
    return (
        f"{timestamp:.6f} "
        f"{t[0]:.6f} {t[1]:.6f} {t[2]:.6f} "
        f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}"
    )


# ---------------------------------------------------------------------------
# 序列化工具
# ---------------------------------------------------------------------------


def _to_jsonable(obj: Any) -> Any:
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, np.floating):
        return float(obj)
    if isinstance(obj, np.integer):
        return int(obj)
    if is_dataclass(obj):
        return _to_jsonable(asdict(obj))
    if isinstance(obj, dict):
        return {str(k): _to_jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_to_jsonable(v) for v in obj]
    return obj


def write_json(path: str | Path, payload: Any) -> None:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "w", encoding="utf-8") as f:
        json.dump(_to_jsonable(payload), f, ensure_ascii=False, indent=2)


# ---------------------------------------------------------------------------
# 轨迹保存
# ---------------------------------------------------------------------------


def save_trajectory_tum(
    path: str | Path,
    records: Sequence[dict],
) -> None:
    """``records[i]`` 至少要包含 ``timestamp`` 和 ``T_wc`` (4x4)."""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "w", encoding="utf-8") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for r in records:
            f.write(pose_to_tum_line(float(r["timestamp"]), np.asarray(r["T_wc"])) + "\n")


def save_trajectory_json(path: str | Path, records: Sequence[dict]) -> None:
    poses = []
    for r in records:
        T_wc = np.asarray(r["T_wc"], dtype=np.float64)
        t = T_wc[:3, 3]
        q = _rot_to_quat_xyzw(T_wc[:3, :3])
        entry = {
            "frame_index": int(r.get("frame_index", -1)),
            "timestamp": float(r.get("timestamp", 0.0)),
            "T_wc": T_wc.tolist(),
            "translation": t.tolist(),
            "quaternion_xyzw": q.tolist(),
            "mean_confidence": float(r.get("mean_confidence", 0.0)),
            "fps": float(r.get("fps", 0.0)),
            "mode": str(r.get("mode", "tracking")),
        }
        poses.append(entry)
    payload = {
        "coordinate_convention": "T_wc camera-to-world; t = camera center in world frame; quaternion xyzw",
        "poses": poses,
    }
    write_json(path, payload)


def save_keyframes_json(path: str | Path, keyframe_records: Sequence[dict]) -> None:
    out = []
    for r in keyframe_records:
        T_wc = np.asarray(r["T_wc"], dtype=np.float64)
        t = T_wc[:3, 3]
        q = _rot_to_quat_xyzw(T_wc[:3, :3])
        entry = {
            "kf_index": int(r.get("kf_index", -1)),
            "frame_index": int(r.get("frame_index", -1)),
            "timestamp": float(r.get("timestamp", 0.0)),
            "T_wc": T_wc.tolist(),
            "translation": t.tolist(),
            "quaternion_xyzw": q.tolist(),
            "mean_confidence": float(r.get("mean_confidence", 0.0)),
        }
        out.append(entry)
    write_json(
        path,
        {
            "description": "KV-Tracker mapping keyframes; reconstructed via π³ + KV cache.",
            "keyframes": out,
        },
    )


def save_confidence_json(path: str | Path, confidence_records: Sequence[dict]) -> None:
    write_json(
        path,
        {
            "description": "KV-Tracker confidence statistics from Pi3 confidence head.",
            "frames": list(confidence_records),
        },
    )


def save_runtime_csv(path: str | Path, runtime_records: Sequence[dict]) -> None:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    if not runtime_records:
        # 空文件也建一个，便于下游知道流水线跑过
        with open(p, "w", encoding="utf-8") as f:
            f.write("frame_index,timestamp,fps,pi3_ms,total_ms,mode\n")
        return
    keys = list(runtime_records[0].keys())
    with open(p, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in runtime_records:
            w.writerow(r)


# ---------------------------------------------------------------------------
# 点云 PLY
# ---------------------------------------------------------------------------


def save_pointcloud_ply(
    path: str | Path,
    xyz: np.ndarray,
    rgb: Optional[np.ndarray] = None,
    confidence: Optional[np.ndarray] = None,
    conf_threshold: Optional[float] = None,
) -> int:
    """保存 PLY 点云. 返回写出点数."""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    n = xyz.shape[0]
    valid = np.ones(n, dtype=bool)
    if confidence is not None and conf_threshold is not None:
        c = np.asarray(confidence).reshape(-1)
        valid &= c >= float(conf_threshold)
    xyz = xyz[valid]
    if rgb is not None:
        rgb = np.asarray(rgb).reshape(-1, 3)
        rgb = rgb[valid]
        if rgb.dtype != np.uint8:
            rgb = np.clip(rgb, 0, 255).astype(np.uint8)
    m = xyz.shape[0]
    has_rgb = rgb is not None and rgb.shape[0] == m
    with open(p, "w", encoding="utf-8") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {m}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        if has_rgb:
            f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        if has_rgb:
            for (x, y, z), (r, g, b) in zip(xyz, rgb):
                f.write(f"{x:.5f} {y:.5f} {z:.5f} {int(r)} {int(g)} {int(b)}\n")
        else:
            for x, y, z in xyz:
                f.write(f"{x:.5f} {y:.5f} {z:.5f}\n")
    return m
