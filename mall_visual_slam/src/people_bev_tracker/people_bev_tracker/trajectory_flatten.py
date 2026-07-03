"""DPVO 轨迹平面化: 去除头戴相机走路的高度抖动。

已知地面 ``n^T X + d = 0``, 相机中心 ``C(t)``, 高度:

    h(t) = n^T C(t) + d

平面化 (`constant` 模式):
    C_flat(t) = C(t) + (h_ref - h(t)) * n,   h_ref = median(h(t))

平面化 (`lpf` 模式, 保留长期地形变化):
    h_smooth(t) = LPF(h(t))
    C_flat(t) = C(t) + (h_smooth(t) - h(t)) * n

姿态 R_wc 不改, 只改平移。TUM 格式保留原本的 quaternion。
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List

import numpy as np


def _load_tum(path: str) -> np.ndarray:
    """加载 TUM 8 列: (N, 8)  timestamp tx ty tz qx qy qz qw."""
    return np.loadtxt(path, comments="#")


def _save_tum(path: str, arr: np.ndarray, header: str = "") -> None:
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "w", encoding="utf-8") as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        if header:
            f.write(f"# {header}\n")
        for row in arr:
            f.write(
                f"{row[0]:.6f} "
                f"{row[1]:.6f} {row[2]:.6f} {row[3]:.6f} "
                f"{row[4]:.6f} {row[5]:.6f} {row[6]:.6f} {row[7]:.6f}\n"
            )


def _lpf_1d(x: np.ndarray, dt: float, cutoff_hz: float) -> np.ndarray:
    """单极点低通 (RC 一阶)。"""
    if cutoff_hz <= 0:
        return x.copy()
    tau = 1.0 / (2 * np.pi * cutoff_hz)
    alpha = dt / (dt + tau)
    y = np.empty_like(x)
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = alpha * x[i] + (1 - alpha) * y[i - 1]
    # 反向再滤一次消相位延迟
    z = np.empty_like(x)
    z[-1] = y[-1]
    for i in range(len(x) - 2, -1, -1):
        z[i] = alpha * y[i] + (1 - alpha) * z[i + 1]
    return z


def flatten_trajectory(
    input_tum: str,
    output_tum: str,
    ground_plane: Dict,
    mode: str = "constant",
    lpf_cutoff_hz: float = 0.3,
) -> Dict:
    """输入 TUM, 按 ground_plane 平面化, 输出 TUM。返回统计信息。"""
    arr = _load_tum(input_tum)
    if arr.ndim == 1:
        arr = arr[None, :]
    n = np.asarray(ground_plane["normal"], dtype=np.float64)
    d = float(ground_plane["d"])
    n = n / (np.linalg.norm(n) + 1e-12)

    C = arr[:, 1:4].astype(np.float64)          # (N, 3)
    h = C @ n + d                                # (N,)
    if mode == "lpf":
        # 用相邻 timestamp 差估计 dt (rough)
        ts = arr[:, 0]
        # DPVO tick 单位 -> 转成秒
        # 这里默认输入是 tick, 若 pose_io.timestamp_unit=dpvo_tick 已在别处处理
        # trajectory_flat.txt 我们保持和输入一致的 timestamp 单位
        if len(ts) > 1:
            dt = float(np.median(np.diff(ts)))
        else:
            dt = 1.0
        h_smooth = _lpf_1d(h, dt=dt, cutoff_hz=lpf_cutoff_hz)
        delta = h_smooth - h                     # (N,)
    else:  # constant
        h_ref = float(np.median(h))
        delta = h_ref - h
        h_smooth = np.full_like(h, h_ref)

    C_flat = C + delta[:, None] * n[None, :]
    h_after = C_flat @ n + d

    out = arr.copy()
    out[:, 1:4] = C_flat
    _save_tum(output_tum, out,
              header=f"flatten mode={mode} n={n.tolist()} d={d:.6f}")

    return {
        "input_tum": str(input_tum),
        "output_tum": str(output_tum),
        "N": int(arr.shape[0]),
        "mode": mode,
        "normal": n.tolist(),
        "d": float(d),
        "h_before": {
            "mean": float(h.mean()),
            "std": float(h.std()),
            "min": float(h.min()),
            "max": float(h.max()),
            "median": float(np.median(h)),
        },
        "h_after": {
            "mean": float(h_after.mean()),
            "std": float(h_after.std()),
            "min": float(h_after.min()),
            "max": float(h_after.max()),
            "median": float(np.median(h_after)),
        },
    }
