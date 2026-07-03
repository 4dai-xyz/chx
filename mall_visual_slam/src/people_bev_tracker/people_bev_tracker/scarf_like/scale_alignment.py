"""V3.1 阶段 5: 单帧深度尺度对齐 (借鉴 ScaRF-SLAM frame scale optimization)。

Depth Anything V2 Metric 输出的是**米制深度**, 但 DPVO 世界系是**单目尺度
(DPVO 单位)**。两者尺度不同, 需要为每个关键帧求一个 scale s_i, 把米制深度
换算成 DPVO 单位, 使得地面像素反投影后正好落在地面平面上。

数学:
  相机系点 (米):  X_c = z_m * K^-1 [u, v, 1]  (z_m 为米制深度, DA 输出)
  世界系点 (DPVO 单位): X_w = R_wc (s_i X_c) + C_w
  地面约束: n_w · X_w + d = 0
    => s_i (n_w·R_wc)·X_c = -(n_w·C_w + d) = h_signed
    => s_i = h_signed / (g_c · X_c),   g_c = R_wc^T n_w

  对一帧所有 floor 像素求 s_i, 取中位数 (鲁棒)。

约束:
  * floor 像素太少 → 用邻近关键帧尺度;
  * s_i 突变限制 max_scale_jump_ratio;
  * s 序列滑动中值滤波。
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np


def _floor_candidate_pixels(
    kf_h: int, kf_w: int,
    person_mask: Optional[np.ndarray],
    bottom_frac: float = 0.25,
    center_frac: float = 0.6,
) -> np.ndarray:
    """图像下 bottom_frac 行 + 中央 center_frac 列 (排除行人) 当 floor candidate。

    只取下方中央区域, 避开天花板/墙/水平线附近像素 (那些 ray 近水平, 会污染尺度)。
    """
    m = np.zeros((kf_h, kf_w), dtype=bool)
    y0 = int(kf_h * (1.0 - bottom_frac))
    x0 = int(kf_w * (1.0 - center_frac) / 2)
    x1 = int(kf_w * (1.0 + center_frac) / 2)
    m[y0:, x0:x1] = True
    if person_mask is not None:
        pm = person_mask
        if pm.shape != (kf_h, kf_w):
            import cv2
            pm = cv2.resize(pm.astype(np.uint8), (kf_w, kf_h),
                            interpolation=cv2.INTER_NEAREST) > 0
        m &= ~(pm > 0)
    return m


def estimate_frame_scale(
    depth_m: np.ndarray,
    K: np.ndarray,
    T_wc: np.ndarray,
    ground_normal: np.ndarray,
    ground_d: float,
    person_mask: Optional[np.ndarray] = None,
    bottom_frac: float = 0.25,
    min_floor_pixels: int = 300,
    px_stride: int = 2,
    downward_min: float = 0.30,
) -> Tuple[Optional[float], int]:
    """返回 (scale s_i, 使用的 floor 像素数)。floor 不足返回 (None, n)。

    scale = DPVO 单位 / 米。用地面像素约束:
      s = h_signed / (g_c · X_c),  只保留朝下 ray (|g_c·ray_unit| > downward_min) 且 s>0。
    """
    kf_h, kf_w = depth_m.shape
    floor = _floor_candidate_pixels(kf_h, kf_w, person_mask, bottom_frac)
    ys, xs = np.where(floor)
    if ys.size == 0:
        return None, 0
    sel = np.arange(0, ys.size, px_stride)
    ys = ys[sel]; xs = xs[sel]
    z = depth_m[ys, xs].astype(np.float64)
    good = np.isfinite(z) & (z > 1e-3)
    ys, xs, z = ys[good], xs[good], z[good]
    if ys.size == 0:
        return None, 0

    Kinv = np.linalg.inv(K)
    ones = np.ones_like(xs, dtype=np.float64)
    pix = np.stack([xs.astype(np.float64), ys.astype(np.float64), ones], axis=0)
    rays = Kinv @ pix
    rays_u = rays / (np.linalg.norm(rays, axis=0, keepdims=True) + 1e-12)
    Xc = rays * z[None, :]

    n = np.asarray(ground_normal, dtype=np.float64).reshape(3)
    n = n / (np.linalg.norm(n) + 1e-12)
    R_wc = np.asarray(T_wc, dtype=np.float64)[:3, :3]
    C_w = np.asarray(T_wc, dtype=np.float64)[:3, 3]
    g_c = R_wc.T @ n
    h_signed = -(float(n @ C_w) + float(ground_d))

    down = g_c @ rays_u                     # ray 在地面法向上的分量 (朝地面时非零)
    denom = g_c @ Xc
    valid = (np.abs(down) > downward_min) & (np.abs(denom) > 1e-6)
    s = np.where(valid, h_signed / denom, np.nan)
    s_pos = s[valid & (s > 0)]
    if s_pos.size < max(50, min_floor_pixels // 4):
        if s_pos.size == 0:
            return None, int(ys.size)
    return float(np.median(s_pos)), int(s_pos.size)


def _median_filter_1d(x: np.ndarray, win: int) -> np.ndarray:
    if win <= 1:
        return x.copy()
    n = len(x)
    half = win // 2
    out = x.copy()
    for i in range(n):
        lo = max(0, i - half); hi = min(n, i + half + 1)
        out[i] = np.median(x[lo:hi])
    return out


def align_all_scales(
    keyframes: List[dict],
    depth_paths: Dict[int, str],       # kf_index -> depth npy path
    person_mask_paths: Dict[int, str], # kf_index -> mask npy path
    K: np.ndarray,
    ground_normal: np.ndarray,
    ground_d: float,
    out_dir: str,
    bottom_frac: float = 0.45,
    min_floor_pixels: int = 300,
    scale_smooth_window: int = 7,
    max_scale_jump_ratio: float = 1.25,
) -> Dict:
    """对所有关键帧估计 scale + 平滑 + 限跳变。返回统计, 落盘 depth_scales.json。"""
    raw_scales: List[Optional[float]] = []
    floor_counts: List[int] = []
    for kf in keyframes:
        ki = int(kf["kf_index"])
        dpath = depth_paths.get(ki)
        if dpath is None or not Path(dpath).exists():
            raw_scales.append(None); floor_counts.append(0); continue
        depth = np.load(dpath)
        pmask = None
        mp = person_mask_paths.get(ki)
        if mp and Path(mp).exists():
            pmask = np.load(mp)
        T = np.asarray(kf["T_wc"], dtype=np.float64)
        s, nfloor = estimate_frame_scale(
            depth, K, T, ground_normal, ground_d,
            person_mask=pmask, bottom_frac=bottom_frac,
            min_floor_pixels=min_floor_pixels)
        raw_scales.append(s); floor_counts.append(nfloor)

    # Depth Anything V2 **Metric** 输出跨帧尺度一致, 因此用**单一全局尺度**
    # (对 metric 深度这是最鲁棒的做法; per-frame scale 主要针对 relative 深度模型)。
    # 全局尺度 = 所有有效帧 per-frame median 的 trimmed median。
    valid_vals = np.array([s for s in raw_scales if s is not None and np.isfinite(s) and s > 0],
                          dtype=np.float64)
    if valid_vals.size == 0:
        global_scale = 1.0
    else:
        # trimmed: 去掉最高/最低 15%
        lo, hi = np.percentile(valid_vals, [15, 85])
        core = valid_vals[(valid_vals >= lo) & (valid_vals <= hi)]
        global_scale = float(np.median(core if core.size else valid_vals))

    # 每帧诊断值 (raw 或 fallback 到 global)
    filled = np.asarray([
        (s if (s is not None and np.isfinite(s) and s > 0) else global_scale)
        for s in raw_scales
    ], dtype=np.float64)

    # 限跳变 (仅诊断): 逐帧 clamp
    r = float(max_scale_jump_ratio)
    clamped = filled.copy()
    for i in range(1, len(clamped)):
        clamped[i] = float(np.clip(clamped[i], clamped[i - 1] / r, clamped[i - 1] * r))

    # 最终每帧尺度 = 全局尺度 (统一, metric 模型一致)
    smoothed = np.full(len(raw_scales), global_scale, dtype=np.float64)

    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    records = []
    for i, kf in enumerate(keyframes):
        records.append({
            "kf_index": int(kf["kf_index"]),
            "src_frame_index": int(kf["src_frame_index"]),
            "scale_raw": (None if raw_scales[i] is None else float(raw_scales[i])),
            "scale_filled": float(filled[i]),
            "scale_clamped": float(clamped[i]),
            "scale_final": float(smoothed[i]),
            "floor_pixels": int(floor_counts[i]),
        })
    payload = {
        "n_keyframes": len(keyframes),
        "mode": "global_metric_scale",
        "global_scale": float(global_scale),
        "n_valid_frames_for_global": int(valid_vals.size),
        "per_frame_raw_median": float(np.median(valid_vals)) if valid_vals.size else None,
        "per_frame_raw_std": float(np.std(valid_vals)) if valid_vals.size else None,
        "n_floor_insufficient": int(sum(1 for c in floor_counts if c < min_floor_pixels)),
        "n_scale_failed": int(sum(1 for s in raw_scales if s is None)),
        "scale_final_min": float(np.min(smoothed)),
        "scale_final_max": float(np.max(smoothed)),
        "scale_final_median": float(np.median(smoothed)),
        "scale_smooth_window": scale_smooth_window,
        "max_scale_jump_ratio": max_scale_jump_ratio,
        "records": records,
    }
    (out / "depth_scales.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    # 曲线图
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        xs = [r["kf_index"] for r in records]
        plt.figure(figsize=(11, 4))
        raw_plot = [r["scale_raw"] if r["scale_raw"] else np.nan for r in records]
        plt.plot(xs, raw_plot, ".", ms=3, alpha=0.5, label="raw")
        plt.plot(xs, [r["scale_clamped"] for r in records], "-", lw=0.8, alpha=0.6, label="clamped")
        plt.plot(xs, [r["scale_final"] for r in records], "-", lw=1.6, label="final (smoothed)")
        plt.xlabel("keyframe index"); plt.ylabel("scale (DPVO unit / meter)")
        plt.title("Per-frame depth scale alignment")
        plt.legend(); plt.grid(alpha=0.3)
        plt.tight_layout()
        plt.savefig(str(out / "depth_scale_curve.png"), dpi=90)
        plt.close()
    except Exception as e:
        print(f"[scale_alignment] curve plot failed: {e}")

    return payload
