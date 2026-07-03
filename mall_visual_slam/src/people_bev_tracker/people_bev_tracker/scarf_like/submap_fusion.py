"""V3.1 阶段 6: 子图融合 + 多帧一致性 (借鉴 ScaRF-SLAM submap + projection fusion)。

流程:
  1. 每 keyframes_per_submap 个关键帧组成一个 submap, 相邻重叠 overlap_keyframes;
  2. 每帧: 用 scale s_i 把米制深度反投影为 DPVO 单位点云 (排除行人 mask、超范围深度);
  3. submap 内 voxel 下采样 + 统计每个 voxel 被几个关键帧观测;
  4. 只保留被 >= min_observations 个关键帧观测的 voxel (删单帧漂浮点/假墙);
  5. 输出每个 submap 点云 + 全局融合 dense_global_static。

产出的点是**世界系 (DPVO 单位)**, 与 DPVO 轨迹一致; 未应用 mirror_y
(mirror_y 在 occupancy 阶段做, 保持点云与原始 world 一致便于复用)。
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np


def backproject_keyframe(
    depth_m: np.ndarray,
    scale: float,
    K: np.ndarray,
    T_wc: np.ndarray,
    person_mask: Optional[np.ndarray],
    min_depth_m: float,
    max_depth_m: float,
    px_stride: int = 4,
) -> np.ndarray:
    """把一个关键帧的深度反投影为世界系点云 (DPVO 单位)。返回 (N, 3)。"""
    kf_h, kf_w = depth_m.shape
    ys, xs = np.mgrid[0:kf_h:px_stride, 0:kf_w:px_stride]
    ys = ys.ravel(); xs = xs.ravel()
    z = depth_m[ys, xs].astype(np.float64)
    valid = np.isfinite(z) & (z >= min_depth_m) & (z <= max_depth_m)
    if person_mask is not None:
        pm = person_mask
        if pm.shape != (kf_h, kf_w):
            import cv2
            pm = cv2.resize(pm.astype(np.uint8), (kf_w, kf_h),
                            interpolation=cv2.INTER_NEAREST)
        valid &= (pm[ys, xs] == 0)
    xs, ys, z = xs[valid], ys[valid], z[valid]
    if xs.size == 0:
        return np.zeros((0, 3), dtype=np.float32)

    Kinv = np.linalg.inv(K)
    ones = np.ones_like(xs, dtype=np.float64)
    pix = np.stack([xs.astype(np.float64), ys.astype(np.float64), ones], axis=0)
    Xc = (Kinv @ pix) * z[None, :]          # 相机系米制
    Xc_dpvo = Xc * float(scale)             # 换算到 DPVO 单位
    R_wc = np.asarray(T_wc, dtype=np.float64)[:3, :3]
    C_w = np.asarray(T_wc, dtype=np.float64)[:3, 3]
    Xw = (R_wc @ Xc_dpvo) + C_w[:, None]
    return Xw.T.astype(np.float32)          # (N, 3)


def _voxel_multiobs(
    points_per_frame: List[np.ndarray],
    voxel: float,
    min_obs: int,
    max_points: int,
) -> Tuple[np.ndarray, np.ndarray]:
    """把多帧点云按 voxel 聚合, 统计每 voxel 被几帧观测。
    返回 (kept_points (M,3), obs_count (M,))。只保留 obs>=min_obs 的 voxel 中心。
    """
    if not points_per_frame:
        return np.zeros((0, 3), np.float32), np.zeros((0,), np.int32)
    # voxel key -> set(frame_idx) + 累加坐标
    from collections import defaultdict
    acc = defaultdict(lambda: [np.zeros(3, np.float64), 0, set()])
    for fidx, pts in enumerate(points_per_frame):
        if pts.shape[0] == 0:
            continue
        keys = np.floor(pts / voxel).astype(np.int64)
        for p, k in zip(pts, keys):
            kk = (int(k[0]), int(k[1]), int(k[2]))
            a = acc[kk]
            a[0] += p
            a[1] += 1
            a[2].add(fidx)
    kept_pts = []
    kept_obs = []
    for kk, (sm, cnt, frames) in acc.items():
        if len(frames) >= min_obs:
            kept_pts.append(sm / cnt)
            kept_obs.append(len(frames))
    if not kept_pts:
        return np.zeros((0, 3), np.float32), np.zeros((0,), np.int32)
    pts = np.asarray(kept_pts, dtype=np.float32)
    obs = np.asarray(kept_obs, dtype=np.int32)
    if pts.shape[0] > max_points:
        idx = np.random.default_rng(0).choice(pts.shape[0], max_points, replace=False)
        pts = pts[idx]; obs = obs[idx]
    return pts, obs


def fuse_submaps(
    keyframes: List[dict],
    depth_paths: Dict[int, str],
    person_mask_paths: Dict[int, str],
    scales: Dict[int, float],
    K: np.ndarray,
    out_dir: str,
    keyframes_per_submap: int = 12,
    overlap_keyframes: int = 3,
    voxel_size_unit: float = 0.015,
    min_observations: int = 2,
    max_points_per_submap: int = 300000,
    min_depth_m: float = 0.3,
    max_depth_m: float = 15.0,
    px_stride: int = 4,
) -> Dict:
    """融合所有子图, 输出 dense_global_static.npy/.ply。"""
    out = Path(out_dir)
    (out / "submaps").mkdir(parents=True, exist_ok=True)

    n_kf = len(keyframes)
    step = max(1, keyframes_per_submap - overlap_keyframes)
    submap_reports = []
    global_pts = []
    global_obs = []

    submap_id = 0
    start = 0
    while start < n_kf:
        end = min(n_kf, start + keyframes_per_submap)
        sub_kfs = keyframes[start:end]
        per_frame_pts = []
        for kf in sub_kfs:
            ki = int(kf["kf_index"])
            dpath = depth_paths.get(ki)
            if not dpath or not Path(dpath).exists():
                per_frame_pts.append(np.zeros((0, 3), np.float32)); continue
            depth = np.load(dpath)
            pmask = None
            mp = person_mask_paths.get(ki)
            if mp and Path(mp).exists():
                pmask = np.load(mp)
            s = float(scales.get(ki, 1.0))
            pts = backproject_keyframe(
                depth, s, K, np.asarray(kf["T_wc"], np.float64),
                pmask, min_depth_m, max_depth_m, px_stride=px_stride)
            per_frame_pts.append(pts)

        kept, obs = _voxel_multiobs(
            per_frame_pts, voxel_size_unit, min_observations, max_points_per_submap)

        # 存 submap
        if kept.shape[0] > 0:
            np.save(str(out / "submaps" / f"submap_{submap_id:03d}.npy"), kept)
            _save_ply(str(out / "submaps" / f"submap_{submap_id:03d}.ply"), kept)
            global_pts.append(kept)
            global_obs.append(obs)
        bbox = (kept.min(axis=0).tolist(), kept.max(axis=0).tolist()) if kept.shape[0] else (None, None)
        rec = {
            "submap_id": submap_id,
            "kf_range": [int(sub_kfs[0]["kf_index"]), int(sub_kfs[-1]["kf_index"])],
            "n_keyframes": len(sub_kfs),
            "n_points": int(kept.shape[0]),
            "bbox_min": bbox[0],
            "bbox_max": bbox[1],
        }
        submap_reports.append(rec)
        (out / "submaps" / f"submap_{submap_id:03d}_meta.json").write_text(
            json.dumps(rec, ensure_ascii=False, indent=2), encoding="utf-8")

        submap_id += 1
        if end >= n_kf:
            break
        start += step

    # 全局融合
    if global_pts:
        allp = np.concatenate(global_pts, axis=0).astype(np.float32)
        allo = np.concatenate(global_obs, axis=0).astype(np.int32)
        # 再做一次全局 voxel 去重 (合并 submap 重叠)
        gk = np.floor(allp / voxel_size_unit).astype(np.int64)
        from collections import defaultdict
        acc = defaultdict(lambda: [np.zeros(3, np.float64), 0, 0])
        for p, k, o in zip(allp, gk, allo):
            kk = (int(k[0]), int(k[1]), int(k[2]))
            a = acc[kk]; a[0] += p; a[1] += 1; a[2] = max(a[2], int(o))
        gpts = np.asarray([v[0] / v[1] for v in acc.values()], dtype=np.float32)
        gobs = np.asarray([v[2] for v in acc.values()], dtype=np.int32)
    else:
        gpts = np.zeros((0, 3), np.float32)
        gobs = np.zeros((0,), np.int32)

    np.save(str(out / "dense_global_static.npy"), gpts)
    np.save(str(out / "dense_global_static_obs.npy"), gobs)
    _save_ply(str(out / "dense_global_static.ply"), gpts)

    payload = {
        "n_keyframes": n_kf,
        "n_submaps": submap_id,
        "keyframes_per_submap": keyframes_per_submap,
        "overlap_keyframes": overlap_keyframes,
        "voxel_size_unit": voxel_size_unit,
        "min_observations": min_observations,
        "dense_global_n_points": int(gpts.shape[0]),
        "submaps": submap_reports,
    }
    if gpts.shape[0]:
        payload["dense_global_bbox_min"] = gpts.min(axis=0).tolist()
        payload["dense_global_bbox_max"] = gpts.max(axis=0).tolist()
    (out / "submap_fusion.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    return payload


def _save_ply(path: str, pts: np.ndarray) -> None:
    n = pts.shape[0]
    with open(path, "w", encoding="utf-8") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {n}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("end_header\n")
        for p in pts:
            f.write(f"{p[0]:.5f} {p[1]:.5f} {p[2]:.5f}\n")
