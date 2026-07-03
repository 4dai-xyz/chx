"""把官方 KV-Tracker 落盘的 .npy 转成更友好的 TUM/JSON/PLY/CSV.

输入 (官方 main.py 在 results_path 下写出的):
    traj.npy        (N, 4, 4)   每帧 T_wc (相机到世界)
    kf_poses.npy    (K, 4, 4)   关键帧 T_wc
    kf_idx.npy      (K,)        关键帧对应的原始帧 idx
    pcd.npy         list of (P_i, 3)  每隔 ~40 帧的局部点云 (非 cam_only)
    pcd_<idx>.ply   关键帧点云 (官方导出, 仅在 --export_pcd)
    kf_<i>.png      关键帧 RGB 截图

输出 (本模块统一名字):
    trajectory.npy
    trajectory.json
    trajectory_tum.txt
    keyframe_poses.npy
    keyframes.json
    confidence.json   (尽量从已有信息派生)
    local_structure.npy  (拼起来的 pcd)
    local_structure.ply
    runtime.csv      (来自 --runtime-log)
"""

from __future__ import annotations

import json
import shutil
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from .export_tools import (
    save_confidence_json,
    save_keyframes_json,
    save_pointcloud_ply,
    save_runtime_csv,
    save_trajectory_json,
    save_trajectory_tum,
)


def _load_npy(p: Path):
    if not p.exists():
        return None
    try:
        return np.load(p, allow_pickle=True)
    except Exception as e:
        print(f"[output_converter] 无法读取 {p}: {e}")
        return None


def convert_official_outputs(
    official_results_dir: Path | str,
    converted_output_dir: Path | str,
    fps: float = 30.0,
    runtime_log_jsonl: Optional[Path | str] = None,
) -> dict:
    """读取官方落盘文件 → 写出 trajectory/keyframes/confidence/local_structure."""

    src = Path(official_results_dir).resolve()
    dst = Path(converted_output_dir).resolve()
    dst.mkdir(parents=True, exist_ok=True)

    summary = {"src": str(src), "dst": str(dst)}

    # ---------------- trajectory ----------------
    traj = _load_npy(src / "traj.npy")
    records = []
    if traj is not None:
        traj = np.asarray(traj, dtype=np.float64).reshape(-1, 4, 4)
        np.save(dst / "trajectory.npy", traj)
        for i, T_wc in enumerate(traj):
            records.append(
                {
                    "frame_index": i,
                    "timestamp": i / max(fps, 1e-9),
                    "T_wc": T_wc,
                    "mean_confidence": 0.0,
                    "fps": float(fps),
                    "mode": "tracking" if i > 0 else "init",
                }
            )
        save_trajectory_tum(dst / "trajectory_tum.txt", records)
        save_trajectory_json(dst / "trajectory.json", records)
        summary["n_poses"] = len(records)
    else:
        summary["n_poses"] = 0
        print(f"[output_converter] WARNING traj.npy missing in {src}")

    # ---------------- keyframes ----------------
    kf_poses = _load_npy(src / "kf_poses.npy")
    kf_idx = _load_npy(src / "kf_idx.npy")
    if kf_poses is not None:
        kf_poses = np.asarray(kf_poses, dtype=np.float64).reshape(-1, 4, 4)
        np.save(dst / "keyframe_poses.npy", kf_poses)
        if kf_idx is None:
            kf_idx = np.arange(kf_poses.shape[0])
        kf_idx = np.asarray(kf_idx, dtype=np.int64).reshape(-1)
        n = min(len(kf_poses), len(kf_idx))
        recs = []
        for i in range(n):
            recs.append(
                {
                    "kf_index": int(i),
                    "frame_index": int(kf_idx[i]),
                    "timestamp": int(kf_idx[i]) / max(fps, 1e-9),
                    "T_wc": kf_poses[i],
                    "mean_confidence": 0.0,
                }
            )
        save_keyframes_json(dst / "keyframes.json", recs)
        summary["n_keyframes"] = n
    else:
        summary["n_keyframes"] = 0

    # ---------------- local structure ----------------
    pcd = _load_npy(src / "pcd.npy")
    pcd_xyz = None
    n_pts = 0
    if pcd is not None:
        # pcd is a list of [N_i, 3] (object array)
        chunks = []
        for chunk in pcd:
            arr = np.asarray(chunk)
            if arr.ndim == 3 and arr.shape[-1] == 3:
                arr = arr.reshape(-1, 3)
            elif arr.ndim != 2 or arr.shape[-1] != 3:
                continue
            chunks.append(arr)
        if chunks:
            pcd_xyz = np.concatenate(chunks, axis=0).astype(np.float32)
            np.save(dst / "local_structure.npy", pcd_xyz)
            n_pts = save_pointcloud_ply(dst / "local_structure.ply", pcd_xyz)
    summary["n_points"] = n_pts

    # ---------------- confidence ----------------
    # 官方代码当前没有逐帧 dump confidence。先从 runtime_log_jsonl (我们 wrapper
    # 写的) 里拿 mean_confidence；否则写个空文件占位。
    conf_records: list[dict] = []
    if runtime_log_jsonl is not None and Path(runtime_log_jsonl).exists():
        with open(runtime_log_jsonl, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                except json.JSONDecodeError:
                    continue
                conf_records.append(
                    {
                        "frame_index": int(rec.get("frame_index", -1)),
                        "timestamp": float(rec.get("timestamp", 0.0)),
                        "mean_confidence": float(rec.get("mean_confidence", 0.0)),
                        "median_confidence": float(rec.get("median_confidence", 0.0)),
                        "valid_ratio": float(rec.get("valid_ratio", 1.0)),
                        "used_for_keyframe": bool(rec.get("used_for_keyframe", False)),
                        "rejected": bool(rec.get("rejected", False)),
                    }
                )
    save_confidence_json(dst / "confidence.json", conf_records)
    if conf_records:
        np.save(dst / "confidence.npy", np.array([r["mean_confidence"] for r in conf_records]))
    summary["n_conf_records"] = len(conf_records)

    # ---------------- runtime CSV ----------------
    runtime_records: list[dict] = []
    if runtime_log_jsonl is not None and Path(runtime_log_jsonl).exists():
        with open(runtime_log_jsonl, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                except json.JSONDecodeError:
                    continue
                runtime_records.append(
                    {
                        "frame_index": int(rec.get("frame_index", -1)),
                        "timestamp": float(rec.get("timestamp", 0.0)),
                        "fps": float(rec.get("fps", 0.0)),
                        "pi3_ms": float(rec.get("pi3_ms", 0.0)),
                        "total_ms": float(rec.get("total_ms", 0.0)),
                        "mode": str(rec.get("mode", "tracking")),
                    }
                )
    save_runtime_csv(dst / "runtime.csv", runtime_records)
    summary["n_runtime_records"] = len(runtime_records)

    # ---------------- mirror auxiliary outputs ----------------
    # PNG keyframe stills
    for png in src.glob("kf_*.png"):
        shutil.copy2(png, dst / png.name)
    # extra per-keyframe .ply
    for ply in src.glob("pcd_*.ply"):
        shutil.copy2(ply, dst / ply.name)

    return summary
