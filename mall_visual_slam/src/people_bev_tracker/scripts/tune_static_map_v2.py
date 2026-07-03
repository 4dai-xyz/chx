#!/usr/bin/env python3
"""V2: 静态地图参数搜索。

对 route_A_v2.yaml 里定义的 static_map_v2 参数网格逐一构建 static map,
计算质量指标并按 score 排序, 复制最优候选到 output/route_A_v2/best/。

输入:
    --config       src/people_bev_tracker/config/route_A_v2.yaml
    --pose         output/route_A/trajectory_flat.txt          (DPVO 平面化)
    --pointcloud   output/route_A/pointcloud_vggt.npy          (V1 过滤后)
    --ground-plane output/route_A/ground_plane_final.json      (V1 拟合结果, sign 已翻)
    --output-dir   output/route_A_v2

输出:
    output/route_A_v2/candidates/<cand_id>/
        static_map.npy / .png / meta.json
        nav_binary_map.png / static_map_tricolor.png / paper_style_global_view.png
        debug_*.png
        quality.json
    output/route_A_v2/best/  ← 得分第一名的候选拷贝
    output/route_A_v2/tune_report.json (所有候选一览)
    output/route_A_v2/tune_report.md
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from itertools import product
from pathlib import Path
from typing import Dict, List

import cv2
import numpy as np

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from people_bev_tracker.io_utils import load_config
from people_bev_tracker.pointcloud_io import load_points
from people_bev_tracker.static_map import (
    build_static_map_v2,
    save_static_map_v2,
    _rot_matrix_align_a_to_b,
)
from people_bev_tracker.map_quality import (
    evaluate_grid_quality,
    check_thresholds,
)
from people_bev_tracker.camera_model import load_intrinsics
from people_bev_tracker.pose_io import load_tum_trajectory


def _resolve(p: str, root: Path) -> str:
    pp = Path(p)
    return str(pp if pp.is_absolute() else (root / p).resolve())


def _load_trajectory_xyz(path: str) -> np.ndarray:
    arr = np.loadtxt(path, comments="#")
    if arr.ndim == 1:
        arr = arr[None, :]
    return arr[:, 1:4].astype(np.float64)


def _build_poses_by_frame(
    tum_path: str,
    dpvo_stride: int,
    video_fps: float,
) -> Dict[int, np.ndarray]:
    """DPVO tick 时间戳 → 源帧 index 对应 (frame_index * stride)。"""
    poses = load_tum_trajectory(
        tum_path, pose_is_twc=True, scale=1.0,
        timestamp_unit="dpvo_tick",
        dpvo_stride=dpvo_stride, video_fps=video_fps,
    )
    d = {}
    for p in poses:
        d[int(p.frame_index)] = np.asarray(p.T_wc, dtype=np.float64)
    return d


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--pose", required=True)
    ap.add_argument("--pointcloud", required=True)
    ap.add_argument("--ground-plane", required=True)
    ap.add_argument("--output-dir", required=True)
    ap.add_argument("--calib", default=None,
                    help="camera calib (default from config.input.calib)")
    ap.add_argument("--video-fps", type=float, default=29.417)
    ap.add_argument("--max-candidates", type=int, default=None)
    args = ap.parse_args()

    root = Path(__file__).resolve().parents[3]
    cfg = load_config(args.config)
    sm_cfg = cfg["static_map_v2"]

    pose_path = _resolve(args.pose, root)
    pcd_path = _resolve(args.pointcloud, root)
    gp_path = _resolve(args.ground_plane, root)
    out_dir = Path(_resolve(args.output_dir, root))
    (out_dir / "candidates").mkdir(parents=True, exist_ok=True)
    (out_dir / "best").mkdir(parents=True, exist_ok=True)

    calib_path = _resolve(args.calib or cfg["input"]["calib"], root)

    # 1) load pointcloud + trajectory
    print(f"[tune_v2] pointcloud = {pcd_path}")
    pts_w = load_points(pcd_path)      # 已经过 V1 过滤过, 不再重复过滤
    tj_xyz = _load_trajectory_xyz(pose_path)
    print(f"[tune_v2] pts = {pts_w.shape[0]}, traj = {tj_xyz.shape[0]}")

    # 2) load ground plane
    gp = json.load(open(gp_path, "r", encoding="utf-8"))
    best_gp = gp["best"] if "best" in gp else gp
    ng = np.asarray(best_gp["normal"], dtype=np.float64)
    dg = float(best_gp["d"])
    print(f"[tune_v2] ground: n={ng.tolist()} d={dg:.4f}")

    # 3) camera intrinsics (只在启用 semantic 时用)
    K, _ = load_intrinsics(calib_path, video_size=(1920, 1080))

    # 4) poses by frame (源帧 index)
    poses_by_frame = _build_poses_by_frame(pose_path, dpvo_stride=2, video_fps=args.video_fps)

    # 5) 自动 origin: 拿轨迹在 aligned frame 里的中心
    R_align = _rot_matrix_align_a_to_b(ng, np.array([0.0, 1.0, 0.0]))
    traj_a = (R_align @ tj_xyz.T).T
    origin_x = float((traj_a[:, 0].min() + traj_a[:, 0].max()) / 2)
    origin_z = float((traj_a[:, 2].min() + traj_a[:, 2].max()) / 2)
    print(f"[tune_v2] auto origin = ({origin_x:.3f}, {origin_z:.3f})")

    # 6) obstacle height range: 从 V1 meta 里取, 否则 fallback
    v1_meta_path = cfg["input"].get("v1_meta_json")
    if sm_cfg.get("obstacle_height_range"):
        obst_range = list(sm_cfg["obstacle_height_range"])
    elif v1_meta_path and Path(_resolve(v1_meta_path, root)).exists():
        v1_meta = json.load(open(_resolve(v1_meta_path, root), "r", encoding="utf-8"))
        obst_range = list(v1_meta.get("obstacle_height_range",
                                      sm_cfg.get("fallback_obstacle_height_range", [0.04, 0.75])))
    else:
        obst_range = list(sm_cfg.get("fallback_obstacle_height_range", [0.04, 0.75]))
    print(f"[tune_v2] obstacle_height_range = {obst_range}")

    # 7) 参数网格 (笛卡尔积, 截断到 max_candidates)
    grids = {
        "resolution":            sm_cfg.get("resolution_candidates", [0.008]),
        "count_thresh":          sm_cfg.get("obstacle_count_thresh_candidates", [1]),
        "gauss_sigma":           sm_cfg.get("obstacle_gaussian_sigma_candidates", [1.0]),
        "close_kernel":          sm_cfg.get("obstacle_close_kernel_candidates", [9]),
        "dilate_kernel":         sm_cfg.get("obstacle_dilate_kernel_candidates", [5]),
        "corridor_r":            sm_cfg.get("free_corridor_radius_unit_candidates", [0.20]),
        "free_close_kernel":     sm_cfg.get("free_close_kernel_candidates", [15]),
        "density_percentile":    sm_cfg.get("obstacle_density_percentile_candidates", [0]),
    }

    cap_n = int(args.max_candidates or sm_cfg.get("max_candidates", 32) or 32)

    combos = list(product(
        grids["resolution"],
        grids["count_thresh"],
        grids["gauss_sigma"],
        grids["close_kernel"],
        grids["dilate_kernel"],
        grids["corridor_r"],
        grids["free_close_kernel"],
        grids["density_percentile"],
    ))
    print(f"[tune_v2] {len(combos)} combinations (cap={cap_n})")
    if len(combos) > cap_n:
        combos = combos[:cap_n]

    all_results: List[Dict] = []
    render_cfg = sm_cfg.get("render", {})
    thresholds = sm_cfg.get("quality_thresholds", {})
    thresholds_relaxed = sm_cfg.get("quality_thresholds_relaxed", {})
    score_weights = sm_cfg.get("score_weights")

    W = int(sm_cfg.get("width_px", 1200))
    H = int(sm_cfg.get("height_px", 1200))
    active_margin_px = int(sm_cfg.get("active_area_margin_px", 120))

    t0 = time.time()
    best_score = -1e30
    best_cand_dir = None
    best_meta = None
    best_grid = None
    best_debug = None

    for i, (res, cnt, gs, ck, dk, cor, fck, dp) in enumerate(combos):
        cfg_i = {
            "resolution_unit_per_px": res,
            "width_px": W, "height_px": H,
            "origin_world": [origin_x, origin_z],
            "bev_axes": ["x", "z"],
            "obstacle_height_range": obst_range,
            "obstacle_count_thresh": cnt,
            "obstacle_gaussian_sigma": gs,
            "obstacle_density_percentile": dp,
            "obstacle_close_kernel": ck,
            "obstacle_dilate_kernel": dk,
            "obstacle_min_component_area_px": int(sm_cfg.get("obstacle_min_component_area_px", 20)),
            "free_corridor_radius_unit": cor,
            "free_frustum_enable": bool(sm_cfg.get("free_frustum_enable", True)),
            "free_frustum_stride_frames": int(sm_cfg.get("free_frustum_stride_frames", 15)),
            "free_frustum_range_unit": float(sm_cfg.get("free_frustum_range_unit", 0.8)),
            "free_frustum_half_fov_deg": float(sm_cfg.get("free_frustum_half_fov_deg", 35.0)),
            "free_close_kernel": fck,
            "free_min_component_area_px": int(sm_cfg.get("free_min_component_area_px", 100)),
            "use_semantic_floor_mask": bool(sm_cfg.get("use_semantic_floor_mask", False)),
            "semantic_stride_frames": int(sm_cfg.get("semantic_stride_frames", 30)),
            "floor_hsv_lower": list(sm_cfg.get("floor_hsv_lower", [15, 60, 60])),
            "floor_hsv_upper": list(sm_cfg.get("floor_hsv_upper", [45, 255, 255])),
            "floor_sample_step_px": int(sm_cfg.get("floor_sample_step_px", 12)),
            "floor_projection_max_range_unit": float(sm_cfg.get("floor_projection_max_range_unit", 1.5)),
        }
        semantic_video = None
        if cfg_i["use_semantic_floor_mask"]:
            sm_path = cfg["input"].get("semantic_mask_video")
            if sm_path:
                sm_full = _resolve(sm_path, root)
                if Path(sm_full).exists():
                    semantic_video = sm_full

        cand_id = f"cand_{i:03d}"
        cand_dir = out_dir / "candidates" / cand_id
        cand_dir.mkdir(parents=True, exist_ok=True)

        try:
            grid, meta, debug = build_static_map_v2(
                points_world=pts_w,
                trajectory_world_xyz=tj_xyz,
                ground_plane={"normal": ng.tolist(), "d": dg},
                cfg=cfg_i,
                poses_T_wc_by_frame=poses_by_frame,
                K=K,
                semantic_mask_video=semantic_video,
                return_debug=True,
            )
        except Exception as e:
            print(f"[tune_v2] {cand_id} FAILED: {e}")
            continue

        # 保存 candidate 图片
        save_static_map_v2(
            grid=grid, meta=meta, out_dir=str(cand_dir),
            debug=debug, render_cfg=render_cfg,
            trajectory_ij=debug["trajectory_ij"],
        )

        # 质量
        q = evaluate_grid_quality(
            grid, meta,
            trajectory_ij=debug["trajectory_ij"],
            small_component_area_px=int(sm_cfg.get("obstacle_min_component_area_px", 20)),
            active_margin_px=active_margin_px,
            score_weights=score_weights,
        )
        q["thresholds_strict"] = check_thresholds(q, thresholds)
        q["thresholds_relaxed"] = check_thresholds(q, thresholds_relaxed)
        q["cfg"] = cfg_i
        q["cand_id"] = cand_id
        (cand_dir / "quality.json").write_text(
            json.dumps(q, ensure_ascii=False, indent=2), encoding="utf-8")

        print(
            f"[tune_v2] {cand_id}  "
            f"res={res}  cnt={cnt} gs={gs} ck={ck} dk={dk}  "
            f"cor={cor} fck={fck} dp={dp}  "
            f"free={q['active_free_ratio']*100:.1f}%  "
            f"unk={q['active_unknown_ratio']*100:.1f}%  "
            f"coll={q['trajectory_collision_ratio']*100:.2f}%  "
            f"small={q['obstacle_small_component_ratio']*100:.0f}%  "
            f"lf={q['largest_free_component_ratio']*100:.1f}%  "
            f"score={q['score']:.4f}"
        )

        all_results.append({"cand_id": cand_id, **q, "cfg": cfg_i})

        if q["score"] > best_score:
            best_score = q["score"]
            best_cand_dir = cand_dir
            best_meta = meta
            best_grid = grid
            best_debug = debug

    dt = time.time() - t0

    # 复制 best/
    best_dir = out_dir / "best"
    if best_cand_dir is not None:
        for f in best_cand_dir.iterdir():
            shutil.copy2(f, best_dir / f.name)
        print(f"[tune_v2] best = {best_cand_dir.name}  score={best_score:.4f}")

    # 汇总报告
    tune_json = {
        "n_candidates": len(all_results),
        "elapsed_sec": dt,
        "best_cand_id": best_cand_dir.name if best_cand_dir else None,
        "best_score": best_score,
        "input": {
            "pointcloud": pcd_path,
            "pose": pose_path,
            "ground_plane": gp_path,
            "calib": calib_path,
            "semantic_mask_video": cfg["input"].get("semantic_mask_video"),
        },
        "obstacle_height_range": obst_range,
        "origin_world_used": [origin_x, origin_z],
        "results_top10": sorted(all_results, key=lambda x: -x["score"])[:10],
    }
    (out_dir / "tune_report.json").write_text(
        json.dumps(tune_json, ensure_ascii=False, indent=2), encoding="utf-8")

    md = [
        "# route_A_v2 静态地图参数搜索报告",
        "",
        f"* 候选数: {len(all_results)}",
        f"* 用时: {dt:.1f} s",
        f"* 最优 candidate: **{tune_json['best_cand_id']}**  (score = {best_score:.4f})",
        "",
        "## 排名前 10",
        "",
        "| rank | cand | score | active_free | active_unk | collision | small_cc | largest_free |",
        "|---:|:---|---:|---:|---:|---:|---:|---:|",
    ]
    for rk, r in enumerate(tune_json["results_top10"], start=1):
        md.append(
            f"| {rk} | {r['cand_id']} | {r['score']:.4f} | "
            f"{r['active_free_ratio']*100:.1f}% | "
            f"{r['active_unknown_ratio']*100:.1f}% | "
            f"{r['trajectory_collision_ratio']*100:.2f}% | "
            f"{r['obstacle_small_component_ratio']*100:.0f}% | "
            f"{r['largest_free_component_ratio']*100:.1f}% |"
        )
    (out_dir / "tune_report.md").write_text("\n".join(md), encoding="utf-8")

    print(f"[tune_v2] done in {dt:.1f}s, best={tune_json['best_cand_id']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
