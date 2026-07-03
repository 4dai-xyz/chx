#!/usr/bin/env python3
"""路线 A 静态地图 + 轨迹平面化 一键构建 (阶段 1-4)。

输入:
    --config      src/people_bev_tracker/config/route_A.yaml
    --pose        output/dpvo/trajectory_tum.txt
    --pointcloud  "project code/DPVO/mall_dpvo.ply"
    --output-dir  output/route_A

输出:
    output/route_A/
      ├── ground_plane_final.json       # 3 方法候选 + 选中的
      ├── trajectory_flat.txt           # 平面化后的 DPVO TUM
      ├── static_map.npy                # (H, W) uint8  0/127/255
      ├── static_map.png                # 渲染的 BGR 图
      ├── static_map_meta.json          # 分辨率 / 原点 / colors / 统计
      ├── pointcloud_input_summary.json
      └── route_A_build_report.md
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Dict

import numpy as np

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from people_bev_tracker.io_utils import load_config
from people_bev_tracker.pointcloud_io import (
    load_points,
    robust_filter_points,
    save_points_npy,
    summarize_points,
    trajectory_proximity_filter,
)
from people_bev_tracker.ground_fit import fit_ground_all_methods
from people_bev_tracker.trajectory_flatten import flatten_trajectory
from people_bev_tracker.static_map import build_static_map, save_static_map


def _load_trajectory_xyz(tum_path: str) -> np.ndarray:
    arr = np.loadtxt(tum_path, comments="#")
    if arr.ndim == 1:
        arr = arr[None, :]
    return arr[:, 1:4].astype(np.float64)  # (N, 3)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--config", required=True)
    p.add_argument("--pose", default=None)
    p.add_argument("--pointcloud", default=None)
    p.add_argument("--output-dir", default=None)
    p.add_argument("--pointcloud-source", default=None,
                   help="override cfg pointcloud.source: dpvo | vggt | kv_aligned")
    args = p.parse_args()

    cfg = load_config(args.config)
    repo_root = Path(__file__).resolve().parents[3]

    def resolve(x: str) -> str:
        pp = Path(x)
        return str(pp if pp.is_absolute() else (repo_root / x).resolve())

    pose_path = resolve(args.pose or cfg["input"]["pose_tum"])
    pcd_source = args.pointcloud_source or cfg["pointcloud"].get("source", "dpvo")
    if args.pointcloud:
        pcd_path = resolve(args.pointcloud)
    else:
        pcd_map = {
            "dpvo": cfg["input"]["primary_pointcloud"],
            "vggt": cfg["input"].get("optional_vggt_pointcloud"),
            "kv_aligned": cfg["input"].get("optional_kv_pcd"),
        }
        pcd_path = resolve(pcd_map[pcd_source])
    output_dir = Path(resolve(args.output_dir or cfg["output"]["dir"]))
    output_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 70)
    print(f"[build_route_A] pose  = {pose_path}")
    print(f"[build_route_A] pcd   = {pcd_path}  (source={pcd_source})")
    print(f"[build_route_A] out   = {output_dir}")

    t0 = time.time()

    # ---------- 1. 加载点云 + 过滤 ----------
    max_points = int(cfg["pointcloud"].get("max_points", 0) or 0)
    points = load_points(pcd_path, max_points=max_points)
    print(f"[build_route_A] loaded pointcloud: N = {points.shape[0]}")
    pct = tuple(cfg["pointcloud"].get("outlier_percentile", [1.0, 99.0]))
    points = robust_filter_points(points, percentile=pct)
    print(f"[build_route_A] after percentile {pct}: N = {points.shape[0]}")
    # 先加载轨迹以便做 proximity filter
    traj_xyz_raw = _load_trajectory_xyz(pose_path)
    prox_ratio = float(cfg["pointcloud"].get("trajectory_proximity_max_ratio", 0) or 0)
    prox_min_r = float(cfg["pointcloud"].get("trajectory_proximity_min_radius", 1.0))
    if prox_ratio > 0:
        points = trajectory_proximity_filter(
            points, traj_xyz_raw, max_ratio=prox_ratio, min_radius=prox_min_r)
        print(f"[build_route_A] after trajectory proximity (<= {prox_ratio}× traj extent): "
              f"N = {points.shape[0]}")

    with open(output_dir / "pointcloud_input_summary.json", "w") as f:
        json.dump({
            "source": pcd_source,
            "path": pcd_path,
            "outlier_percentile": list(pct),
            "summary": summarize_points(points),
        }, f, ensure_ascii=False, indent=2)

    # 顺手 dump 一份 npy 备用
    save_points_npy(str(output_dir / f"pointcloud_{pcd_source}.npy"), points)

    # ---------- 2. 轨迹 (已在 step1 加载过, 复用) ----------
    traj_xyz = traj_xyz_raw
    print(f"[build_route_A] trajectory: N = {traj_xyz.shape[0]}")

    # ---------- 3. 地面拟合 ----------
    ground_cfg = cfg["ground"]
    result = fit_ground_all_methods(points, ground_cfg)
    best = result["best"]
    print(f"[build_route_A] ground fit best: method={best['method']} "
          f"inlier_ratio={best.get('inlier_ratio',0):.3f} "
          f"rmse={best.get('rmse',0):.4f} "
          f"angle={best.get('angle_vs_axis_hint_deg',0):.1f}°")

    with open(output_dir / "ground_plane_final.json", "w") as f:
        json.dump({
            "best": best,
            "candidates": result["candidates"],
            "input_pointcloud": pcd_path,
            "pointcloud_source": pcd_source,
        }, f, ensure_ascii=False, indent=2)

    # ---------- 4. 轨迹平面化 ----------
    flatten_mode = str(cfg["pose"].get("flatten_mode", "constant")).lower()
    lpf_hz = float(cfg["pose"].get("flatten_lpf_cutoff_hz", 0.3))
    flat_stats = flatten_trajectory(
        input_tum=pose_path,
        output_tum=str(output_dir / "trajectory_flat.txt"),
        ground_plane=best,
        mode=flatten_mode,
        lpf_cutoff_hz=lpf_hz,
    )
    print(f"[build_route_A] flatten mode={flatten_mode} "
          f"h_before.std={flat_stats['h_before']['std']:.4f} "
          f"h_after.std={flat_stats['h_after']['std']:.4f}")

    with open(output_dir / "trajectory_flat_stats.json", "w") as f:
        json.dump(flat_stats, f, ensure_ascii=False, indent=2)

    # 用平面化后的轨迹 xyz 构建 static map
    traj_xyz_flat = np.loadtxt(str(output_dir / "trajectory_flat.txt"),
                                comments="#")[:, 1:4].astype(np.float64)

    # 检查相机 h(t) 的符号: 若相机在平面 "负 h" 一侧, 翻转平面, 让相机始终 h > 0.
    # 这样"障碍点在相机上方到脚下"就统一 h ∈ (0, h_camera).
    n_g = np.asarray(best["normal"], dtype=np.float64)
    d_g = float(best["d"])
    Ch = traj_xyz_flat @ n_g + d_g
    if float(np.median(Ch)) < 0:
        n_g = -n_g
        d_g = -d_g
        best["normal"] = n_g.tolist()
        best["d"] = d_g
        best["sign_flipped_to_camera_side"] = True
        with open(output_dir / "ground_plane_final.json", "w") as f:
            json.dump({"best": best, "candidates": result["candidates"],
                       "input_pointcloud": pcd_path,
                       "pointcloud_source": pcd_source}, f, ensure_ascii=False, indent=2)
        Ch = traj_xyz_flat @ n_g + d_g
    camera_h_median = float(np.median(Ch))
    print(f"[build_route_A] camera h median = {camera_h_median:.4f}")

    # ---------- 5. static map ----------
    sm_cfg = dict(cfg["static_map"])
    # 若开启 auto_height, 根据相机高度覆写 obstacle_height_range
    # 障碍带 = 相机脚下附近到相机头附近 = [~0.05, ~0.95] × camera_h_median
    if bool(sm_cfg.get("auto_height", False)) and camera_h_median > 0:
        hmin_new = 0.05 * camera_h_median
        hmax_new = 0.95 * camera_h_median
        print(f"[build_route_A] auto obstacle_height_range = [{hmin_new:.4f}, {hmax_new:.4f}] "
              f"(from camera h median {camera_h_median:.4f})")
        sm_cfg["obstacle_height_range"] = [hmin_new, hmax_new]
        # 关掉 static_map 内部的另一套 auto_height, 直接用我们算好的
        sm_cfg["auto_height"] = False
    # auto origin: 让画布中心对准平面化 trajectory 的 BEV 中心
    if bool(sm_cfg.get("auto_origin", False)):
        bev_axes = tuple(sm_cfg.get("bev_axes", ["x", "z"]))
        idx = {"x": 0, "y": 1, "z": 2}
        a, b = idx[bev_axes[0].lower()], idx[bev_axes[1].lower()]
        # 注意 static_map 内部会先做 R_align 到点云和轨迹, 所以这里要提前 rotate.
        # 简化: 用 traj_xyz_flat (在原始 world) 平均值; static_map 内部 R_align 会一致处理.
        # (旋转会同时作用于 trajectory 和 origin_world? origin_world 是 BEV 世界系,
        # 我们让 origin 落在 aligned 空间的 BEV 中心。 提前算 R_align)
        from people_bev_tracker.static_map import _rot_matrix_align_a_to_b
        n_g = np.asarray(best["normal"], dtype=np.float64)
        R_align = _rot_matrix_align_a_to_b(n_g, np.array([0.0, 1.0, 0.0]))
        traj_a = (R_align @ traj_xyz_flat.T).T
        cx = float((traj_a[:, a].min() + traj_a[:, a].max()) / 2)
        cy = float((traj_a[:, b].min() + traj_a[:, b].max()) / 2)
        sm_cfg["origin_world"] = [cx, cy]
        print(f"[build_route_A] auto_origin -> [{cx:.3f}, {cy:.3f}] (BEV axes {bev_axes})")

    # ground_plane 里 normal 已可能被翻转, 传入 best (in-place 已更新)
    grid, meta = build_static_map(
        points_world=points,
        trajectory_world_xyz=traj_xyz_flat,
        ground_plane=best,
        cfg=sm_cfg,
    )
    save_static_map(
        grid=grid,
        meta=meta,
        npy_path=str(output_dir / "static_map.npy"),
        png_path=str(output_dir / "static_map.png"),
        meta_path=str(output_dir / "static_map_meta.json"),
    )
    print(f"[build_route_A] static_map stats: "
          f"occupied={meta['statistics']['occupied_ratio']*100:.2f}% "
          f"free={meta['statistics']['free_ratio']*100:.2f}% "
          f"unknown={meta['statistics']['unknown_ratio']*100:.2f}%")

    dt = time.time() - t0
    print(f"[build_route_A] done in {dt:.1f}s")

    # ---------- 6. build report md ----------
    md_lines = [
        "# route_A 静态地图构建报告",
        "",
        "## 输入",
        f"* 视频: (only 用来给下一阶段, 本步骤没读)",
        f"* 相机轨迹 TUM: `{pose_path}` (N = {traj_xyz.shape[0]})",
        f"* 点云: `{pcd_path}` (source = {pcd_source}, N = {points.shape[0]} 过滤后)",
        "",
        "## 地面拟合",
        f"* 选中方法: **{best['method']}**",
        f"* 法向: {best['normal']}",
        f"* d: {best['d']}",
        f"* inlier_ratio: {best.get('inlier_ratio', 0):.4f}",
        f"* RMSE: {best.get('rmse', 0):.4f}",
        f"* 法向 vs axis_hint 夹角: {best.get('angle_vs_axis_hint_deg', 0):.1f}°",
        f"* 选中原因: {best.get('selected_reason', '')}",
        "",
        "候选:",
        "",
    ]
    for c in result["candidates"]:
        md_lines.append(
            f"* {c['method']}: "
            f"inlier_ratio={c.get('inlier_ratio', 0):.3f}, "
            f"RMSE={c.get('rmse', 0):.4f}, "
            f"angle={c.get('angle_vs_axis_hint_deg', 0):.1f}°"
        )
    md_lines += [
        "",
        "## 轨迹平面化",
        f"* 模式: {flat_stats['mode']}",
        f"* 平面化前 h(t) std = {flat_stats['h_before']['std']:.4f}",
        f"* 平面化后 h(t) std = {flat_stats['h_after']['std']:.4f}  (越小越好)",
        f"* 输出: `{output_dir / 'trajectory_flat.txt'}`",
        "",
        "## 静态地图",
        f"* 分辨率: {meta['resolution_unit_per_px']} DPVO 单位 / px",
        f"* 画布: {meta['width_px']} × {meta['height_px']}",
        f"* 原点 (BEV x, z): {meta['origin_world']}",
        f"* 障碍高度带: {meta['obstacle_height_range']}",
        f"* count_thresh: {meta['count_thresh']}",
        f"* dilate_kernel: {meta['dilate_kernel']}",
        f"* free_corridor_radius_px: {meta['free_corridor_radius_px']}",
        "",
        "### 统计",
        f"* occupied: {meta['statistics']['occupied_ratio']*100:.2f}%",
        f"* free:     {meta['statistics']['free_ratio']*100:.2f}%",
        f"* unknown:  {meta['statistics']['unknown_ratio']*100:.2f}%",
        f"* 高度带内障碍点数: {meta['statistics']['n_obstacle_points_after_height_filter']}",
        "",
        "## 用时",
        f"{dt:.1f} s",
        "",
    ]
    (output_dir / "route_A_build_report.md").write_text("\n".join(md_lines), encoding="utf-8")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
