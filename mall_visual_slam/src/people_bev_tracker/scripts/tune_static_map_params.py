#!/usr/bin/env python3
"""扫描 static_map 关键参数, 输出多张候选 png 给人挑。

参数网格:
    obstacle_height_range: [(0.02,0.35), (0.02,0.25), (0.03,0.30), (0.01,0.40)]
    count_thresh:          [3, 5, 8, 12]
    dilate_kernel:         [1, 3, 5]
"""

from __future__ import annotations

import argparse
import json
import sys
from itertools import product
from pathlib import Path

import cv2
import numpy as np

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from people_bev_tracker.io_utils import load_config
from people_bev_tracker.pointcloud_io import load_points, robust_filter_points
from people_bev_tracker.ground_fit import fit_ground_all_methods
from people_bev_tracker.static_map import build_static_map, render_static_map


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--config", required=True)
    p.add_argument("--pose", required=True)
    p.add_argument("--pointcloud", required=True)
    p.add_argument("--output-dir", required=True)
    args = p.parse_args()

    cfg = load_config(args.config)
    outdir = Path(args.output_dir)
    outdir.mkdir(parents=True, exist_ok=True)

    pts = load_points(args.pointcloud, max_points=int(cfg["pointcloud"].get("max_points", 500000)))
    pts = robust_filter_points(pts, percentile=tuple(cfg["pointcloud"].get("outlier_percentile", [1.0, 99.0])))
    traj_xyz = np.loadtxt(args.pose, comments="#")[:, 1:4].astype(np.float64)
    gp = fit_ground_all_methods(pts, cfg["ground"])["best"]
    print(f"[tune] ground: {gp['method']}  inlier_ratio={gp.get('inlier_ratio',0):.3f}")

    heights = [(0.02, 0.35), (0.02, 0.25), (0.03, 0.30), (0.01, 0.40)]
    thresholds = [3, 5, 8, 12]
    dilations = [1, 3, 5]

    grid_report = []
    base_cfg = dict(cfg["static_map"])
    for h, t, dk in product(heights, thresholds, dilations):
        sc = dict(base_cfg)
        sc["obstacle_height_range"] = list(h)
        sc["count_thresh"] = t
        sc["dilate_kernel"] = dk
        g, meta = build_static_map(
            points_world=pts,
            trajectory_world_xyz=traj_xyz,
            ground_plane=gp,
            cfg=sc,
        )
        name = f"h{h[0]:.2f}-{h[1]:.2f}_t{t}_dk{dk}.png"
        cv2.imwrite(str(outdir / name), render_static_map(g, meta))
        grid_report.append({
            "file": name,
            "obstacle_height_range": list(h),
            "count_thresh": t,
            "dilate_kernel": dk,
            "occupied_ratio": meta["statistics"]["occupied_ratio"],
            "free_ratio": meta["statistics"]["free_ratio"],
            "unknown_ratio": meta["statistics"]["unknown_ratio"],
        })
        print(f"[tune] {name}  occ={meta['statistics']['occupied_ratio']*100:.2f}%  "
              f"free={meta['statistics']['free_ratio']*100:.2f}%")

    with open(outdir / "tune_report.json", "w") as f:
        json.dump({"ground": gp, "results": grid_report}, f, ensure_ascii=False, indent=2)
    print(f"[tune] {len(grid_report)} candidates -> {outdir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
