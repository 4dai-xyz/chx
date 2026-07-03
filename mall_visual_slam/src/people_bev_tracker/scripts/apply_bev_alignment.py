#!/usr/bin/env python3
"""把 alignment_selected.json 里选好的 transform 烘焙到 V2 static_map, 生成
aligned_preview 目录下的最终对齐地图 + 3 种 render + final_frame 预览。

输入:
    --static-map        output/route_A_v2/best/static_map.npy
    --static-map-meta   output/route_A_v2/best/static_map_meta.json
    --alignment-json    output/route_A_v3_scarf/alignment_selected.json
    --camera-json       output/route_A_v2/camera_trajectory_route_A_v2.json
    --people-json       output/route_A_v2/people_tracks_route_A_v2.json
    --config            src/people_bev_tracker/config/route_A_v2.yaml  (取 render 颜色)
    --output-dir        output/route_A_v3_scarf/aligned_preview

输出:
    output/route_A_v3_scarf/aligned_preview/
    ├── static_map.npy                    (aligned uint8 0/127/255)
    ├── static_map_meta.json              (含 bev_alignment 记录)
    ├── nav_binary_map.png                (aligned 黑白)
    ├── static_map_tricolor.png           (aligned 三值)
    ├── paper_style_global_view.png       (aligned + camera trajectory + people 叠加)
    ├── final_frame_alignment_preview.png (与 paper_style 相同, 保留任务书命名)
    └── apply_alignment_report.json       (metadata)
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from people_bev_tracker.bev_alignment import (
    TRANSFORMS,
    apply_bev_alignment_xy,
    apply_bev_alignment_heading,
    bake_alignment_into_map,
    load_alignment_selected,
    load_camera_bev_and_headings,
    load_people_final_positions,
    world_xy_to_grid_ij,
)
from people_bev_tracker.io_utils import load_config
from people_bev_tracker.static_map import (
    render_nav_binary,
    render_tricolor,
    render_paper_style,
)


def _resolve(p: str, root: Path) -> str:
    pp = Path(p)
    return str(pp if pp.is_absolute() else (root / p).resolve())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--static-map", required=True)
    ap.add_argument("--static-map-meta", required=True)
    ap.add_argument("--alignment-json", required=True)
    ap.add_argument("--camera-json", required=True)
    ap.add_argument("--people-json", default=None)
    ap.add_argument("--config", default=None,
                    help="取 render.paper_style 等颜色配置 (可选)")
    ap.add_argument("--output-dir", required=True)
    args = ap.parse_args()

    root = Path(__file__).resolve().parents[3]
    map_npy = _resolve(args.static_map, root)
    map_meta_path = _resolve(args.static_map_meta, root)
    align_json = _resolve(args.alignment_json, root)
    cam_json = _resolve(args.camera_json, root)
    ppl_json = _resolve(args.people_json, root) if args.people_json else None
    out_dir = Path(_resolve(args.output_dir, root))
    out_dir.mkdir(parents=True, exist_ok=True)

    align = load_alignment_selected(align_json)
    transform = align["selected_transform"]
    if transform not in TRANSFORMS:
        raise SystemExit(f"unknown transform in alignment_selected.json: {transform!r}. "
                         f"valid: {list(TRANSFORMS)}")
    print(f"[apply_alignment] selected transform = {transform}   (source={align['source']})")

    # ---- 1) 加载原 V2 static map ----
    grid_v2 = np.load(map_npy)
    with open(map_meta_path, "r", encoding="utf-8") as f:
        meta_v2 = json.load(f)
    print(f"[apply_alignment] V2 grid = {grid_v2.shape} "
          f"origin_world={meta_v2['origin_world']}")

    # ---- 2) 烘焙 transform 到 grid + origin ----
    grid_aligned, meta_aligned = bake_alignment_into_map(
        grid_v2, meta_v2, transform, source=align["source"])
    print(f"[apply_alignment] aligned grid = {grid_aligned.shape} "
          f"new origin_world={meta_aligned['origin_world']}")

    np.save(str(out_dir / "static_map.npy"), grid_aligned)
    with open(out_dir / "static_map_meta.json", "w", encoding="utf-8") as f:
        json.dump(meta_aligned, f, ensure_ascii=False, indent=2)

    # ---- 3) 三种 render (grid 已 baked, 直接渲染) ----
    render_cfg = {}
    if args.config:
        cfg = load_config(_resolve(args.config, root))
        render_cfg = (cfg.get("static_map_v2", {}) or {}).get("render", {}) or {}

    nav_bin = render_nav_binary(grid_aligned, render_cfg.get("nav_binary"))
    tri = render_tricolor(grid_aligned, render_cfg.get("tricolor"))

    cv2.imwrite(str(out_dir / "nav_binary_map.png"), nav_bin)
    cv2.imwrite(str(out_dir / "static_map_tricolor.png"), tri)

    # ---- 4) paper_style + camera + people overlay ----
    # 相机轨迹和 heading 也要应用 transform, 因为 meta 已经 baked 到新 frame。
    R_align = np.asarray(meta_v2.get("R_align", np.eye(3).tolist()), dtype=np.float64)
    bev_axes = tuple(meta_v2.get("bev_axes", ["x", "z"]))
    traj_bev_v2, heading_samples_v2 = load_camera_bev_and_headings(
        cam_json, R_align, bev_axes=bev_axes, heading_stride=100)

    # 把 V2 aligned world BEV → V3 aligned world BEV (再走一次 transform)
    align_cfg = {"enabled": True, "transform": transform}
    traj_bev_v3 = apply_bev_alignment_xy(traj_bev_v2, align_cfg)
    traj_ij = world_xy_to_grid_ij(traj_bev_v3, meta_aligned)

    people_ij = None
    if ppl_json and Path(ppl_json).exists():
        ppl_v2 = load_people_final_positions(ppl_json, use_last_frames=50)
        if ppl_v2.shape[0]:
            ppl_v3 = apply_bev_alignment_xy(ppl_v2, align_cfg)
            people_ij = world_xy_to_grid_ij(ppl_v3, meta_aligned)

    paper = render_paper_style(
        grid_aligned,
        trajectory_ij=traj_ij,
        people_ij=people_ij,
        colors=render_cfg.get("paper_style"),
    )
    cv2.imwrite(str(out_dir / "paper_style_global_view.png"), paper)
    # 任务书里的 "final_frame_alignment_preview.png" 就是 paper_style + 全轨迹 + 全行人
    cv2.imwrite(str(out_dir / "final_frame_alignment_preview.png"), paper)

    # ---- 5) 报告 ----
    report = {
        "selected_transform": transform,
        "alignment_source": align["source"],
        "confirmed_at": align.get("confirmed_at"),
        "input": {
            "static_map_v2": map_npy,
            "static_map_meta_v2": map_meta_path,
            "camera_json_v2": cam_json,
            "people_json_v2": ppl_json,
        },
        "v2_origin_world": meta_v2["origin_world"],
        "v2_grid_shape": [int(grid_v2.shape[0]), int(grid_v2.shape[1])],
        "aligned_origin_world": meta_aligned["origin_world"],
        "aligned_grid_shape": [int(grid_aligned.shape[0]), int(grid_aligned.shape[1])],
        "n_trajectory_points": int(traj_bev_v3.shape[0]),
        "n_people": int(people_ij.shape[0]) if people_ij is not None else 0,
        "outputs": {
            "static_map_npy": str(out_dir / "static_map.npy"),
            "static_map_meta_json": str(out_dir / "static_map_meta.json"),
            "nav_binary_map": str(out_dir / "nav_binary_map.png"),
            "static_map_tricolor": str(out_dir / "static_map_tricolor.png"),
            "paper_style_global_view": str(out_dir / "paper_style_global_view.png"),
            "final_frame_alignment_preview": str(out_dir / "final_frame_alignment_preview.png"),
        },
    }
    with open(out_dir / "apply_alignment_report.json", "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)

    print(f"[apply_alignment] done. output = {out_dir}")
    for k, v in report["outputs"].items():
        print(f"    {k}  ->  {v}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
