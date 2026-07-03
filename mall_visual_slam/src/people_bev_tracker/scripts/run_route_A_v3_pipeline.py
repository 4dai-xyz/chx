#!/usr/bin/env python3
"""V3.1 阶段 8: 用 V3 稠密静态地图重跑 BEV 视频 (相机轨迹 + 动态行人)。

薄封装: 调用 offline_pipeline_A.run(), 输出后缀 route_A_v3_dense。
静态地图默认用 V3 best (已烘焙 mirror_y); pipeline 会自动再对相机/行人坐标
应用 mirror_y (meta.bev_alignment)。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

import offline_pipeline_A as opa  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--config",
                   default=str(PKG_ROOT / "config" / "route_A_v3_scarf.yaml"))
    p.add_argument("--pose", default="output/route_A/trajectory_flat.txt")
    p.add_argument("--static-map",
                   default="output/route_A_v3_scarf/best/static_map.npy")
    p.add_argument("--static-map-meta",
                   default="output/route_A_v3_scarf/best/static_map_meta.json")
    p.add_argument("--ground-plane",
                   default="output/route_A/ground_plane_final.json")
    p.add_argument("--output-dir", default="output/route_A_v3_scarf")
    p.add_argument("--map-render-mode", default="paper",
                   choices=["paper", "binary", "tricolor", "v1"])
    p.add_argument("--output-suffix", default="route_A_v3_dense")
    p.add_argument("--max-frames", type=int, default=None)
    p.add_argument("--live", action="store_true")
    args = p.parse_args()
    # offline_pipeline_A.run 需要这些字段
    ns = argparse.Namespace(
        config=args.config, video=None, calib=None, pose=args.pose,
        static_map=args.static_map, static_map_meta=args.static_map_meta,
        ground_plane=args.ground_plane, output_dir=args.output_dir,
        live=args.live, max_frames=args.max_frames,
        map_render_mode=args.map_render_mode, output_suffix=args.output_suffix)
    return opa.run(ns)


if __name__ == "__main__":
    raise SystemExit(main())
