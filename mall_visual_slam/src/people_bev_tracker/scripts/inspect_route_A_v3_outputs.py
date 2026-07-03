#!/usr/bin/env python3
"""V3.1 输出体检: 列出 route_A_v3_scarf 关键产物是否存在 + 摘要。"""

from __future__ import annotations

import json
from pathlib import Path

REPO = Path("/home/ros/ros2_orbslam3")
OUT = REPO / "output" / "route_A_v3_scarf"


def _exists(p):
    pp = OUT / p
    if pp.exists():
        sz = pp.stat().st_size / 1024
        return f"OK   {sz:8.1f} KB  {p}"
    return f"MISS            {p}"


def main() -> int:
    files = [
        "reports/00_深度后端体检与安装报告.md",
        "reports/01_V3工程结构与配置报告.md",
        "reports/02_关键帧选择报告.md",
        "reports/03_动态行人Mask报告.md",
        "reports/04_单目深度推理报告.md",
        "reports/05_深度尺度对齐报告.md",
        "reports/06_子图融合与稠密点云报告.md",
        "reports/07_二维栅格地图生成报告.md",
        "reports/08_BEV视频重渲染报告.md",
        "route_A_v3_scarf_execution_report.md",
        "keyframes/keyframes.json",
        "depth_backend_check.json",
        "depth_scales.json",
        "depth_scale_curve.png",
        "dense_global_static.ply",
        "dense_global_static.npy",
        "best/nav_binary_map.png",
        "best/static_map_tricolor.png",
        "best/paper_style_global_view.png",
        "best/topdown_3d_scene.png",
        "best/static_map.npy",
        "best/static_map_meta.json",
        "best/quality.json",
        "bev_tracking_route_A_v3_dense.mp4",
    ]
    print("=" * 60)
    print("Route A V3.1 输出体检")
    print("=" * 60)
    for f in files:
        print(_exists(f))

    q = OUT / "best" / "quality.json"
    if q.exists():
        d = json.loads(q.read_text())
        print("\n质量指标 (best/quality.json):")
        for k in ["active_free_ratio", "active_unknown_ratio",
                  "trajectory_collision_ratio", "obstacle_small_component_ratio",
                  "largest_free_component_ratio"]:
            if k in d:
                print(f"  {k}: {d[k]*100:.2f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
