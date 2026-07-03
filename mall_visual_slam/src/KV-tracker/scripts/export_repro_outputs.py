#!/usr/bin/env python3
"""把官方 KV-Tracker 落盘的 traj.npy / pcd.npy / kf_poses.npy 转成
TUM/JSON/PLY/CSV，方便下游 (people_bev_tracker / RViz / Open3D) 使用。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from kv_track3r_app.output_converter import convert_official_outputs


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument(
        "--input",
        required=True,
        help="dir that contains traj.npy / kf_poses.npy / pcd.npy from official main.py",
    )
    p.add_argument(
        "--output",
        default=None,
        help="dir to write converted outputs; default = same as --input",
    )
    p.add_argument("--fps", type=float, default=29.417)
    p.add_argument("--runtime-log", default=None)
    args = p.parse_args()

    src = Path(args.input)
    dst = Path(args.output) if args.output else src
    summary = convert_official_outputs(
        official_results_dir=src,
        converted_output_dir=dst,
        fps=args.fps,
        runtime_log_jsonl=args.runtime_log,
    )
    print(f"[export_repro_outputs] summary: {summary}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
