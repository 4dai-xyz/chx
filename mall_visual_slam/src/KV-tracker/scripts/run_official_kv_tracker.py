#!/usr/bin/env python3
"""调用官方 KV-Tracker main.py 跑用户视频。

不修改官方代码：只生成 / 传递 yaml 配置 + subprocess。
跑完之后会调用 export_repro_outputs.py 把 traj.npy / kf_poses.npy 等转
成 TUM/JSON/PLY/CSV。
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from kv_track3r_app.official_bridge import (
    DEFAULT_OFFICIAL_ROOT,
    run_official_subprocess,
)
from kv_track3r_app.output_converter import convert_official_outputs


def _write_config(
    config_path: Path,
    video: Path,
    results_path: Path,
) -> Path:
    config_path.parent.mkdir(parents=True, exist_ok=True)
    text = (
        "datasource: phoneLoader\n"
        "que_size: 1\n"
        f"results_path: {results_path}\n"
        f"scene_dir: {video}\n"
    )
    config_path.write_text(text, encoding="utf-8")
    return config_path


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument(
        "--official-root",
        default=str(DEFAULT_OFFICIAL_ROOT),
        help="official KV-Tracker code dir (read-only)",
    )
    p.add_argument(
        "--config",
        default=str(REPO_ROOT / "src" / "KV-tracker" / "config" / "mall_video.yaml"),
        help="YAML config; if --video is also given, a generated one will override",
    )
    p.add_argument("--video", default=None, help="override scene_dir in yaml")
    p.add_argument(
        "--output",
        default=str(REPO_ROOT / "output" / "kv_track3r_repro"),
        help="where to put official + converted outputs",
    )
    p.add_argument("--cam-only", action="store_true", help="forward --cam_only to main.py")
    p.add_argument("--rerun", action="store_true", help="forward --rerun to main.py")
    p.add_argument("--resize-dim", type=int, default=308)
    p.add_argument("--kf-auto", type=int, default=50)
    p.add_argument("--export-pcd", action="store_true")
    p.add_argument("--sim3", action="store_true")
    p.add_argument("--export", action="store_true",
                   help="run output conversion after main.py returns")
    p.add_argument("--no-run", action="store_true",
                   help="skip running official main.py (only convert existing outputs)")
    p.add_argument("--max-frames", type=int, default=0,
                   help="if >0, set KV_TRACK3R_MAX_FRAMES env so wrapper can shorten")
    p.add_argument("--fps", type=float, default=29.417,
                   help="video fps used to stamp timestamps in converted outputs")
    args = p.parse_args()

    official_root = Path(args.official_root).resolve()
    output_dir = Path(args.output).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    # 生成 / 解析配置
    if args.video is not None:
        gen_cfg = REPO_ROOT / "src" / "KV-tracker" / "config" / "mall_video.generated.yaml"
        config_path = _write_config(gen_cfg, Path(args.video).resolve(), output_dir)
    else:
        config_path = Path(args.config).resolve()
    print(f"[run_official_kv_tracker] using config: {config_path}")
    print(f"[run_official_kv_tracker] official root: {official_root}")
    print(f"[run_official_kv_tracker] output dir   : {output_dir}")

    rc = 0
    if not args.no_run:
        extra: list[str] = []
        if args.cam_only:
            extra.append("--cam_only")
        if args.rerun:
            extra.append("--rerun")
        extra += ["--resize_dim", str(args.resize_dim)]
        extra += ["--kf_auto", str(args.kf_auto)]
        if args.export_pcd:
            extra.append("--export_pcd")
        if args.sim3:
            extra.append("--sim3")

        env = {}
        if args.max_frames > 0:
            env["KV_TRACK3R_MAX_FRAMES"] = str(args.max_frames)
        rc = run_official_subprocess(official_root, config_path, extra, env=env)
        print(f"[run_official_kv_tracker] official main.py exit code = {rc}")

    if args.export:
        runtime_log = output_dir / "runtime_log.jsonl"
        summary = convert_official_outputs(
            official_results_dir=output_dir,
            converted_output_dir=output_dir,
            fps=args.fps,
            runtime_log_jsonl=runtime_log if runtime_log.exists() else None,
        )
        print(f"[run_official_kv_tracker] convert summary: {summary}")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
