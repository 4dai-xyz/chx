#!/usr/bin/env python3
"""跑官方 KV-Tracker, 把日志写到 .rrd; 过滤掉占体积的 RGB log, 得到 slim .rrd。

和 ``run_kv_tracker_rerun.py`` 的区别:
* 那个原样保留所有 ``rr.log``, 11 GB .rrd (商场 3181 帧)。
* 本脚本额外 monkey-patch ``rr.log``, 默认 skip:
    - ``latest_rgb``         (每帧 1920x1080 RGB)
    - ``keyframes/images``   (关键帧 RGB 拼图带)
  剩下的 (相机轨迹/frustum/3D 点云/FPS 曲线) 全保留。
  实测同样数据从 ~11 GB → ~300-500 MB。

用法:

    python src/KV-tracker/scripts/run_kv_tracker_rerun_slim.py \\
        --official-root "project code/KV-tracker/kv_tracker-main" \\
        --config         src/KV-tracker/config/mall_video.yaml \\
        --resize-dim 224 \\
        --kf-auto 100 \\
        --rrd            output/kv_track3r_repro/rerun_recording_slim.rrd

之后:
    rerun output/kv_track3r_repro/rerun_recording_slim.rrd
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
PKG_ROOT = Path(__file__).resolve().parents[1]
OFFICIAL_ROOT = REPO_ROOT / "project code" / "KV-tracker" / "kv_tracker-main"

if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from kv_track3r_app.official_bridge import (
    DEFAULT_OFFICIAL_ROOT,
    _ensure_sam2_checkpoint_visible,
)


# 这些 entity path 默认被 skip; CLI 可以增减
DEFAULT_SKIP_PATHS = ("latest_rgb", "keyframes/images")


def _patch_rerun_slim(rrd_path: Path, skip_paths: tuple[str, ...]):
    """同时 monkey-patch ``rr.init`` (改 spawn=False + save) 和 ``rr.log``
    (skip 指定 entity path)。
    """
    import rerun as rr

    print(f"[slim] will save Rerun session to: {rrd_path}")
    print(f"[slim] entity paths to SKIP: {skip_paths}")
    rrd_path.parent.mkdir(parents=True, exist_ok=True)

    skip_set = set(skip_paths)
    # ---------- patch rr.init ----------
    orig_init = rr.init
    saved = {"done": False}

    def patched_init(application_id, *args, **kwargs):
        kwargs["spawn"] = False
        result = orig_init(application_id, *args, **kwargs)
        rr.save(str(rrd_path))
        saved["done"] = True
        print(f"[slim] rerun.save() wired to {rrd_path}")
        return result

    rr.init = patched_init

    # ---------- patch rr.log ----------
    orig_log = rr.log
    skipped_count = {"n": 0}

    def patched_log(entity_path, *args, **kwargs):
        # entity_path 可能是 str 或 list[str]
        if isinstance(entity_path, str):
            ep = entity_path
        elif isinstance(entity_path, (list, tuple)) and entity_path:
            ep = entity_path[0] if isinstance(entity_path[0], str) else str(entity_path)
        else:
            ep = str(entity_path)
        # 精确匹配
        if ep in skip_set:
            skipped_count["n"] += 1
            return None
        return orig_log(entity_path, *args, **kwargs)

    rr.log = patched_log

    def finalize():
        print(f"[slim] skipped {skipped_count['n']} rr.log() calls "
              f"on paths {sorted(skip_set)}")
        if not saved["done"]:
            print("[slim] no rr.init() was called by main.py; .rrd not produced")
        else:
            print(f"[slim] finalized .rrd at {rrd_path}")

    return finalize


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--official-root", default=str(DEFAULT_OFFICIAL_ROOT))
    p.add_argument(
        "--config",
        default=str(REPO_ROOT / "src" / "KV-tracker" / "config" / "mall_video.yaml"),
    )
    p.add_argument(
        "--rrd",
        default=str(REPO_ROOT / "output" / "kv_track3r_repro" / "rerun_recording_slim.rrd"),
    )
    p.add_argument(
        "--skip-paths",
        nargs="*",
        default=None,
        help=f"entity paths to filter out (default {DEFAULT_SKIP_PATHS})",
    )
    p.add_argument(
        "--keep-rgb",
        action="store_true",
        help="不过滤 latest_rgb / keyframes/images (回到 11 GB 版)",
    )
    p.add_argument("--cam-only", action="store_true",
                   help="forward --cam_only to main.py (跳过点云, 适合纯轨迹场景)")
    p.add_argument("--resize-dim", type=int, default=224,
                   help="default 224 (vs 308) 省 RAM, 适合 7-8 GB 系统")
    p.add_argument("--kf-auto", type=int, default=100,
                   help="default 100 (vs 50) 关键帧间距加大, KV-cache 增长慢")
    p.add_argument("--export-pcd", action="store_true")
    p.add_argument("--sim3", action="store_true")
    args = p.parse_args()

    official_root = Path(args.official_root).resolve()
    config_path = Path(args.config).resolve()
    rrd_path = Path(args.rrd).resolve()

    if not (official_root / "main.py").exists():
        raise SystemExit(f"official main.py not found at {official_root}")
    if not config_path.exists():
        raise SystemExit(f"config not found at {config_path}")

    _ensure_sam2_checkpoint_visible(official_root)

    os.chdir(official_root)
    print(f"[slim] cwd -> {official_root}")
    sys.path.insert(0, str(official_root))

    argv = [
        "main.py",
        str(config_path),
        "--resize_dim", str(args.resize_dim),
        "--kf_auto",    str(args.kf_auto),
        "--rerun",
    ]
    if args.cam_only:
        argv.append("--cam_only")
    if args.export_pcd:
        argv.append("--export_pcd")
    if args.sim3:
        argv.append("--sim3")
    sys.argv = argv
    print(f"[slim] sys.argv = {argv}")

    if args.keep_rgb:
        skip_paths: tuple[str, ...] = ()
        print("[slim] --keep-rgb: 不过滤任何 entity path (.rrd 会很大)")
    else:
        skip_paths = tuple(args.skip_paths) if args.skip_paths is not None else DEFAULT_SKIP_PATHS

    finalize = _patch_rerun_slim(rrd_path, skip_paths)

    def _on_signal(signum, frame):
        print(f"\n[slim] caught signal {signum}, finalizing...")
        finalize()
        raise KeyboardInterrupt()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        import main as official_main
        official_main.run_track3r()
    except KeyboardInterrupt:
        print("[slim] interrupted by user")
    except Exception as e:
        print(f"[slim] run_track3r raised: {e}")
        raise
    finally:
        finalize()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
