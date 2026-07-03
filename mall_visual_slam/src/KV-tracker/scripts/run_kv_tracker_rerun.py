#!/usr/bin/env python3
"""跑官方 KV-Tracker 的 Rerun 实时可视化, 并把会话保存到 .rrd 文件。

和 ``run_official_kv_tracker.py`` 的关键差异:
* 不用 subprocess, 而是 inline 把官方 ``main.run_track3r()`` 直接 import 进来。
* 在 import 前 monkey-patch ``rerun.init`` 强制 ``spawn=False``,
  并在 ``run_track3r()`` 跑完后调用 ``rerun.save(...)`` 把整段会话写成 ``.rrd``。
* 默认**不开** ``--cam_only``, 让 main.py 输出局部点云 + 关键帧 RGB,
  这样回放时能看到论文 demo 风格的"相机轨迹 + 局部建图 + 置信度"。

用法:

    python src/KV-tracker/scripts/run_kv_tracker_rerun.py \
        --official-root "project code/KV-tracker/kv_tracker-main" \
        --config src/KV-tracker/config/mall_video.yaml \
        --resize-dim 308 \
        --kf-auto 50 \
        --rrd output/kv_track3r_repro/rerun_recording.rrd

回放:

    rerun output/kv_track3r_repro/rerun_recording.rrd

或者:

    python -m rerun output/kv_track3r_repro/rerun_recording.rrd
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


def _patch_rerun(rrd_path: Path):
    """Monkey-patch rerun.init/spawn 使会话只写到磁盘, 不弹 viewer。

    返回一个 close 函数, 在 run_track3r 退出后调用以 flush 写盘。
    """
    import rerun as rr

    print(f"[rerun_runner] will save Rerun session to: {rrd_path}")
    rrd_path.parent.mkdir(parents=True, exist_ok=True)

    orig_init = rr.init
    saved = {"done": False}

    def patched_init(application_id, *args, **kwargs):
        # 永远 spawn=False, 由我们自己 save
        kwargs["spawn"] = False
        result = orig_init(application_id, *args, **kwargs)
        # 立刻 save, 之后所有 log 都会写到这个 stream
        rr.save(str(rrd_path))
        saved["done"] = True
        print(f"[rerun_runner] rerun.save() wired to {rrd_path}")
        return result

    rr.init = patched_init

    def finalize():
        if not saved["done"]:
            print("[rerun_runner] no rr.init() was called by main.py; .rrd not produced")
        else:
            # 0.33 没有 explicit flush API; save 是 streaming, 退出时自动 flush
            print(f"[rerun_runner] finalized .rrd at {rrd_path}")

    return finalize


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--official-root", default=str(DEFAULT_OFFICIAL_ROOT))
    p.add_argument(
        "--config",
        default=str(REPO_ROOT / "src" / "KV-tracker" / "config" / "mall_video.yaml"),
    )
    p.add_argument("--rrd", default=str(REPO_ROOT / "output" / "kv_track3r_repro" / "rerun_recording.rrd"))
    p.add_argument("--cam-only", action="store_true",
                   help="forward --cam_only to main.py (default OFF for richer rerun)")
    p.add_argument("--resize-dim", type=int, default=308)
    p.add_argument("--kf-auto", type=int, default=50)
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

    # 让官方 main.py 能从相对路径找到 SAM2 ckpt
    _ensure_sam2_checkpoint_visible(official_root)

    # 切到官方目录 (main.py 内部用了相对路径 "thirdparty/...")
    os.chdir(official_root)
    print(f"[rerun_runner] cwd -> {official_root}")
    sys.path.insert(0, str(official_root))

    # 构造 sys.argv 给 main.py 的 argparse
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
    print(f"[rerun_runner] sys.argv = {argv}")

    # MUST patch rerun BEFORE importing main (main does `from kv_tracker.rerun_tools import *`)
    finalize = _patch_rerun(rrd_path)

    # 捕获 Ctrl+C / kill, 让 rerun 正常 flush
    def _on_signal(signum, frame):
        print(f"\n[rerun_runner] caught signal {signum}, finalizing rerun...")
        finalize()
        raise KeyboardInterrupt()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        # 现在 import 官方 main 模块 (会触发 mp.set_start_method 等)
        import main as official_main
        official_main.run_track3r()
    except KeyboardInterrupt:
        print("[rerun_runner] interrupted by user")
    except Exception as e:
        print(f"[rerun_runner] run_track3r raised: {e}")
        raise
    finally:
        finalize()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
