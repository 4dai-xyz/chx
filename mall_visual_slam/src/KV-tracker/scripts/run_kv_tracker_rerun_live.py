#!/usr/bin/env python3
"""跑官方 KV-Tracker 并把 Rerun 日志推给**已经在运行**的 Rerun viewer (不落盘)。

和 ``run_kv_tracker_rerun.py`` 的区别:
* 那个写 .rrd 文件 (回放用)。
* 本脚本只把日志推给 viewer (实时看), 不写盘, 不爆磁盘。

用法 (两个终端):

    # 终端 A
    rerun --port 9876

    # 终端 B
    RERUN_CONNECT_ADDR=127.0.0.1:9876 \\
    python src/KV-tracker/scripts/run_kv_tracker_rerun_live.py \\
        --official-root "project code/KV-tracker/kv_tracker-main" \\
        --config src/KV-tracker/config/mall_video.yaml \\
        --resize-dim 308

注意去掉 ``--cam-only``, 否则只能看相机, 看不到稠密点云。
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
PKG_ROOT = Path(__file__).resolve().parents[1]

if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from kv_track3r_app.official_bridge import (
    DEFAULT_OFFICIAL_ROOT,
    _ensure_sam2_checkpoint_visible,
)


def _patch_rerun_live(addr: str | None):
    """让官方 main.py 里的 ``rr.init(..., spawn=True)`` 改成
    ``rr.init(..., spawn=False)`` + ``rr.connect_grpc(url)``。

    ``addr`` 可以是:
        * 完整 URL ``rerun+http://127.0.0.1:9876/proxy`` (推荐)
        * ``host:port`` 形式, 比如 ``127.0.0.1:9876``, 会自动补成完整 URL

    None 时默认 ``rerun+http://127.0.0.1:9876/proxy``。
    """
    import rerun as rr

    raw = addr or os.environ.get("RERUN_CONNECT_ADDR", "127.0.0.1:9876")
    if raw.startswith("rerun+http://") or raw.startswith("rerun+https://"):
        url = raw
    else:
        url = f"rerun+http://{raw}/proxy"
    print(f"[rerun_live] will connect rerun SDK to gRPC at {url}")

    orig_init = rr.init

    def patched_init(application_id, *args, **kwargs):
        kwargs["spawn"] = False
        result = orig_init(application_id, *args, **kwargs)
        if hasattr(rr, "connect_grpc"):
            rr.connect_grpc(url)
        elif hasattr(rr, "connect_tcp"):
            rr.connect_tcp(url)
        elif hasattr(rr, "connect"):
            rr.connect(url)
        else:
            raise RuntimeError("rerun-sdk 没有 connect_grpc / connect_tcp / connect API")
        print(f"[rerun_live] connected to viewer at {url}")
        return result

    rr.init = patched_init


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--official-root", default=str(DEFAULT_OFFICIAL_ROOT))
    p.add_argument(
        "--config",
        default=str(REPO_ROOT / "src" / "KV-tracker" / "config" / "mall_video.yaml"),
    )
    p.add_argument("--addr", default=None,
                   help="rerun viewer address (default: $RERUN_CONNECT_ADDR or 127.0.0.1:9876)")
    p.add_argument("--cam-only", action="store_true",
                   help="跳过 point/conf head; 速度更快但看不到点云")
    p.add_argument("--resize-dim", type=int, default=308)
    p.add_argument("--kf-auto", type=int, default=50)
    p.add_argument("--export-pcd", action="store_true",
                   help="额外把每个关键帧导出为 pcd_*.ply")
    p.add_argument("--sim3", action="store_true")
    args = p.parse_args()

    official_root = Path(args.official_root).resolve()
    config_path = Path(args.config).resolve()

    if not (official_root / "main.py").exists():
        raise SystemExit(f"official main.py not found at {official_root}")
    if not config_path.exists():
        raise SystemExit(f"config not found at {config_path}")

    _ensure_sam2_checkpoint_visible(official_root)
    os.chdir(official_root)
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
    print(f"[rerun_live] cwd     = {official_root}")
    print(f"[rerun_live] sys.argv= {argv}")

    # MUST patch rerun BEFORE main.py imports anything that uses rr.init
    _patch_rerun_live(args.addr)

    def _on_signal(signum, frame):
        print(f"\n[rerun_live] caught signal {signum}, exiting...")
        raise KeyboardInterrupt()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        import main as official_main
        official_main.run_track3r()
    except KeyboardInterrupt:
        print("[rerun_live] interrupted by user")
    except Exception as e:
        print(f"[rerun_live] run_track3r raised: {e}")
        raise
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
