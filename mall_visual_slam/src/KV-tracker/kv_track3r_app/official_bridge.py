"""把官方 KV-Tracker 代码以只读方式接入。

官方目录默认放在:
    /home/ros/ros2_orbslam3/project code/KV-tracker/kv_tracker-main

只通过 sys.path.insert 引入其包路径，不在官方目录里写任何文件。
SAM2 / Pi3 的源码独立放在 src/KV-tracker/thirdparty/，已 pip install。
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path
from typing import List, Sequence

REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_OFFICIAL_ROOT = REPO_ROOT / "project code" / "KV-tracker" / "kv_tracker-main"
SAM2_CHECKPOINT = (
    REPO_ROOT
    / "src"
    / "KV-tracker"
    / "thirdparty"
    / "segment-anything-2-real-time"
    / "checkpoints"
    / "sam2.1_hiera_small.pt"
)


def add_official_to_syspath(official_root: str | Path | None = None) -> Path:
    """把官方 KV-Tracker 加入 sys.path 并返回根目录。"""
    root = Path(official_root or DEFAULT_OFFICIAL_ROOT).resolve()
    if not (root / "main.py").exists():
        raise FileNotFoundError(f"official KV-Tracker main.py not found: {root}")
    p = str(root)
    if p not in sys.path:
        sys.path.insert(0, p)
    return root


def _ensure_sam2_checkpoint_visible(official_root: Path) -> Path:
    """官方 sam_interface.py 默认用相对路径
    ``thirdparty/segment-anything-2-real-time/checkpoints/sam2.1_hiera_small.pt``。

    官方 thirdparty 是空 submodule, 真权重在 src/KV-tracker/thirdparty/...。
    所以这里把 src 下的权重以**符号链接**方式让官方相对路径解析到。

    注意：这只在官方 ``thirdparty/.../checkpoints`` 目录里建一个 symlink 指向
    src/ 下的真权重。这不是「修改源码」，只是补一个 git submodule 应该有
    但因为离线没拉下来缺失的资源链接。如果不想动官方目录，请用
    ``--explicit-sam-checkpoint`` 直接传绝对路径 (见 run_official_kv_tracker.py)。
    """
    target_link = official_root / "thirdparty" / "segment-anything-2-real-time" / "checkpoints" / "sam2.1_hiera_small.pt"
    if target_link.exists() or target_link.is_symlink():
        return target_link
    target_link.parent.mkdir(parents=True, exist_ok=True)
    if not SAM2_CHECKPOINT.exists():
        raise FileNotFoundError(
            f"SAM2 checkpoint missing at {SAM2_CHECKPOINT}. "
            "Download with the URL in src/KV-tracker/IMPLEMENTATION.md."
        )
    os.symlink(SAM2_CHECKPOINT, target_link)
    return target_link


def run_official_subprocess(
    official_root: str | Path,
    config_path: str | Path,
    extra_args: Sequence[str],
    env: dict | None = None,
) -> int:
    """以 subprocess 启动官方 main.py。

    工作目录设为官方根目录（这样它内部相对路径 ``thirdparty/.../checkpoints/...``
    才能解析）。但不写入任何文件。
    """
    root = Path(official_root).resolve()
    config_path = Path(config_path).resolve()
    if not config_path.exists():
        raise FileNotFoundError(f"config not found: {config_path}")
    _ensure_sam2_checkpoint_visible(root)
    cmd: List[str] = [
        sys.executable,
        "main.py",
        str(config_path),
        *list(extra_args),
    ]
    print(f"[official_bridge] cwd={root}")
    print(f"[official_bridge] cmd={' '.join(cmd)}")
    proc_env = os.environ.copy()
    if env:
        proc_env.update(env)
    # 让官方代码在 import 时能找到我们安装到 src/KV-tracker/thirdparty 的 sam2/pi3
    # （pi3 和 sam-2 实际上已经通过 pip install -e 进入 site-packages, 这里
    # 主要是兜底）
    extra_pp = [str(root)]
    if "PYTHONPATH" in proc_env:
        extra_pp.append(proc_env["PYTHONPATH"])
    proc_env["PYTHONPATH"] = os.pathsep.join(extra_pp)
    rc = subprocess.run(cmd, cwd=str(root), env=proc_env)
    return rc.returncode
