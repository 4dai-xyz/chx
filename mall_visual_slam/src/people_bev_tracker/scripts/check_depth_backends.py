#!/usr/bin/env python3
"""Route A V3.1 阶段 0: 单目深度后端 + 依赖体检 (只读扫描)。

检查:
  * Depth Anything V3 / V2 / ZoeDepth 本地是否已有 (代码目录 + HF 缓存)
  * 官方 ScaRF-SLAM 是否已有
  * dpvo 环境的 torch / cv2 / numpy / open3d
  * CUDA 是否可用

扫描路径:
  /home/ros/ros2_orbslam3
  /home/ros/ros2_orbslam3/project code
  /home/ros/ros2_orbslam3/thirdparty
  /home/ros/miniconda3/envs
  ~/.cache/huggingface

输出:
  output/route_A_v3_scarf/depth_backend_check.json
  output/route_A_v3_scarf/depth_backend_check.md
  (安装报告由调用方另写: reports/00_深度后端体检与安装报告.md)
"""

from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
from pathlib import Path

REPO = Path("/home/ros/ros2_orbslam3")
SCAN_DIRS = [
    REPO,
    REPO / "project code",
    REPO / "thirdparty",
    Path("/home/ros/miniconda3/envs"),
    Path.home() / ".cache" / "huggingface",
]


def _find_dirs(patterns):
    """在 SCAN_DIRS 里找名字匹配 patterns 的目录 (maxdepth ~5)。"""
    hits = []
    for base in SCAN_DIRS:
        if not base.exists():
            continue
        try:
            for p in base.rglob("*"):
                # 限制深度, 避免 rglob 走太深
                try:
                    rel_depth = len(p.relative_to(base).parts)
                except ValueError:
                    continue
                if rel_depth > 5:
                    continue
                if not p.is_dir():
                    continue
                name = p.name.lower()
                for pat in patterns:
                    if pat in name:
                        hits.append(str(p))
                        break
        except (PermissionError, OSError):
            continue
    return sorted(set(hits))


def _module_available(mod: str) -> dict:
    try:
        spec = importlib.util.find_spec(mod)
        return {"available": spec is not None,
                "origin": getattr(spec, "origin", None) if spec else None}
    except (ImportError, ValueError, ModuleNotFoundError):
        return {"available": False, "origin": None}


def _pip_show(pkg: str) -> str | None:
    try:
        out = subprocess.run([sys.executable, "-m", "pip", "show", pkg],
                             capture_output=True, text=True, timeout=30)
        if out.returncode == 0:
            for line in out.stdout.splitlines():
                if line.startswith("Version:"):
                    return line.split(":", 1)[1].strip()
        return None
    except Exception:
        return None


def main() -> int:
    out_dir = REPO / "output" / "route_A_v3_scarf"
    out_dir.mkdir(parents=True, exist_ok=True)

    report = {"scan_dirs": [str(d) for d in SCAN_DIRS]}

    # ---- 核心依赖 ----
    core = {}
    for m in ["torch", "cv2", "numpy", "open3d", "scipy", "PIL", "matplotlib"]:
        core[m] = _module_available(m)
    report["core_modules"] = core

    # torch + cuda
    torch_info = {"available": False}
    try:
        import torch
        torch_info = {
            "available": True,
            "version": torch.__version__,
            "cuda_available": bool(torch.cuda.is_available()),
            "cuda_version": getattr(torch.version, "cuda", None),
            "device_name": (torch.cuda.get_device_name(0)
                            if torch.cuda.is_available() else None),
        }
    except Exception as e:
        torch_info["error"] = str(e)
    report["torch"] = torch_info

    # ---- depth backends: python 包 ----
    py_pkgs = {}
    for mod, pip_name in [
        ("depth_anything_v2", "depth-anything-v2"),
        ("depth_anything", "depth-anything"),
        ("zoedepth", "zoedepth"),
        ("transformers", "transformers"),   # DA V2/V3 常经 HF transformers
        ("timm", "timm"),                   # DA/ZoeDepth backbone 依赖
    ]:
        info = _module_available(mod)
        info["pip_version"] = _pip_show(pip_name)
        py_pkgs[mod] = info
    report["depth_python_packages"] = py_pkgs

    # ---- depth backends: 本地代码目录 ----
    report["local_code_dirs"] = {
        "depth_anything_v3": _find_dirs(["depth-anything-3", "depth_anything_3",
                                          "depth-anything-v3", "depthanything3"]),
        "depth_anything_v2": _find_dirs(["depth-anything-v2", "depth_anything_v2",
                                          "depthanythingv2"]),
        "depth_anything_any": _find_dirs(["depth-anything", "depth_anything"]),
        "zoedepth": _find_dirs(["zoedepth", "zoe-depth", "zoe_depth"]),
        "scarf_slam": _find_dirs(["scarf-slam", "scarf_slam", "scarfslam"]),
    }

    # ---- HF 缓存里的权重 ----
    hf_cache = Path.home() / ".cache" / "huggingface" / "hub"
    hf_hits = []
    if hf_cache.exists():
        for p in hf_cache.iterdir():
            n = p.name.lower()
            if any(k in n for k in ["depth-anything", "depth_anything", "zoedepth",
                                     "zoe", "dpt", "midas"]):
                hf_hits.append(p.name)
    report["hf_cache_models"] = sorted(hf_hits)

    # ---- 结论: 选哪个 backend ----
    def has_transformers_da():
        # transformers 里带 DepthAnything / DPT 支持即可用 pipeline
        t = py_pkgs.get("transformers", {})
        return bool(t.get("available"))

    decision = {"selected_backend": None, "reason": "", "needs_install": True}
    # 优先级: DA V2 > ZoeDepth > DA V3
    if py_pkgs["depth_anything_v2"]["available"] or report["local_code_dirs"]["depth_anything_v2"]:
        decision = {"selected_backend": "depth_anything_v2",
                    "reason": "local Depth Anything V2 code/package found",
                    "needs_install": False}
    elif has_transformers_da():
        decision = {"selected_backend": "depth_anything_v2_hf",
                    "reason": "transformers available; can run Depth-Anything-V2 via HF pipeline",
                    "needs_install": False}
    elif py_pkgs["zoedepth"]["available"] or report["local_code_dirs"]["zoedepth"]:
        decision = {"selected_backend": "zoedepth",
                    "reason": "local ZoeDepth found",
                    "needs_install": False}
    else:
        decision = {"selected_backend": None,
                    "reason": "no local depth backend; install transformers+DA-V2 (HF) recommended",
                    "needs_install": True}
    report["decision"] = decision

    # ---- 写盘 ----
    (out_dir / "depth_backend_check.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    md = _render_md(report)
    (out_dir / "depth_backend_check.md").write_text(md, encoding="utf-8")

    print(json.dumps({
        "torch": torch_info.get("version"),
        "cuda": torch_info.get("cuda_available"),
        "transformers": py_pkgs["transformers"]["available"],
        "timm": py_pkgs["timm"]["available"],
        "open3d": core["open3d"]["available"],
        "selected_backend": decision["selected_backend"],
        "needs_install": decision["needs_install"],
    }, ensure_ascii=False, indent=2))
    return 0


def _render_md(r: dict) -> str:
    d = r["decision"]
    t = r["torch"]
    lines = [
        "# 深度后端体检 (depth_backend_check)",
        "",
        "## 核心依赖",
        f"* torch: {t.get('version')}  (CUDA available = {t.get('cuda_available')}, "
        f"cuda {t.get('cuda_version')}, device = {t.get('device_name')})",
        f"* open3d: {r['core_modules']['open3d']['available']}",
        f"* cv2: {r['core_modules']['cv2']['available']}",
        f"* numpy: {r['core_modules']['numpy']['available']}",
        f"* scipy: {r['core_modules']['scipy']['available']}",
        "",
        "## 深度相关 python 包",
    ]
    for k, v in r["depth_python_packages"].items():
        lines.append(f"* {k}: available={v['available']}  pip_version={v.get('pip_version')}")
    lines += ["", "## 本地代码目录扫描"]
    for k, v in r["local_code_dirs"].items():
        lines.append(f"* {k}: {v if v else '(未找到)'}")
    lines += ["", "## HuggingFace 缓存里的深度模型",
              ("* " + ", ".join(r["hf_cache_models"])) if r["hf_cache_models"] else "* (无)"]
    lines += [
        "",
        "## 结论",
        f"* 选定 backend: **{d['selected_backend']}**",
        f"* 原因: {d['reason']}",
        f"* 是否需要安装: {d['needs_install']}",
    ]
    return "\n".join(lines)


if __name__ == "__main__":
    raise SystemExit(main())
