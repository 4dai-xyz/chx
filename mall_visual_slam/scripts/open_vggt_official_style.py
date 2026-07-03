#!/usr/bin/env python3
"""
用 VGGT 官方 demo 的 Viser 构图方式，直接打开已有 predictions.npz。

这个脚本不重新跑模型，只读取已经保存好的预测结果，然后调用官方
demo_viser.py 里的 viser_wrapper，把点云、相机和视锥放到同一个 3D 场景里。
这样最接近官方示例网站的视觉风格。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np


REPO = Path("/home/ros/ros2_orbslam3")
VGGT_ROOT = REPO / "Opensource code" / "vggt-main" / "vggt-main"
DEFAULT_PREDICTIONS = REPO / "output" / "vggt_scene_full" / "predictions.npz"


def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(description="Open VGGT predictions with the official Viser style")
    parser.add_argument("--predictions", default=str(DEFAULT_PREDICTIONS), help="VGGT 预测文件 predictions.npz")
    parser.add_argument("--port", type=int, default=8097, help="Viser 服务端口")
    parser.add_argument("--conf-threshold", type=float, default=25.0, help="置信度百分位过滤阈值")
    parser.add_argument("--use-point-map", action="store_true", help="使用 point map 分支而不是深度反投影")
    parser.add_argument("--mask-sky", action="store_true", help="可选：启用 sky segmentation")
    parser.add_argument("--image-folder", default="", help="可选：原图文件夹，用于 sky segmentation")
    return parser.parse_args()


def prepare_imports() -> None:
    """让 Python 能导入官方 VGGT 源码。"""
    if str(VGGT_ROOT) not in sys.path:
        sys.path.insert(0, str(VGGT_ROOT))


def load_predictions(path: Path) -> dict:
    """读取已有的 VGGT 预测结果。"""
    if not path.exists():
        raise FileNotFoundError(f"找不到预测文件: {path}")

    with np.load(path, allow_pickle=False) as data:
        return {key: data[key] for key in data.files}


def main() -> None:
    """脚本入口。"""
    args = parse_args()
    prepare_imports()

    from demo_viser import viser_wrapper

    predictions = load_predictions(Path(args.predictions))
    if "world_points" not in predictions and "world_points_from_depth" in predictions:
        predictions["world_points"] = predictions["world_points_from_depth"]
    if "world_points_conf" not in predictions and "depth_conf" in predictions:
        predictions["world_points_conf"] = predictions["depth_conf"]
    summary_path = Path(args.predictions).with_name("summary.json")
    if summary_path.exists():
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        print(f"读取到结果摘要: {summary_path}")
        print(f"处理模式: {summary.get('mode', 'unknown')}")

    if args.image_folder:
        image_folder = args.image_folder
    else:
        image_folder = str(Path(args.predictions).with_name("images"))

    print("使用官方 demo_viser 构图方式打开已有预测结果...")
    viser_wrapper(
        predictions,
        port=args.port,
        init_conf_threshold=args.conf_threshold,
        use_point_map=args.use_point_map,
        background_mode=True,
        mask_sky=args.mask_sky,
        image_folder=image_folder if image_folder else None,
    )

    print(f"官方风格 3D viewer 已启动。请在浏览器打开: http://localhost:{args.port}")
    print("按 Ctrl+C 退出。")
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("viewer 已退出。")


if __name__ == "__main__":
    main()
