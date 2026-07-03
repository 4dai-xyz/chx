#!/usr/bin/env python3
"""
用 VGGT 导出的 COLMAP 结果启动官方 gsplat / 3DGS 训练。

这个脚本是一个稳定封装层：训练器仍然使用 thirdparty/gsplat/examples/simple_trainer.py，
这里只负责补齐本项目的 PYTHONPATH、CUDA 扩展缓存目录和常用训练参数。
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


REPO = Path("/home/ros/ros2_orbslam3")
GSPLAT_ROOT = REPO / "thirdparty" / "gsplat"
GSPLAT_EXAMPLES = GSPLAT_ROOT / "examples"
THIRDPARTY_PACKAGES = REPO / "thirdparty" / "python_packages"
LOCAL_CCCL_SHIM_INCLUDE = REPO / "thirdparty" / "cuda_cccl_shim" / "include"
CUDA_121_LIBCXX_INCLUDE = Path(
    "/usr/local/cuda-12.1/targets/x86_64-linux/include/cuda/std/detail/libcxx/include"
)
DEFAULT_SCENE_DIR = REPO / "output" / "vggt_input_video_show"
DEFAULT_RESULT_DIR = DEFAULT_SCENE_DIR / "gsplat_results"


def parse_int_list(text: str, default: list[int]) -> list[int]:
    """把 1,2,3 形式的参数解析成整数列表。"""
    if not text:
        return default
    values = []
    for item in text.split(","):
        item = item.strip()
        if item:
            values.append(int(item))
    return values


def parse_args() -> argparse.Namespace:
    """解析训练参数。"""
    parser = argparse.ArgumentParser(description="Run official gsplat trainer on VGGT COLMAP output")
    parser.add_argument("--scene-dir", default=str(DEFAULT_SCENE_DIR), help="包含 images/ 和 sparse/ 的场景目录")
    parser.add_argument("--result-dir", default=str(DEFAULT_RESULT_DIR), help="3DGS 训练输出目录")
    parser.add_argument("--steps", type=int, default=30_000, help="训练步数；冒烟测试建议 3 或 10")
    parser.add_argument("--eval-steps", default="", help="评估步数，逗号分隔；为空表示不做中途评估")
    parser.add_argument("--save-steps", default="", help="保存 checkpoint 的步数，逗号分隔；为空表示最后一步保存")
    parser.add_argument("--ply-steps", default="", help="导出 PLY 的步数，逗号分隔；为空表示最后一步导出")
    parser.add_argument("--data-factor", type=int, default=1, help="图片下采样倍率；1 表示使用原图")
    parser.add_argument("--test-every", type=int, default=8, help="每隔多少张图作为验证集")
    parser.add_argument("--init-type", choices=["sfm", "random"], default="sfm", help="高斯初始化方式")
    parser.add_argument("--init-num-pts", type=int, default=100_000, help="random 初始化时的高斯数量")
    parser.add_argument("--no-save-ply", action="store_true", help="不在训练结束时导出 PLY")
    parser.add_argument("--tb-every", type=int, default=100, help="TensorBoard 写入间隔；0 表示关闭")
    return parser.parse_args()


def prepare_runtime() -> None:
    """准备导入路径和 CUDA 扩展缓存目录。"""
    ordered_paths = [str(GSPLAT_ROOT), str(THIRDPARTY_PACKAGES), str(GSPLAT_EXAMPLES)]
    sys.path[:] = ordered_paths + [path for path in sys.path if path not in ordered_paths]

    os.environ.setdefault(
        "PYTHONPATH",
        os.pathsep.join(ordered_paths),
    )
    os.environ.setdefault("MPLCONFIGDIR", f"/tmp/matplotlib_{os.environ.get('USER', 'ros')}")
    os.environ.setdefault("TORCH_EXTENSIONS_DIR", f"/tmp/torch_extensions_{os.environ.get('USER', 'ros')}_cccl129")
    os.environ.setdefault("TORCH_CUDA_ARCH_LIST", "8.9")
    os.environ.setdefault("MAX_JOBS", "1")

    # 当前项目只需要标准 3DGS 训练链路，不需要 3DGUT / 2DGS / relocation。
    # 这样可以直接绕开 gsplat 中依赖 cuda/std/optional 的 3DGUT 编译分支，
    # 避免在 CUDA 12.1 环境里继续碰到头文件兼容问题。
    os.environ["BUILD_3DGS"] = "1"
    os.environ["BUILD_2DGS"] = "0"
    os.environ["BUILD_3DGUT"] = "0"
    os.environ["BUILD_ADAM"] = "0"
    os.environ["BUILD_RELOC"] = "0"
    os.environ["BUILD_CAMERA_WRAPPERS"] = "0"

    if LOCAL_CCCL_SHIM_INCLUDE.exists():
        cccl_paths = [str(LOCAL_CCCL_SHIM_INCLUDE)]
        os.environ.setdefault("GSPLAT_EXTRA_INCLUDE_PATHS", os.pathsep.join(cccl_paths))
        for env_name in ("CPATH", "CPLUS_INCLUDE_PATH", "C_INCLUDE_PATH"):
            old_value = os.environ.get(env_name, "")
            paths = cccl_paths + [path for path in old_value.split(os.pathsep) if path]
            os.environ[env_name] = os.pathsep.join(paths)
    if CUDA_121_LIBCXX_INCLUDE.exists():
        os.environ.setdefault("GSPLAT_EXTRA_IDIRAFTER_PATHS", str(CUDA_121_LIBCXX_INCLUDE))

    Path(os.environ["MPLCONFIGDIR"]).mkdir(parents=True, exist_ok=True)
    Path(os.environ["TORCH_EXTENSIONS_DIR"]).mkdir(parents=True, exist_ok=True)


def print_gpu_status() -> None:
    """打印 PyTorch 看到的 GPU 状态，方便从日志判断是否真的用了显卡。"""
    import torch

    print(f"PyTorch: {torch.__version__}, CUDA build: {torch.version.cuda}", flush=True)
    print(f"torch.cuda.is_available(): {torch.cuda.is_available()}", flush=True)
    if not torch.cuda.is_available():
        raise RuntimeError("当前进程看不到 CUDA，不能运行 3DGS 训练。")
    print(f"GPU: {torch.cuda.get_device_name(0)}", flush=True)
    print(f"MAX_JOBS: {os.environ.get('MAX_JOBS')}", flush=True)
    print(f"GSPLAT_EXTRA_INCLUDE_PATHS: {os.environ.get('GSPLAT_EXTRA_INCLUDE_PATHS', '')}", flush=True)
    print(f"GSPLAT_EXTRA_IDIRAFTER_PATHS: {os.environ.get('GSPLAT_EXTRA_IDIRAFTER_PATHS', '')}", flush=True)
    build_flags = {
        name: os.environ.get(name, "")
        for name in ("BUILD_3DGS", "BUILD_2DGS", "BUILD_3DGUT", "BUILD_ADAM", "BUILD_RELOC", "BUILD_CAMERA_WRAPPERS")
    }
    print(f"gsplat 编译模块: {build_flags}", flush=True)


def main() -> None:
    """脚本入口。"""
    args = parse_args()
    prepare_runtime()
    os.chdir(GSPLAT_EXAMPLES)

    print_gpu_status()

    from simple_trainer import Config, DefaultStrategy, main as gsplat_main

    scene_dir = Path(args.scene_dir)
    result_dir = Path(args.result_dir)
    if not (scene_dir / "images").exists():
        raise FileNotFoundError(f"找不到 images 目录: {scene_dir / 'images'}")
    if not (scene_dir / "sparse").exists():
        raise FileNotFoundError(f"找不到 sparse 目录: {scene_dir / 'sparse'}")

    save_steps = parse_int_list(args.save_steps, [args.steps])
    ply_steps = parse_int_list(args.ply_steps, [args.steps])
    eval_steps = parse_int_list(args.eval_steps, [])

    cfg = Config(
        disable_viewer=True,
        data_dir=str(scene_dir),
        data_factor=args.data_factor,
        result_dir=str(result_dir),
        test_every=args.test_every,
        max_steps=args.steps,
        eval_steps=eval_steps,
        save_steps=save_steps,
        save_ply=not args.no_save_ply,
        ply_steps=ply_steps,
        init_type=args.init_type,
        init_num_pts=args.init_num_pts,
        tb_every=args.tb_every,
        load_exposure=False,
        batch_size=1,
        strategy=DefaultStrategy(verbose=True),
    )

    print(f"场景目录: {scene_dir}", flush=True)
    print(f"输出目录: {result_dir}", flush=True)
    print(f"训练步数: {args.steps}", flush=True)
    print(f"初始化方式: {args.init_type}", flush=True)
    gsplat_main(local_rank=0, world_rank=0, world_size=1, cfg=cfg)
    print("gsplat 训练结束。", flush=True)


if __name__ == "__main__":
    main()
