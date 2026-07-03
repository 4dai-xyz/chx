#!/usr/bin/env python3
"""
用 VGGT 官方的 `demo_colmap.py` 导出 COLMAP sparse 结果，并生成 3DGS 的官方训练命令。

这个脚本不重写 VGGT 的几何逻辑，只做工程封装：
1. 读取当前已经抽帧好的 scene_dir/images；
2. 调用官方 `demo_colmap.py` 导出 `sparse/`；
3. 可选走 `--use_ba` 的官方 BA 路线；
4. 生成官方 3DGS 训练命令，方便后续接 `gsplat`。

注意：
官方 BA 路线需要 `pycolmap`，并且会间接依赖官方 tracker / LightGlue 等包。
如果本机还没装好，这个脚本会给出明确提示，而不是悄悄失败。
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path


REPO = Path("/home/ros/ros2_orbslam3")
VGGT_ROOT = REPO / "Opensource code" / "vggt-main" / "vggt-main"
THIRDPARTY_PACKAGES = REPO / "thirdparty" / "python_packages"
DEFAULT_SCENE_DIR = REPO / "output" / "vggt_input_video_show"
DEFAULT_PYTHON = Path("/home/ros/miniconda3/envs/dpvo/bin/python")
DEFAULT_TORCH_HOME = REPO / ".cache" / "torch"
DEFAULT_GSPLAT_RESULT_DIR = DEFAULT_SCENE_DIR / "gsplat_results"


def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(description="Run official VGGT COLMAP + BA + 3DGS post-processing")
    parser.add_argument("--scene-dir", default=str(DEFAULT_SCENE_DIR), help="包含 images/ 的场景目录")
    parser.add_argument("--vggt-root", default=str(VGGT_ROOT), help="VGGT 官方源码目录")
    parser.add_argument("--python", default=str(DEFAULT_PYTHON), help="运行官方脚本所用的 Python")
    parser.add_argument("--torch-home", default=str(DEFAULT_TORCH_HOME), help="VGGT 权重缓存目录")
    parser.add_argument("--use-ba", action="store_true", help="启用官方 track + pycolmap BA 路线")
    parser.add_argument("--seed", type=int, default=42, help="官方 demo_colmap.py 的随机种子")
    parser.add_argument("--max-reproj-error", type=float, default=8.0, help="BA 路线的最大重投影误差")
    parser.add_argument("--shared-camera", action="store_true", help="所有图像共享同一个相机")
    parser.add_argument("--camera-type", default="SIMPLE_PINHOLE", help="COLMAP 相机模型")
    parser.add_argument("--vis-thresh", type=float, default=0.2, help="BA 路线的可见性阈值")
    parser.add_argument("--query-frame-num", type=int, default=8, help="BA 路线查询多少帧")
    parser.add_argument("--max-query-pts", type=int, default=4096, help="BA 路线最大查询点数")
    parser.add_argument("--conf-thres-value", type=float, default=5.0, help="非 BA 路线的深度置信度阈值")
    parser.add_argument("--gsplat-root", default="", help="gsplat 仓库目录，填写后可直接打印官方训练命令")
    parser.add_argument(
        "--gsplat-result-dir",
        default=str(DEFAULT_GSPLAT_RESULT_DIR),
        help="3DGS 训练结果目录，默认放在当前场景目录下",
    )
    parser.add_argument(
        "--image-step",
        type=int,
        default=1,
        help="官方导出时每隔多少张图片取 1 张；大场景在 6GB 显存上建议用 4、5 或 6",
    )
    parser.add_argument(
        "--max-images",
        type=int,
        default=0,
        help="官方导出时最多保留多少张图片；0 表示不限制",
    )
    parser.add_argument("--dry-run", action="store_true", help="只打印命令，不真正执行官方导出")
    return parser.parse_args()


def ensure_scene_dir(scene_dir: Path) -> None:
    """检查场景目录是否具备官方 COLMAP 导出需要的 images/ 结构。"""
    images_dir = scene_dir / "images"
    if not images_dir.exists():
        raise FileNotFoundError(f"找不到 {images_dir}，官方 demo_colmap.py 需要把图片放在 scene_dir/images/")

    image_files = [p for p in images_dir.iterdir() if p.is_file() and not p.name.startswith(".")]
    if not image_files:
        raise RuntimeError(f"{images_dir} 为空，官方 COLMAP 路线没有输入图片")


def stage_official_scene(scene_dir: Path, image_step: int, max_images: int) -> Path:
    """把原始图片按步长/数量抽样到一个独立的官方输入目录里。"""
    image_step = max(1, image_step)
    if image_step == 1 and max_images <= 0:
        return scene_dir

    images_dir = scene_dir / "images"
    staged_scene_dir = scene_dir / f"_official_colmap_input_step{image_step:02d}"
    staged_images_dir = staged_scene_dir / "images"
    staged_images_dir.mkdir(parents=True, exist_ok=True)

    for old_file in staged_images_dir.iterdir():
        if old_file.is_file() or old_file.is_symlink():
            old_file.unlink()

    staged_sparse_dir = staged_scene_dir / "sparse"
    if staged_sparse_dir.exists():
        shutil.rmtree(staged_sparse_dir)

    image_files = sorted(p for p in images_dir.iterdir() if p.is_file() and not p.name.startswith("."))
    selected = image_files[::image_step]
    if max_images > 0:
        selected = selected[:max_images]

    if not selected:
        raise RuntimeError("抽样后没有保留任何图片，请检查 --image-step 或 --max-images")

    for src in selected:
        dst = staged_images_dir / src.name
        if dst.exists() or dst.is_symlink():
            dst.unlink()
        try:
            os.symlink(src.resolve(), dst)
        except OSError:
            shutil.copy2(src, dst)

    print(
        f"已为官方 COLMAP/BA 路线准备抽样场景: {staged_scene_dir} "
        f"(原始图片 {len(image_files)} 张，保留 {len(selected)} 张)"
    )
    return staged_scene_dir


def inject_local_packages() -> None:
    """如果工作区里已经装了本地依赖，就优先从那里导入。"""
    if THIRDPARTY_PACKAGES.exists() and str(THIRDPARTY_PACKAGES) not in sys.path:
        sys.path.insert(0, str(THIRDPARTY_PACKAGES))


def build_demo_colmap_command(args: argparse.Namespace) -> list[str]:
    """拼出官方 demo_colmap.py 的命令。"""
    demo_colmap = Path(args.vggt_root) / "demo_colmap.py"
    if not demo_colmap.exists():
        raise FileNotFoundError(f"找不到官方脚本: {demo_colmap}")

    cmd = [
        args.python,
        str(demo_colmap),
        f"--scene_dir={args.scene_dir}",
        f"--seed={args.seed}",
        f"--camera_type={args.camera_type}",
        f"--vis_thresh={args.vis_thresh}",
        f"--query_frame_num={args.query_frame_num}",
        f"--max_query_pts={args.max_query_pts}",
        f"--conf_thres_value={args.conf_thres_value}",
        f"--max_reproj_error={args.max_reproj_error}",
    ]
    if args.use_ba:
        cmd.append("--use_ba")
    if args.shared_camera:
        cmd.append("--shared_camera")
    return cmd


def check_runtime_deps(use_ba: bool) -> None:
    """在当前 Python 环境里预检查关键依赖，给出更明确的报错。"""
    missing = []
    if importlib.util.find_spec("pycolmap") is None:
        missing.append("pycolmap")
    if use_ba and importlib.util.find_spec("lightglue") is None:
        missing.append("lightglue")
    if use_ba and importlib.util.find_spec("hydra") is None:
        missing.append("hydra-core")
    if use_ba and importlib.util.find_spec("omegaconf") is None:
        missing.append("omegaconf")

    if missing:
        dep_list = ", ".join(missing)
        raise RuntimeError(
            "官方 COLMAP/BA 路线缺少依赖: "
            f"{dep_list}\n"
            "请先安装官方所需包后再运行。\n"
            "参考：\n"
            "  pip install pycolmap pyceres\n"
            "  pip install hydra-core omegaconf\n"
            "  pip install git+https://github.com/jytime/LightGlue.git#egg=lightglue\n"
            "  pip install gsplat==1.3.0"
        )


def write_gsplat_command(scene_dir: Path, gsplat_result_dir: Path, gsplat_root: str) -> Path:
    """写出官方 3DGS 训练命令，方便后续直接复制执行。"""
    command_path = scene_dir / "gsplat_command.sh"
    text_path = scene_dir / "gsplat_command.txt"
    python_packages = REPO / "thirdparty" / "python_packages"
    gsplat_root_path = Path(gsplat_root) if gsplat_root else Path("<GSPLAT仓库路径>")
    examples_dir = gsplat_root_path / "examples"
    pythonpath = f"{gsplat_root_path}:{python_packages}:{examples_dir}:{os.environ.get('PYTHONPATH', '')}".rstrip(":")
    env_line = (
        f'export PYTHONPATH="{pythonpath}"\n'
        f'export MPLCONFIGDIR="/tmp/matplotlib_${{USER}}"\n'
        f'export TORCH_EXTENSIONS_DIR="/tmp/torch_extensions_${{USER}}"\n'
        f'export TORCH_CUDA_ARCH_LIST="${{TORCH_CUDA_ARCH_LIST:-8.9}}"\n'
        f'mkdir -p "$MPLCONFIGDIR"\n'
        f'mkdir -p "$TORCH_EXTENSIONS_DIR"\n'
    )

    if gsplat_root:
        trainer_line = (
            f'cd "{REPO}"\n'
            f'"{DEFAULT_PYTHON}" scripts/run_vggt_gsplat_train.py '
            f'--scene-dir "{scene_dir}" --result-dir "{gsplat_result_dir}" --steps 30000\n'
        )
    else:
        trainer_line = (
            "# 下面是本项目对 VGGT 官方 gsplat 训练器的稳定封装命令。\n"
            "# 它内部仍然调用 thirdparty/gsplat/examples/simple_trainer.py。\n"
            f'cd "{REPO}"\n'
            f'"{DEFAULT_PYTHON}" scripts/run_vggt_gsplat_train.py '
            f'--scene-dir "{scene_dir}" --result-dir "{gsplat_result_dir}" --steps 30000\n'
        )

    command_path.write_text(
        "#!/usr/bin/env bash\n"
        "set -e\n"
        f"{env_line}"
        f"{trainer_line}",
        encoding="utf-8",
    )
    os.chmod(command_path, 0o755)

    text_path.write_text(
        "VGGT 官方 3DGS 训练命令\n"
        "================================\n\n"
        f"{env_line}"
        f"{trainer_line}\n"
        "说明：VGGT 官方 README 推荐把导出的 COLMAP 结果直接接到 gsplat。\n"
        "本项目的 run_vggt_gsplat_train.py 只是稳定封装，核心训练仍调用官方 simple_trainer.py。\n"
        "如果远程终端容易断开，建议使用 scripts/run_vggt_gsplat_background.sh start 后台运行。\n",
        encoding="utf-8",
    )
    return command_path


def mirror_sparse_to_scene_dir(official_scene_dir: Path, scene_dir: Path) -> Path:
    """把官方导出的 sparse 结果镜像回主结果目录，方便后续查看和训练。"""
    source_sparse = official_scene_dir / "sparse"
    target_sparse = scene_dir / "sparse"
    if not source_sparse.exists():
        raise FileNotFoundError(f"找不到官方 sparse 结果: {source_sparse}")

    if target_sparse.exists():
        shutil.rmtree(target_sparse)
    shutil.copytree(source_sparse, target_sparse)
    return target_sparse


def write_official_summary(scene_dir: Path, args: argparse.Namespace, command: list[str], official_scene_dir: Path) -> Path:
    """记录这次官方后处理的参数，便于文档脚本读取。"""
    sparse_dir = scene_dir / "sparse"
    summary = {
        "scene_dir": str(scene_dir),
        "official_scene_dir": str(official_scene_dir),
        "official_sparse_dir": str(official_scene_dir / "sparse"),
        "sparse_dir": str(sparse_dir),
        "use_ba": args.use_ba,
        "camera_type": args.camera_type,
        "seed": args.seed,
        "max_reproj_error": args.max_reproj_error,
        "shared_camera": args.shared_camera,
        "vis_thresh": args.vis_thresh,
        "query_frame_num": args.query_frame_num,
        "max_query_pts": args.max_query_pts,
        "conf_thres_value": args.conf_thres_value,
        "image_step": args.image_step,
        "max_images": args.max_images,
        "demo_colmap_command": command,
    }
    summary_path = scene_dir / "official_colmap_summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    return summary_path


def main() -> None:
    """脚本入口。"""
    args = parse_args()
    scene_dir = Path(args.scene_dir)

    inject_local_packages()
    ensure_scene_dir(scene_dir)

    official_scene_dir = stage_official_scene(scene_dir, args.image_step, args.max_images)

    env = os.environ.copy()
    env["TORCH_HOME"] = args.torch_home
    env["PYTHONUNBUFFERED"] = "1"
    pythonpath_parts = [str(Path(args.vggt_root))]
    if THIRDPARTY_PACKAGES.exists():
        pythonpath_parts.insert(0, str(THIRDPARTY_PACKAGES))
    if env.get("PYTHONPATH"):
        pythonpath_parts.append(env["PYTHONPATH"])
    env["PYTHONPATH"] = os.pathsep.join(pythonpath_parts)

    command_args = argparse.Namespace(**{**vars(args), "scene_dir": str(official_scene_dir)})
    command = build_demo_colmap_command(command_args)
    print("官方 COLMAP/BA 命令：")
    print(" ".join(command))

    write_official_summary(scene_dir, args, command, official_scene_dir)
    gsplat_result_dir = Path(args.gsplat_result_dir)
    command_path = write_gsplat_command(scene_dir, gsplat_result_dir, args.gsplat_root)
    print(f"已写出 3DGS 官方命令文件: {command_path}")
    print(f"3DGS 结果目录建议: {gsplat_result_dir}")

    if args.dry_run:
        print("当前是 dry-run，只生成命令和摘要，不执行官方脚本。")
        return

    check_runtime_deps(args.use_ba)

    subprocess.run(command, check=True, env=env)
    sparse_dir = mirror_sparse_to_scene_dir(official_scene_dir, scene_dir)
    print(f"官方 COLMAP 结果已写入: {sparse_dir}")
    print(f"后续可直接读取: {sparse_dir / 'cameras.bin'}")
    print(f"后续可直接读取: {sparse_dir / 'images.bin'}")
    print(f"后续可直接读取: {sparse_dir / 'points3D.bin'}")


if __name__ == "__main__":
    main()
