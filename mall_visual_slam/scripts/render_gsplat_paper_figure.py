#!/usr/bin/env python3
"""
用训练好的 3DGS 高斯检查点渲染论文 Fig.1 风格的场景图。

流程：
1. 加载 gsplat 训练保存的 ckpt（包含 means/quats/scales/opacities/sh0/shN）;
2. 读取 COLMAP sparse 里的相机位姿和内参；
3. 沿训练相机轨迹渲染若干帧（包括首帧、尾帧、和几个中间帧）；
4. 把渲染结果和对应的 GT 原图拼成一张「真实场景 vs 3DGS 重建」对比图；
5. 再渲染一张 trajectory 上方的 hero 视角，作为单图大图输出。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Sequence

import numpy as np
import torch


REPO = Path(__file__).resolve().parent.parent
THIRDPARTY = REPO / "thirdparty" / "python_packages"
GSPLAT_ROOT = REPO / "thirdparty" / "gsplat"
LOCAL_CCCL_SHIM_INCLUDE = REPO / "thirdparty" / "cuda_cccl_shim" / "include"
CUDA_121_LIBCXX_INCLUDE = Path(
    "/usr/local/cuda-12.1/targets/x86_64-linux/include/cuda/std/detail/libcxx/include"
)


def setup_imports() -> None:
    """镜像 run_vggt_gsplat_train.py 的环境，否则 gsplat 会去编译 3DGUT 分支
    （需要 cuda/std/optional），在 CUDA 12.1 上无法成功。"""
    for p in (str(THIRDPARTY), str(GSPLAT_ROOT), str(GSPLAT_ROOT / "examples")):
        if p not in sys.path:
            sys.path.insert(0, p)

    # 只编译 3DGS 分支，跳过 2DGS / 3DGUT / Adam / Reloc / camera wrappers。
    os.environ["BUILD_3DGS"] = "1"
    os.environ["BUILD_2DGS"] = "0"
    os.environ["BUILD_3DGUT"] = "0"
    os.environ["BUILD_ADAM"] = "0"
    os.environ["BUILD_RELOC"] = "0"
    os.environ["BUILD_CAMERA_WRAPPERS"] = "0"
    os.environ.setdefault("TORCH_CUDA_ARCH_LIST", "8.9")
    os.environ.setdefault("MAX_JOBS", "1")
    os.environ.setdefault("MPLCONFIGDIR", f"/tmp/matplotlib_{os.environ.get('USER', 'ros')}")
    os.environ.setdefault(
        "TORCH_EXTENSIONS_DIR",
        f"/tmp/torch_extensions_{os.environ.get('USER', 'ros')}_cccl129",
    )

    if LOCAL_CCCL_SHIM_INCLUDE.exists():
        cccl_paths = [str(LOCAL_CCCL_SHIM_INCLUDE)]
        os.environ.setdefault("GSPLAT_EXTRA_INCLUDE_PATHS", os.pathsep.join(cccl_paths))
        for env_name in ("CPATH", "CPLUS_INCLUDE_PATH", "C_INCLUDE_PATH"):
            old_value = os.environ.get(env_name, "")
            paths = cccl_paths + [p for p in old_value.split(os.pathsep) if p]
            os.environ[env_name] = os.pathsep.join(paths)
    if CUDA_121_LIBCXX_INCLUDE.exists():
        os.environ.setdefault("GSPLAT_EXTRA_IDIRAFTER_PATHS", str(CUDA_121_LIBCXX_INCLUDE))

    Path(os.environ["MPLCONFIGDIR"]).mkdir(parents=True, exist_ok=True)
    Path(os.environ["TORCH_EXTENSIONS_DIR"]).mkdir(parents=True, exist_ok=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scene-dir",
        default=str(REPO / "output" / "vggt_aligned_full_run" / "aligned_colmap"),
        help="带 sparse/ 和 images/ 的目录（build_colmap_from_aligned.py 输出）",
    )
    parser.add_argument(
        "--ckpt",
        default="",
        help="3DGS ckpt 路径（默认从 result-dir/ckpts/ 自动找最大步数的）",
    )
    parser.add_argument(
        "--result-dir",
        default=str(REPO / "output" / "vggt_aligned_full_run" / "gsplat_results"),
        help="gsplat 训练输出目录（用来找 ckpt 和写渲染结果）",
    )
    parser.add_argument(
        "--out-dir",
        default="",
        help="渲染输出目录；默认 result-dir/paper_renders",
    )
    parser.add_argument(
        "--render-factor",
        type=int,
        default=2,
        help="渲染分辨率下采样倍率，匹配训练时的 --data-factor",
    )
    parser.add_argument(
        "--num-comparison",
        type=int,
        default=6,
        help="对比图里展示多少帧（均匀采样训练相机）",
    )
    parser.add_argument(
        "--num-trajectory-renders",
        type=int,
        default=12,
        help="沿轨迹渲染多少帧拼大图",
    )
    parser.add_argument(
        "--bg",
        choices=["white", "black"],
        default="white",
        help="3DGS 背景色",
    )
    return parser.parse_args()


def _find_latest_ckpt(result_dir: Path) -> Path:
    ckpt_dir = result_dir / "ckpts"
    if not ckpt_dir.exists():
        raise FileNotFoundError(f"找不到 ckpt 目录: {ckpt_dir}")
    cands = sorted(ckpt_dir.glob("ckpt_*_rank0.pt"))
    if not cands:
        raise FileNotFoundError(f"{ckpt_dir} 里没有 ckpt 文件")
    return cands[-1]


def _load_gaussians_from_ckpt(ckpt_path: Path, device: torch.device) -> dict:
    """从 gsplat simple_trainer 保存的 ckpt 读高斯参数。"""
    ckpt = torch.load(str(ckpt_path), map_location="cpu", weights_only=False)
    # simple_trainer 的 ckpt 结构：{'splats': {'means':..., 'quats':..., ...}, ...}
    if "splats" in ckpt:
        splats = ckpt["splats"]
    elif "gaussians" in ckpt:
        splats = ckpt["gaussians"]
    else:
        splats = ckpt
    keys_needed = ("means", "quats", "scales", "opacities", "sh0", "shN")
    out = {}
    for k in keys_needed:
        if k in splats:
            v = splats[k]
            if isinstance(v, torch.Tensor):
                out[k] = v.detach().to(device)
            else:
                out[k] = torch.as_tensor(v, device=device)
    if "means" not in out:
        raise KeyError(f"ckpt 缺关键字 means: {ckpt_path}")
    return out


def _load_colmap(scene_dir: Path) -> tuple[list[dict], int, int, np.ndarray]:
    """读 COLMAP sparse/0/，返回每张图的元数据 + 图像尺寸 + K。"""
    import pycolmap as pc

    sparse_dir = scene_dir / "sparse" / "0"
    rec = pc.Reconstruction(str(sparse_dir))
    if len(rec.cameras) == 0 or len(rec.images) == 0:
        raise RuntimeError(f"COLMAP sparse 为空: {sparse_dir}")

    cam = next(iter(rec.cameras.values()))
    W, H = cam.width, cam.height
    K = np.asarray(cam.calibration_matrix(), dtype=np.float64)

    entries: list[dict] = []
    for image_id in sorted(rec.images.keys()):
        img = rec.images[image_id]
        w2c = np.eye(4, dtype=np.float64)
        w2c[:3, :4] = np.asarray(img.cam_from_world.matrix(), dtype=np.float64)
        entries.append({
            "image_id": int(image_id),
            "name": str(img.name),
            "w2c": w2c,
        })
    return entries, int(W), int(H), K


def _render_one(
    splats: dict,
    K: torch.Tensor,
    viewmat: torch.Tensor,
    width: int,
    height: int,
    background: tuple[float, float, float],
) -> np.ndarray:
    """渲染一张图。viewmat 是 4x4 world-to-cam。返回 (H, W, 3) uint8。"""
    from gsplat.rendering import rasterization

    means = splats["means"]
    quats = splats["quats"]
    scales = torch.exp(splats["scales"])
    opacities = torch.sigmoid(splats["opacities"]).squeeze(-1)
    # 把 sh0 (N,1,3) 和 shN (N,K,3) 拼起来，rasterization 期望 (N, (K+1), 3)。
    sh0 = splats["sh0"]
    shN = splats["shN"]
    colors = torch.cat([sh0, shN], dim=1)
    sh_degree = int(np.sqrt(colors.shape[1])) - 1

    # gsplat 默认 packed=True，那条路径下 backgrounds 期望形状是 (channels,)。
    # 这里强制 packed=False 走「(C, N, 2)」的稠密分支，backgrounds 用 (C, 3)。
    bg_tensor = torch.tensor([list(background)], dtype=torch.float32, device=means.device)
    renders, _alphas, _info = rasterization(
        means=means,
        quats=quats,
        scales=scales,
        opacities=opacities,
        colors=colors,
        viewmats=viewmat[None],
        Ks=K[None],
        width=width,
        height=height,
        sh_degree=sh_degree,
        backgrounds=bg_tensor,
        render_mode="RGB",
        packed=False,
    )
    img = renders[0].clamp(0, 1).detach().cpu().numpy()
    return (img * 255.0).astype(np.uint8)


def _load_gt_image(path: Path, target_w: int, target_h: int) -> np.ndarray:
    import cv2

    bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if bgr is None:
        return np.full((target_h, target_w, 3), 255, dtype=np.uint8)
    if bgr.shape[1] != target_w or bgr.shape[0] != target_h:
        bgr = cv2.resize(bgr, (target_w, target_h), interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def main() -> None:
    setup_imports()
    args = parse_args()

    scene_dir = Path(args.scene_dir)
    result_dir = Path(args.result_dir)
    out_dir = Path(args.out_dir) if args.out_dir else result_dir / "paper_renders"
    out_dir.mkdir(parents=True, exist_ok=True)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"渲染设备: {device}")

    ckpt_path = Path(args.ckpt) if args.ckpt else _find_latest_ckpt(result_dir)
    print(f"加载 ckpt: {ckpt_path}")
    splats = _load_gaussians_from_ckpt(ckpt_path, device)
    print(f"  Gaussians: {splats['means'].shape[0]}")

    entries, W, H, K_orig = _load_colmap(scene_dir)
    print(f"COLMAP 相机: {len(entries)} 张，原图 {W}x{H}")

    # 用渲染下采样倍率得到目标分辨率，并同步缩放 K。
    factor = max(1, int(args.render_factor))
    rW, rH = W // factor, H // factor
    K_render = K_orig.copy()
    K_render[0, :] /= factor
    K_render[1, :] /= factor
    print(f"渲染分辨率: {rW}x{rH}")

    K_t = torch.as_tensor(K_render, dtype=torch.float32, device=device)
    bg = (1.0, 1.0, 1.0) if args.bg == "white" else (0.0, 0.0, 0.0)

    # 1) 对比图：均匀挑 N 帧训练相机，左为原图 GT，右为同视角渲染。
    n_cmp = max(1, args.num_comparison)
    cmp_indices = np.linspace(0, len(entries) - 1, n_cmp, dtype=np.int64)

    import matplotlib
    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    cmp_dir = out_dir / "comparisons"
    cmp_dir.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(n_cmp, 2, figsize=(11, 3.0 * n_cmp), facecolor="white")
    if n_cmp == 1:
        axes = np.array([axes])
    for row, idx in enumerate(cmp_indices):
        entry = entries[int(idx)]
        gt_path = scene_dir / "images" / entry["name"]
        gt_img = _load_gt_image(gt_path, rW, rH)
        viewmat = torch.as_tensor(entry["w2c"], dtype=torch.float32, device=device)
        rendered = _render_one(splats, K_t, viewmat, rW, rH, bg)

        axes[row, 0].imshow(gt_img)
        axes[row, 0].set_title(f"GT: {entry['name']}", fontsize=9)
        axes[row, 0].axis("off")
        axes[row, 1].imshow(rendered)
        axes[row, 1].set_title(f"3DGS render", fontsize=9)
        axes[row, 1].axis("off")

        # 也单张保存便于挑图。
        import cv2
        cv2.imwrite(str(cmp_dir / f"gt_{int(idx):03d}.png"), cv2.cvtColor(gt_img, cv2.COLOR_RGB2BGR))
        cv2.imwrite(str(cmp_dir / f"render_{int(idx):03d}.png"), cv2.cvtColor(rendered, cv2.COLOR_RGB2BGR))

    fig.tight_layout()
    cmp_path = out_dir / "gt_vs_render_comparison.png"
    fig.savefig(cmp_path, dpi=160, facecolor="white", bbox_inches="tight")
    plt.close(fig)
    print(f"保存对比图: {cmp_path}")

    # 2) 沿轨迹渲染若干帧拼成大图（4xN 网格），作为「整段视频 3DGS 重建结果」。
    n_traj = max(1, args.num_trajectory_renders)
    traj_indices = np.linspace(0, len(entries) - 1, n_traj, dtype=np.int64)

    cols = 4
    rows = (n_traj + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(cols * 3.2, rows * 2.0), facecolor="white")
    axes = np.atleast_2d(axes)
    for k, idx in enumerate(traj_indices):
        entry = entries[int(idx)]
        viewmat = torch.as_tensor(entry["w2c"], dtype=torch.float32, device=device)
        rendered = _render_one(splats, K_t, viewmat, rW, rH, bg)
        r, c = divmod(k, cols)
        axes[r, c].imshow(rendered)
        axes[r, c].set_title(f"frame {idx}", fontsize=8)
        axes[r, c].axis("off")
    for k in range(n_traj, rows * cols):
        r, c = divmod(k, cols)
        axes[r, c].axis("off")
    fig.suptitle("3DGS renders along the camera trajectory", fontsize=12)
    fig.tight_layout()
    grid_path = out_dir / "trajectory_renders_grid.png"
    fig.savefig(grid_path, dpi=160, facecolor="white", bbox_inches="tight")
    plt.close(fig)
    print(f"保存轨迹网格图: {grid_path}")

    # 3) 单张 hero：从场景外部俯视的「论文 Fig.1」视角，保证整段场景在视野里。
    # 训练后 gaussian 集中分布在 means 的包围盒里，相机轨迹是其中一条曲线。
    # 把相机放到包围盒上方对角，朝向场景中心，渲染一张干净的外部视角。
    means_np = splats["means"].detach().cpu().numpy()
    bb_min = means_np.min(axis=0)
    bb_max = means_np.max(axis=0)
    bb_center = (bb_min + bb_max) / 2.0
    bb_extent = float(np.linalg.norm(bb_max - bb_min))

    # 在 +X/+Y/+Z 对角方向上离开场景中心 1.2 * extent 的位置。
    offset_dir = np.array([0.9, -0.9, 1.0], dtype=np.float64)
    offset_dir /= np.linalg.norm(offset_dir)
    cam_pos = bb_center + offset_dir * (bb_extent * 0.75)
    # look_at 方向：从相机指向场景中心。
    look_dir = bb_center - cam_pos
    look_dir /= np.linalg.norm(look_dir)
    # 构造一个稳定的 up 向量，避免和 look 共线。
    world_up = np.array([0.0, -1.0, 0.0], dtype=np.float64)
    if abs(np.dot(world_up, look_dir)) > 0.95:
        world_up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    right = np.cross(look_dir, world_up)
    right /= np.linalg.norm(right)
    up = np.cross(right, look_dir)
    # OpenCV 相机系：x_right, y_down, z_forward。R_cam_to_world = [right | -up | look]
    R_c2w = np.stack([right, -up, look_dir], axis=1)
    t_c2w = cam_pos
    R_w2c = R_c2w.T
    t_w2c = -R_w2c @ t_c2w
    hero_w2c = np.eye(4, dtype=np.float64)
    hero_w2c[:3, :3] = R_w2c
    hero_w2c[:3, 3] = t_w2c
    hero_viewmat = torch.as_tensor(hero_w2c, dtype=torch.float32, device=device)

    # hero 视角用较小焦距以扩大视野，cx/cy 在画面中心。
    hero_W, hero_H = 1600, 1200
    hero_focal = 0.85 * hero_W
    hero_K = torch.tensor(
        [[hero_focal, 0.0, hero_W / 2.0], [0.0, hero_focal, hero_H / 2.0], [0.0, 0.0, 1.0]],
        dtype=torch.float32, device=device,
    )
    try:
        hero_render = _render_one(splats, hero_K, hero_viewmat, hero_W, hero_H, bg)
    except torch.cuda.OutOfMemoryError:
        hero_W, hero_H = 1200, 900
        hero_focal = 0.85 * hero_W
        hero_K = torch.tensor(
            [[hero_focal, 0.0, hero_W / 2.0], [0.0, hero_focal, hero_H / 2.0], [0.0, 0.0, 1.0]],
            dtype=torch.float32, device=device,
        )
        hero_render = _render_one(splats, hero_K, hero_viewmat, hero_W, hero_H, bg)

    import cv2
    hero_path = out_dir / "hero_render_external.png"
    cv2.imwrite(str(hero_path), cv2.cvtColor(hero_render, cv2.COLOR_RGB2BGR))
    print(f"保存外部 hero 大图: {hero_path}")

    # 4) 再渲染 4 张多角度外部视角拼成一张图（俯视/正视/侧视/对角）。
    external_views = []
    spec_list = [
        ("Top-down", np.array([0.0, -1.0, 0.0]), np.array([0.0, 0.0, 1.0])),
        ("Front",    np.array([0.0, 0.0, -1.0]), np.array([0.0, -1.0, 0.0])),
        ("Side",     np.array([-1.0, 0.0, 0.0]), np.array([0.0, -1.0, 0.0])),
        ("Diagonal", np.array([0.9, -0.9, 1.0]), np.array([0.0, -1.0, 0.0])),
    ]
    multi_W, multi_H = 900, 700
    multi_focal = 0.85 * multi_W
    multi_K = torch.tensor(
        [[multi_focal, 0.0, multi_W / 2.0], [0.0, multi_focal, multi_H / 2.0], [0.0, 0.0, 1.0]],
        dtype=torch.float32, device=device,
    )
    for title, offdir, world_up_pref in spec_list:
        offdir = offdir / np.linalg.norm(offdir)
        cp = bb_center - offdir * (bb_extent * 0.75)
        look = bb_center - cp
        look /= np.linalg.norm(look)
        # 选 up：跟 look 不共线的世界轴。
        wu = world_up_pref.astype(np.float64)
        if abs(np.dot(wu, look)) > 0.95:
            wu = np.array([0.0, 0.0, 1.0])
            if abs(np.dot(wu, look)) > 0.95:
                wu = np.array([1.0, 0.0, 0.0])
        right_v = np.cross(look, wu); right_v /= np.linalg.norm(right_v)
        up_v = np.cross(right_v, look)
        R_c2w_v = np.stack([right_v, -up_v, look], axis=1)
        R_w2c_v = R_c2w_v.T
        t_w2c_v = -R_w2c_v @ cp
        vw = np.eye(4); vw[:3,:3] = R_w2c_v; vw[:3,3] = t_w2c_v
        vm = torch.as_tensor(vw, dtype=torch.float32, device=device)
        img = _render_one(splats, multi_K, vm, multi_W, multi_H, bg)
        external_views.append((title, img))

    fig, axes = plt.subplots(2, 2, figsize=(12, 10), facecolor="white")
    for ax, (title, img) in zip(axes.flat, external_views):
        ax.imshow(img); ax.axis("off"); ax.set_title(title, fontsize=12)
    fig.suptitle("3DGS scene viewed from outside (4 external angles)", fontsize=14)
    fig.tight_layout()
    multi_path = out_dir / "external_4views.png"
    fig.savefig(multi_path, dpi=160, facecolor="white", bbox_inches="tight")
    plt.close(fig)
    print(f"保存外部 4 视角: {multi_path}")

    summary = {
        "ckpt": str(ckpt_path),
        "num_gaussians": int(splats["means"].shape[0]),
        "scene_dir": str(scene_dir),
        "result_dir": str(result_dir),
        "render_resolution": [rW, rH],
        "comparison_png": str(cmp_path),
        "trajectory_grid_png": str(grid_path),
        "hero_png": str(hero_path),
    }
    (out_dir / "paper_renders_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
