#!/usr/bin/env python3
"""把训练好的 3DGS PLY 当成「带颜色的稠密点云」用 matplotlib 渲染。

为什么这么做：当前商场视频是一条直线前进的相机轨迹，每个表面只被
窄角度看到 → 3DGS 高斯收敛成「针状」椭球，从训练视角外看会出现长
条状投影伪影。但每个高斯的 mean 位置 + 颜色（用 SH0 直流分量解码）
本质上是个稠密 RGB 点云，绕场景外旋视可以得到非常干净的「室内空间
俯瞰图」，视觉上接近论文 Fig.1 的风格。

输出 4 张外部视角 + 1 张 hero 拼图。
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


REPO = Path(__file__).resolve().parent.parent

# SH 直流分量解码常数，参考 INRIA 3DGS / gsplat 约定：
# 颜色 = sigmoid(C0 * f_dc + 0.5)，但 gsplat 训练保存的是 sh0 已经是 color/2C 的 SH 系数。
SH_C0 = 0.28209479177387814


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument(
        "--ply",
        default=str(
            REPO
            / "output"
            / "vggt_aligned_full_run"
            / "gsplat_results_30k"
            / "ply"
            / "point_cloud_29999.ply"
        ),
    )
    p.add_argument(
        "--out-dir",
        default=str(
            REPO
            / "output"
            / "vggt_aligned_full_run"
            / "gsplat_results_30k"
            / "paper_renders_pointcloud"
        ),
    )
    p.add_argument("--max-points", type=int, default=200_000)
    p.add_argument("--opacity-min", type=float, default=0.05, help="过滤几乎全透明的高斯")
    p.add_argument("--scale-max-quantile", type=float, default=0.95, help="剔除超大高斯（多为浮云）")
    p.add_argument("--bg", choices=["white", "black"], default="white")
    p.add_argument("--point-size", type=float, default=0.6)
    p.add_argument("--cam-traj", default=str(
        REPO / "output" / "vggt_aligned_full_run" / "aligned_full" / "aligned_camera_trajectory.json"
    ))
    return p.parse_args()


def _sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-np.clip(x, -30, 30)))


def load_gaussian_ply(path: Path) -> dict:
    """读 gsplat 训练保存的 3DGS PLY，提取位置 / 颜色 / 不透明度 / 尺度。"""
    with open(path, "rb") as f:
        header_lines = []
        while True:
            line = f.readline()
            if not line:
                raise ValueError("bad PLY")
            text = line.decode("ascii", errors="ignore").strip()
            header_lines.append(text)
            if text == "end_header":
                break

        vertex_count = 0
        properties: list[tuple[str, str]] = []
        fmt = "binary_little_endian"
        for line in header_lines:
            if line.startswith("format "):
                fmt = line.split()[1]
            elif line.startswith("element vertex"):
                vertex_count = int(line.split()[-1])
            elif line.startswith("property "):
                tokens = line.split()
                properties.append((tokens[2], tokens[1]))

        if fmt != "binary_little_endian":
            raise ValueError(f"unsupported PLY format: {fmt}")

        type_map = {
            "float": "<f4",
            "float32": "<f4",
            "double": "<f8",
            "uchar": "u1",
            "uint8": "u1",
            "int": "<i4",
        }
        dtype = np.dtype([(name, type_map[ty]) for name, ty in properties])
        body_offset = f.tell()
        f.seek(body_offset)
        data = np.fromfile(f, dtype=dtype, count=vertex_count)

    xyz = np.stack([data["x"], data["y"], data["z"]], axis=-1).astype(np.float32)
    # f_dc_0/1/2 是 SH 直流分量。颜色 = sigmoid(SH_C0 * f_dc + 0.5)? 不对。
    # 3DGS 约定：RGB = 0.5 + SH_C0 * f_dc_*; 再 clamp 到 [0,1] 即可。
    f_dc = np.stack([data["f_dc_0"], data["f_dc_1"], data["f_dc_2"]], axis=-1).astype(np.float32)
    rgb = np.clip(0.5 + SH_C0 * f_dc, 0.0, 1.0)

    opacity = _sigmoid(data["opacity"].astype(np.float32))
    scales = np.stack([data["scale_0"], data["scale_1"], data["scale_2"]], axis=-1).astype(np.float32)
    scales = np.exp(scales)  # gsplat 存 log-scale

    return {
        "xyz": xyz,
        "rgb": rgb,
        "opacity": opacity,
        "scale": scales,
    }


def render(args: argparse.Namespace) -> None:
    import json
    import os
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-gs")
    import matplotlib
    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Line3DCollection

    ply = Path(args.ply)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"读取 3DGS PLY: {ply}")
    g = load_gaussian_ply(ply)
    print(f"  原始高斯数: {len(g['xyz'])}")

    # 过滤：opacity 太低 + scale 太大（多为浮云）。
    mask = g["opacity"] >= args.opacity_min
    scale_max = g["scale"].max(axis=-1)
    cut = np.quantile(scale_max, args.scale_max_quantile)
    mask &= scale_max <= cut
    xyz = g["xyz"][mask]
    rgb = g["rgb"][mask]
    print(f"  过滤后: {len(xyz)} (opacity>={args.opacity_min}, scale 上 {args.scale_max_quantile*100:.0f}% 截断)")

    if len(xyz) > args.max_points:
        rng = np.random.default_rng(0)
        idx = rng.choice(len(xyz), size=args.max_points, replace=False)
        xyz = xyz[idx]
        rgb = rgb[idx]
    print(f"  渲染点数: {len(xyz)}")

    # 中心化 + 计算包围盒。
    center = np.median(xyz, axis=0)
    xyz = xyz - center
    # 剔除离中心最远 0.5% 的点（极少数飞点）。
    dist = np.linalg.norm(xyz, axis=1)
    cut2 = np.quantile(dist, 0.995)
    keep = dist <= cut2
    xyz = xyz[keep]
    rgb = rgb[keep]

    box_min = xyz.min(axis=0)
    box_max = xyz.max(axis=0)
    bc = (box_min + box_max) / 2
    be = (box_max - box_min).max() / 2 * 1.05

    # 相机轨迹（用 aligned_camera_trajectory.json）。
    cams = []
    traj_p = Path(args.cam_traj)
    if traj_p.exists():
        seen = set()
        for c in json.loads(traj_p.read_text(encoding="utf-8")).get("cameras", []):
            sf = c.get("source_frame")
            if sf in seen:
                continue
            seen.add(sf)
            cams.append(np.asarray(c["position"], dtype=np.float32))
        cams.sort(key=lambda _: 0)
        cams = np.asarray(sorted([np.asarray(c["position"], dtype=np.float32)
                                    for c in json.loads(traj_p.read_text(encoding='utf-8')).get("cameras", [])
                                    if c["source_frame"] in seen],
                                  key=lambda p: float(p[2]))) if False else None
        # 实际上 cameras 是 list；保留按顺序、去重后中心化。
        cam_list = []
        seen2 = set()
        for c in json.loads(traj_p.read_text(encoding='utf-8')).get("cameras", []):
            sf = c["source_frame"]
            if sf in seen2: continue
            seen2.add(sf)
            cam_list.append((sf, np.asarray(c["position"], dtype=np.float32)))
        cam_list.sort(key=lambda x: x[0])
        cams = np.asarray([p for _, p in cam_list]) - center
    else:
        cams = np.empty((0, 3), dtype=np.float32)

    bg = args.bg
    text_color = "white" if bg == "black" else "#222"

    def setup(ax):
        ax.set_facecolor(bg)
        ax.set_xlim(bc[0] - be, bc[0] + be)
        ax.set_ylim(bc[1] - be, bc[1] + be)
        ax.set_zlim(bc[2] - be, bc[2] + be)
        ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
        if bg == "black":
            ax.xaxis.pane.set_facecolor("black")
            ax.yaxis.pane.set_facecolor("black")
            ax.zaxis.pane.set_facecolor("black")
        ax.grid(False)
        ax.set_box_aspect((1, 1, 1))

    def draw_scene(ax, elev, azim, title=None):
        setup(ax)
        ax.view_init(elev=elev, azim=azim)
        ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], c=rgb, s=args.point_size,
                   depthshade=False, linewidths=0, alpha=0.85)
        if len(cams) >= 2:
            ax.plot(cams[:, 0], cams[:, 1], cams[:, 2], color="#ff7f00", linewidth=2.0, zorder=10)
            ax.scatter([cams[0, 0]], [cams[0, 1]], [cams[0, 2]], color="#22cc44", s=60, depthshade=False, zorder=11)
            ax.scatter([cams[-1, 0]], [cams[-1, 1]], [cams[-1, 2]], color="#ee2233", s=60, depthshade=False, zorder=11)
        if title:
            ax.set_title(title, color=text_color, fontsize=11, pad=4)

    # 单图大 hero
    fig = plt.figure(figsize=(12, 9), facecolor=bg)
    ax = fig.add_subplot(111, projection="3d")
    draw_scene(ax, elev=22, azim=-55, title=None)
    fig.tight_layout()
    hero_path = out_dir / "gsplat_pointcloud_hero.png"
    fig.savefig(hero_path, dpi=200, facecolor=bg, bbox_inches="tight")
    plt.close(fig)
    print(f"保存 hero: {hero_path}")

    # 4 视角
    specs = [
        ("Top-down", 88, -90),
        ("Side (X)", 6, -90),
        ("Side (Z)", 6, 0),
        ("Diagonal", 22, -55),
    ]
    fig = plt.figure(figsize=(14, 11), facecolor=bg)
    for i, (title, elev, azim) in enumerate(specs):
        ax = fig.add_subplot(2, 2, i + 1, projection="3d")
        draw_scene(ax, elev, azim, title)
    fig.tight_layout()
    multi_path = out_dir / "gsplat_pointcloud_4views.png"
    fig.savefig(multi_path, dpi=180, facecolor=bg, bbox_inches="tight")
    plt.close(fig)
    print(f"保存 4 视角: {multi_path}")


if __name__ == "__main__":
    render(parse_args())
