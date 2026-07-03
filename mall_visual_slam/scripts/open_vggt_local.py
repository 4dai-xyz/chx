#!/usr/bin/env python3
"""
用本地 matplotlib 窗口查看 VGGT 生成的 PLY 点云。

这个脚本不启动网页服务，适合在不想用 Viser/browser 的时候直接看 3D 点云。
默认打开当前完整结果：

output/vggt_input_video_show/full_viewer/vggt_full_windows_aggregated.ply

如果当前 WSL 图形窗口不可用，脚本仍会保存一张 PNG 预览图，方便先确认点云是否正常。
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import numpy as np


REPO = Path("/home/ros/ros2_orbslam3")
DEFAULT_PLY = REPO / "output" / "vggt_input_video_show" / "full_viewer" / "vggt_full_windows_aggregated.ply"
DEFAULT_PREVIEW = REPO / "output" / "vggt_input_video_show" / "full_viewer" / "vggt_full_windows_preview.png"


def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(description="Open VGGT PLY point cloud with a local matplotlib 3D window")
    parser.add_argument("--ply", default=str(DEFAULT_PLY), help="要打开的 PLY 点云文件")
    parser.add_argument("--save-preview", default=str(DEFAULT_PREVIEW), help="保存静态 3D 预览图的位置")
    parser.add_argument("--max-points", type=int, default=120000, help="最多显示多少个点，太多会让 matplotlib 变慢")
    parser.add_argument("--point-size", type=float, default=0.15, help="点的显示大小")
    parser.add_argument("--elev", type=float, default=22.0, help="默认观察仰角")
    parser.add_argument("--azim", type=float, default=-65.0, help="默认观察方位角")
    parser.add_argument("--no-show", action="store_true", help="只保存预览图，不弹出窗口")
    parser.add_argument(
        "--preserve-scale",
        action="store_true",
        help="保持原始坐标比例；默认会把三个坐标轴分别缩放到相近范围，方便观察细节",
    )
    return parser.parse_args()


def read_ascii_ply(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """读取本项目导出的 ASCII PLY 点云，返回点坐标和颜色。"""
    if not path.exists():
        raise FileNotFoundError(f"找不到点云文件: {path}")

    vertex_count = None
    header_lines = 0
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            header_lines += 1
            if line.startswith("element vertex"):
                vertex_count = int(line.split()[-1])
            if line.strip() == "end_header":
                break

    if vertex_count is None:
        raise ValueError(f"无法从 PLY 头部读取点数量: {path}")
    if vertex_count == 0:
        raise ValueError(f"PLY 点云为空: {path}")

    data = np.loadtxt(path, skiprows=header_lines, max_rows=vertex_count, dtype=np.float32)
    if data.ndim == 1:
        data = data[None, :]

    points = data[:, :3].astype(np.float32)
    if data.shape[1] >= 6:
        colors = np.clip(data[:, 3:6], 0, 255).astype(np.uint8)
    else:
        colors = np.full((len(points), 3), 180, dtype=np.uint8)
    return points, colors


def choose_backend(no_show: bool) -> str:
    """选择 matplotlib 后端；有图形显示时优先使用 TkAgg，否则使用 Agg 保存图片。"""
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-vggt")

    import matplotlib

    if no_show:
        matplotlib.use("Agg", force=True)
        return "Agg"

    has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    if has_display:
        try:
            matplotlib.use("TkAgg", force=True)
            return "TkAgg"
        except Exception:
            matplotlib.use("Agg", force=True)
            return "Agg"

    matplotlib.use("Agg", force=True)
    return "Agg"


def downsample(points: np.ndarray, colors: np.ndarray, max_points: int) -> tuple[np.ndarray, np.ndarray]:
    """随机下采样，避免 matplotlib 一次画太多点导致窗口卡顿。"""
    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    colors = colors[finite]

    if max_points > 0 and len(points) > max_points:
        rng = np.random.default_rng(42)
        selected = rng.choice(len(points), size=max_points, replace=False)
        points = points[selected]
        colors = colors[selected]
    return points, colors


def prepare_points(points: np.ndarray, preserve_scale: bool) -> np.ndarray:
    """整理点云坐标，让本地窗口里更容易看清楚结构。"""
    points = points - np.mean(points, axis=0, keepdims=True)
    if preserve_scale:
        return points

    span = np.ptp(points, axis=0)
    span[span < 1e-6] = 1.0
    return points / span.max()


def plot_point_cloud(
    points: np.ndarray,
    colors: np.ndarray,
    title: str,
    save_preview: Path,
    point_size: float,
    elev: float,
    azim: float,
    show: bool,
) -> None:
    """绘制点云，保存预览图，并在可用时弹出本地 3D 窗口。"""
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(
        points[:, 0],
        points[:, 1],
        points[:, 2],
        c=colors.astype(np.float32) / 255.0,
        s=point_size,
        depthshade=False,
        linewidths=0,
    )

    ax.set_title(title)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.view_init(elev=elev, azim=azim)
    ax.set_box_aspect((1, 1, 1))
    fig.tight_layout()

    save_preview.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save_preview, dpi=180)
    print(f"已保存静态 3D 预览图: {save_preview}")

    if show:
        print("已打开本地 3D 窗口。可以用鼠标拖动旋转，关闭窗口即可退出。")
        plt.show()
    else:
        plt.close(fig)


def main() -> None:
    """脚本入口。"""
    args = parse_args()
    ply_path = Path(args.ply)
    preview_path = Path(args.save_preview)

    backend = choose_backend(args.no_show)
    points, colors = read_ascii_ply(ply_path)
    total_points = len(points)
    points, colors = downsample(points, colors, args.max_points)
    points = prepare_points(points, args.preserve_scale)

    print(f"点云文件: {ply_path}")
    print(f"原始点数: {total_points}")
    print(f"显示点数: {len(points)}")
    print(f"matplotlib 后端: {backend}")

    show_window = backend != "Agg" and not args.no_show
    plot_point_cloud(
        points=points,
        colors=colors,
        title=f"VGGT point cloud: {ply_path.name}",
        save_preview=preview_path,
        point_size=args.point_size,
        elev=args.elev,
        azim=args.azim,
        show=show_window,
    )

    if not show_window:
        print("当前没有可用的本地弹窗后端，只生成了 PNG 预览图。")


if __name__ == "__main__":
    main()
