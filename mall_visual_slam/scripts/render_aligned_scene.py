#!/usr/bin/env python3
"""
渲染 VGGT 对齐后的全局 3D 场景，输出论文风格的 PNG 截图。

为什么单独写这个脚本：
WSL 上的 viser 必须靠浏览器看，截图不方便；这个脚本走纯 matplotlib，
能直接保存几张固定视角 + 一组旋转视角的 PNG，方便在文档 / 汇报里贴。
渲染内容对齐 VGGT 论文 Fig.4 的风格：
1. 全局点云（按 RGB 颜色或按相机 viridis 上色）；
2. 相机 frustum；
3. 相机中心连成一条轨迹线。
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import numpy as np


REPO = Path(__file__).resolve().parent.parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="渲染 VGGT 对齐后的 3D 场景")
    parser.add_argument(
        "--result-dir",
        default=str(REPO / "output" / "vggt_input_video_show"),
        help="run_vggt_video.py 的输出目录（要带 aligned_full 子目录）",
    )
    parser.add_argument(
        "--aligned-subdir",
        default="aligned_full",
        help="对齐结果的子目录名",
    )
    parser.add_argument(
        "--output-dir",
        default="",
        help="渲染图保存目录；默认在 aligned_full 下建 renders 子目录",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=80000,
        help="渲染时最多保留多少个点；太多会让 matplotlib 渲染极慢",
    )
    parser.add_argument(
        "--point-size",
        type=float,
        default=0.6,
        help="散点大小",
    )
    parser.add_argument(
        "--frustum-scale",
        type=float,
        default=0.04,
        help="相机 frustum 在场景中的大小",
    )
    parser.add_argument(
        "--show-every-cam",
        type=int,
        default=4,
        help="每隔多少帧画一个相机 frustum，避免太密",
    )
    parser.add_argument(
        "--bg",
        choices=["white", "black"],
        default="white",
        help="渲染背景色",
    )
    parser.add_argument(
        "--rotate-frames",
        type=int,
        default=24,
        help="生成多少张围绕场景中心旋转的视角图；0 表示不生成",
    )
    parser.add_argument(
        "--no-static",
        action="store_true",
        help="不输出 4 张固定视角的静态截图",
    )
    return parser.parse_args()


def read_ascii_ply(path: Path) -> tuple[np.ndarray, np.ndarray]:
    vertex_count = None
    header_lines = 0
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            header_lines += 1
            if line.startswith("element vertex"):
                vertex_count = int(line.split()[-1])
            if line.strip() == "end_header":
                break
    if vertex_count is None or vertex_count == 0:
        raise ValueError(f"PLY 异常: {path}")
    data = np.loadtxt(path, skiprows=header_lines, max_rows=vertex_count, dtype=np.float32)
    if data.ndim == 1:
        data = data[None, :]
    points = data[:, :3].astype(np.float32)
    colors = np.clip(data[:, 3:6], 0, 255).astype(np.uint8)
    return points, colors


def downsample(points: np.ndarray, colors: np.ndarray, max_points: int) -> tuple[np.ndarray, np.ndarray]:
    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    colors = colors[finite]
    if max_points > 0 and len(points) > max_points:
        rng = np.random.default_rng(42)
        idx = rng.choice(len(points), size=max_points, replace=False)
        points = points[idx]
        colors = colors[idx]
    return points, colors


def remove_outliers_radius(points: np.ndarray, colors: np.ndarray, percentile: float = 99.0) -> tuple[np.ndarray, np.ndarray]:
    """简单的中心距离离群剔除：把离场景中心最远的几个百分位丢掉。

    论文截图里的场景一般是紧凑团块，VGGT 偶尔会有少量"漂浮"远点拉爆坐标轴。
    """
    if len(points) == 0:
        return points, colors
    center = np.median(points, axis=0)
    dist = np.linalg.norm(points - center, axis=1)
    cutoff = float(np.percentile(dist, percentile))
    mask = dist <= cutoff
    return points[mask], colors[mask]


def frustum_lines(
    position: np.ndarray,
    rotation: np.ndarray,
    scale: float,
    aspect: float = 1.0,
    fov_deg: float = 60.0,
) -> list[tuple[np.ndarray, np.ndarray]]:
    """生成一个简化相机 frustum 的线段列表（每条线返回起点+终点）。

    这里只画 4 条 apex->corner 线 + 4 条 corner 连边，共 8 条；apex 取相机中心 position，
    corner 在相机 +Z 方向 scale 远的位置（相机系是 OpenCV 约定）。
    """
    half_h = scale * np.tan(np.radians(fov_deg) / 2.0)
    half_w = half_h * aspect
    # 相机坐标系下的 4 个 corner 和 apex。
    apex_cam = np.zeros(3)
    corners_cam = np.array(
        [
            [+half_w, +half_h, scale],
            [-half_w, +half_h, scale],
            [-half_w, -half_h, scale],
            [+half_w, -half_h, scale],
        ]
    )
    # 相机系 -> 世界系：rotation 是 cam-to-world，position 是相机中心。
    apex_world = rotation @ apex_cam + position
    corners_world = (rotation @ corners_cam.T).T + position

    lines = []
    for c in corners_world:
        lines.append((apex_world, c))
    for i in range(4):
        lines.append((corners_world[i], corners_world[(i + 1) % 4]))
    return lines


def render_paper_style_scene(
    points: np.ndarray,
    colors: np.ndarray,
    cameras: list[dict],
    output_dir: Path,
    point_size: float,
    frustum_scale: float,
    show_every_cam: int,
    bg: str,
    rotate_frames: int,
    static_views: bool,
) -> dict:
    """主渲染函数。返回保存的文件清单。"""
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Line3DCollection

    output_dir.mkdir(parents=True, exist_ok=True)
    saved: list[Path] = []

    # 把点云中心化，让 matplotlib 的 box_aspect 不被远点拉爆。
    center = np.median(points, axis=0)
    points_centered = points - center

    # 相机轨迹同样平移。注意 cameras 里的 position 是绝对坐标。
    cam_positions = np.asarray([np.asarray(c["position"], dtype=np.float32) for c in cameras])
    cam_positions_centered = cam_positions - center
    cam_rotations = [np.asarray(c["rotation"], dtype=np.float32) for c in cameras]

    # 计算渲染 box。
    all_points_for_box = np.vstack([points_centered, cam_positions_centered])
    box_min = all_points_for_box.min(axis=0)
    box_max = all_points_for_box.max(axis=0)
    box_center = (box_min + box_max) / 2.0
    box_extent = (box_max - box_min).max() / 2.0 * 1.05

    def setup_axes(ax) -> None:
        ax.set_facecolor(bg)
        ax.set_xlim(box_center[0] - box_extent, box_center[0] + box_extent)
        ax.set_ylim(box_center[1] - box_extent, box_center[1] + box_extent)
        ax.set_zlim(box_center[2] - box_extent, box_center[2] + box_extent)
        # 论文风格通常隐藏坐标轴。
        ax.set_xticks([]); ax.set_yticks([]); ax.set_zticks([])
        if bg == "black":
            ax.xaxis.pane.set_facecolor("black")
            ax.yaxis.pane.set_facecolor("black")
            ax.zaxis.pane.set_facecolor("black")
        ax.grid(False)
        ax.set_box_aspect((1, 1, 1))

    def draw_one(ax, elev: float, azim: float, title: str | None) -> None:
        setup_axes(ax)
        ax.view_init(elev=elev, azim=azim)

        # 点云。论文风格的图里点云通常偏弱、半透明，让相机轨迹和 frustum 浮出来。
        ax.scatter(
            points_centered[:, 0],
            points_centered[:, 1],
            points_centered[:, 2],
            c=colors.astype(np.float32) / 255.0,
            s=point_size,
            depthshade=False,
            linewidths=0,
            alpha=0.55,
        )

        # 相机 frustum。先画 frustum 让它落在更靠前的 z-order 上。
        every = max(1, show_every_cam)
        all_lines = []
        traj_color = "#ff7f00"
        frustum_color = "#0066cc" if bg == "white" else "#33ddff"
        for idx in range(0, len(cam_positions_centered), every):
            lines = frustum_lines(
                cam_positions_centered[idx],
                cam_rotations[idx],
                frustum_scale,
                aspect=1.0,
                fov_deg=55.0,
            )
            for start, end in lines:
                all_lines.append([start, end])
        if all_lines:
            lc = Line3DCollection(
                np.asarray(all_lines, dtype=np.float32),
                colors=frustum_color,
                linewidths=1.0,
                alpha=0.95,
            )
            ax.add_collection3d(lc)

        # 相机轨迹连线。比 frustum 更亮、更粗，主导视觉。
        if len(cam_positions_centered) >= 2:
            ax.plot(
                cam_positions_centered[:, 0],
                cam_positions_centered[:, 1],
                cam_positions_centered[:, 2],
                color=traj_color,
                linewidth=2.2,
                alpha=1.0,
                zorder=10,
            )
            # 起点 / 终点圆点。
            ax.scatter(
                [cam_positions_centered[0, 0]],
                [cam_positions_centered[0, 1]],
                [cam_positions_centered[0, 2]],
                color="#22cc44", s=60, depthshade=False, zorder=11,
            )
            ax.scatter(
                [cam_positions_centered[-1, 0]],
                [cam_positions_centered[-1, 1]],
                [cam_positions_centered[-1, 2]],
                color="#ee2233", s=60, depthshade=False, zorder=11,
            )

        if title:
            text_color = "white" if bg == "black" else "black"
            ax.set_title(title, color=text_color, fontsize=11, pad=4)

    # 4 张固定视角截图：俯视、侧视 2 张、3/4 视角。
    if static_views:
        view_specs = [
            ("topdown", 88.0, -90.0, "Top-down"),
            ("side_x", 5.0, -90.0, "Side view (X axis)"),
            ("side_z", 5.0, 0.0, "Side view (Z axis)"),
            ("oblique", 25.0, -55.0, "3/4 view"),
        ]
        fig = plt.figure(figsize=(16, 12), facecolor=bg)
        for i, (name, elev, azim, title) in enumerate(view_specs):
            ax = fig.add_subplot(2, 2, i + 1, projection="3d")
            draw_one(ax, elev, azim, title)
        fig.tight_layout()
        path = output_dir / "aligned_scene_4views.png"
        fig.savefig(path, dpi=180, facecolor=bg)
        plt.close(fig)
        saved.append(path)

        # 单张 3/4 视角大图。
        fig = plt.figure(figsize=(11, 9), facecolor=bg)
        ax = fig.add_subplot(111, projection="3d")
        draw_one(ax, 22.0, -55.0, None)
        fig.tight_layout()
        path = output_dir / "aligned_scene_hero.png"
        fig.savefig(path, dpi=200, facecolor=bg)
        plt.close(fig)
        saved.append(path)

    # 旋转帧序列：用于 GIF/视频或人工挑选最好看的视角。
    if rotate_frames > 0:
        rot_dir = output_dir / "rotate_frames"
        rot_dir.mkdir(parents=True, exist_ok=True)
        # 避免 _frame_0 与 _frame_N 重复，azim 走 [0, 360) 的均匀采样。
        azimuths = np.linspace(0.0, 360.0, rotate_frames, endpoint=False)
        for i, azim in enumerate(azimuths):
            fig = plt.figure(figsize=(8, 7), facecolor=bg)
            ax = fig.add_subplot(111, projection="3d")
            draw_one(ax, 18.0, float(azim), None)
            fig.tight_layout()
            frame_path = rot_dir / f"frame_{i:03d}.png"
            fig.savefig(frame_path, dpi=120, facecolor=bg)
            plt.close(fig)
            saved.append(frame_path)

    return {
        "saved_files": [str(p) for p in saved],
        "num_points_rendered": int(len(points)),
        "num_cameras": int(len(cameras)),
    }


def main() -> None:
    args = parse_args()
    result_dir = Path(args.result_dir)
    aligned_dir = result_dir / args.aligned_subdir

    ply_path = aligned_dir / "aligned_full_scene.ply"
    traj_path = aligned_dir / "aligned_camera_trajectory.json"
    if not ply_path.exists():
        raise FileNotFoundError(
            f"找不到对齐点云: {ply_path}\n"
            "请先运行 scripts/run_vggt_video.py 或 scripts/aggregate_vggt_aligned.py。"
        )

    print(f"读取对齐点云: {ply_path}")
    points, colors = read_ascii_ply(ply_path)
    print(f"  原始点数: {len(points)}")

    points, colors = remove_outliers_radius(points, colors, percentile=99.0)
    print(f"  剔除 1% 远点后: {len(points)}")

    points, colors = downsample(points, colors, args.max_points)
    print(f"  下采样到: {len(points)}")

    cameras: list[dict] = []
    if traj_path.exists():
        traj = json.loads(traj_path.read_text(encoding="utf-8"))
        # 同一个 source_frame 在多个窗口里都会出现，去重只保留第一次。
        seen = set()
        for cam in traj.get("cameras", []):
            sf = cam.get("source_frame")
            if sf in seen:
                continue
            seen.add(sf)
            cameras.append(cam)
        cameras.sort(key=lambda c: c.get("source_frame", 0))
        print(f"  相机数: {len(cameras)} (按 source_frame 去重)")

    output_dir = Path(args.output_dir) if args.output_dir else aligned_dir / "renders"
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-vggt")

    summary = render_paper_style_scene(
        points=points,
        colors=colors,
        cameras=cameras,
        output_dir=output_dir,
        point_size=args.point_size,
        frustum_scale=args.frustum_scale,
        show_every_cam=args.show_every_cam,
        bg=args.bg,
        rotate_frames=args.rotate_frames,
        static_views=not args.no_static,
    )

    print()
    print("渲染完成。文件清单：")
    for p in summary["saved_files"][:6]:
        print(f"  {p}")
    if len(summary["saved_files"]) > 6:
        print(f"  ... 还有 {len(summary['saved_files']) - 6} 张")
    print(f"输出目录: {output_dir}")


if __name__ == "__main__":
    main()
