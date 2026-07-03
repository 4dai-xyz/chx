#!/usr/bin/env python3
"""
把对齐后的 3D 场景渲染成一张完整的「论文 Fig.4 风格」图：
左侧大图是 3/4 hero 视角，右侧三个小图分别是俯视、侧视、轨迹特写。

只输出一张 PNG，方便直接贴文档/汇报。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


REPO = Path(__file__).resolve().parent.parent


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
    if not vertex_count:
        raise ValueError(f"bad PLY: {path}")
    data = np.loadtxt(path, skiprows=header_lines, max_rows=vertex_count, dtype=np.float32)
    if data.ndim == 1:
        data = data[None, :]
    return data[:, :3].astype(np.float32), np.clip(data[:, 3:6], 0, 255).astype(np.uint8)


def frustum_lines(position, rotation, scale, fov_deg=55.0, aspect=1.0):
    half_h = scale * np.tan(np.radians(fov_deg) / 2.0)
    half_w = half_h * aspect
    apex = np.zeros(3)
    corners = np.array([
        [+half_w, +half_h, scale],
        [-half_w, +half_h, scale],
        [-half_w, -half_h, scale],
        [+half_w, -half_h, scale],
    ])
    apex_w = rotation @ apex + position
    corners_w = (rotation @ corners.T).T + position
    lines = [(apex_w, c) for c in corners_w]
    for i in range(4):
        lines.append((corners_w[i], corners_w[(i + 1) % 4]))
    return lines


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--result-dir", default=str(REPO / "output" / "vggt_aligned_full_run"))
    parser.add_argument("--aligned-subdir", default="aligned_full")
    parser.add_argument("--out", default="")
    parser.add_argument("--bg", choices=["white", "black"], default="white")
    parser.add_argument("--max-points", type=int, default=150000)
    args = parser.parse_args()

    aligned_dir = Path(args.result_dir) / args.aligned_subdir
    ply = aligned_dir / "aligned_full_scene.ply"
    traj_p = aligned_dir / "aligned_camera_trajectory.json"
    out_path = Path(args.out) if args.out else aligned_dir / "renders" / "paper_figure_hero.png"

    points, colors = read_ascii_ply(ply)
    # 远点剔除（99%）。
    center = np.median(points, axis=0)
    dist = np.linalg.norm(points - center, axis=1)
    mask = dist <= np.percentile(dist, 99.0)
    points, colors = points[mask], colors[mask]
    if len(points) > args.max_points:
        rng = np.random.default_rng(42)
        idx = rng.choice(len(points), size=args.max_points, replace=False)
        points, colors = points[idx], colors[idx]

    cams = []
    if traj_p.exists():
        seen = set()
        for c in json.loads(traj_p.read_text(encoding="utf-8")).get("cameras", []):
            sf = c.get("source_frame")
            if sf in seen:
                continue
            seen.add(sf)
            cams.append(c)
        cams.sort(key=lambda c: c.get("source_frame", 0))

    cam_pos = np.array([c["position"] for c in cams], dtype=np.float32)
    cam_rot = [np.array(c["rotation"], dtype=np.float32) for c in cams]

    # 中心化。
    box_center = np.median(points, axis=0)
    points = points - box_center
    cam_pos = cam_pos - box_center

    all_for_box = np.vstack([points, cam_pos])
    box_min = all_for_box.min(axis=0)
    box_max = all_for_box.max(axis=0)
    bc = (box_min + box_max) / 2
    be = (box_max - box_min).max() / 2 * 1.05

    # frustum 尺寸正比于场景。
    frustum_scale = float(be * 0.045)

    import os
    os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-vggt")
    import matplotlib
    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Line3DCollection

    bg = args.bg
    text_color = "white" if bg == "black" else "#222222"
    traj_color = "#ff7f00"
    frustum_color = "#0066cc" if bg == "white" else "#33ddff"

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

    def draw(ax, elev, azim, frustum_every=6, point_alpha=0.55, point_size=0.55, title=None):
        setup(ax)
        ax.view_init(elev=elev, azim=azim)
        ax.scatter(points[:, 0], points[:, 1], points[:, 2],
                   c=colors.astype(np.float32) / 255.0,
                   s=point_size, depthshade=False, linewidths=0, alpha=point_alpha)
        # 相机 frustum。
        lines = []
        for i in range(0, len(cam_pos), max(1, frustum_every)):
            for s, e in frustum_lines(cam_pos[i], cam_rot[i], frustum_scale):
                lines.append([s, e])
        if lines:
            ax.add_collection3d(Line3DCollection(np.asarray(lines, dtype=np.float32),
                                                 colors=frustum_color, linewidths=0.9, alpha=0.95))
        # 轨迹。
        if len(cam_pos) >= 2:
            ax.plot(cam_pos[:, 0], cam_pos[:, 1], cam_pos[:, 2],
                    color=traj_color, linewidth=2.4, zorder=10)
            ax.scatter([cam_pos[0, 0]], [cam_pos[0, 1]], [cam_pos[0, 2]],
                       color="#22cc44", s=70, depthshade=False, zorder=11)
            ax.scatter([cam_pos[-1, 0]], [cam_pos[-1, 1]], [cam_pos[-1, 2]],
                       color="#ee2233", s=70, depthshade=False, zorder=11)
        if title:
            ax.set_title(title, color=text_color, fontsize=12, pad=4)

    fig = plt.figure(figsize=(18, 11), facecolor=bg)
    gs = fig.add_gridspec(2, 3, width_ratios=[2.2, 1, 1], wspace=0.02, hspace=0.05)

    # 大图：3/4 视角 hero。
    ax_hero = fig.add_subplot(gs[:, 0], projection="3d")
    draw(ax_hero, elev=22.0, azim=-55.0, frustum_every=6, point_alpha=0.55, point_size=0.6,
         title="VGGT aligned 3D scene (108s mall walk)")

    # 右上：俯视。
    ax_top = fig.add_subplot(gs[0, 1], projection="3d")
    draw(ax_top, elev=88.0, azim=-90.0, frustum_every=8, point_alpha=0.45, point_size=0.4,
         title="Top-down")

    # 右中：侧视 X。
    ax_sx = fig.add_subplot(gs[0, 2], projection="3d")
    draw(ax_sx, elev=8.0, azim=-90.0, frustum_every=8, point_alpha=0.45, point_size=0.4,
         title="Side view")

    # 右下：相机轨迹特写（无点云）。
    ax_tr = fig.add_subplot(gs[1, 1:], projection="3d")
    setup(ax_tr)
    ax_tr.view_init(elev=18.0, azim=-50.0)
    lines = []
    for i in range(0, len(cam_pos)):
        for s, e in frustum_lines(cam_pos[i], cam_rot[i], frustum_scale * 1.4):
            lines.append([s, e])
    ax_tr.add_collection3d(Line3DCollection(np.asarray(lines, dtype=np.float32),
                                              colors=frustum_color, linewidths=0.9, alpha=0.95))
    ax_tr.plot(cam_pos[:, 0], cam_pos[:, 1], cam_pos[:, 2],
               color=traj_color, linewidth=2.6, zorder=10)
    ax_tr.scatter([cam_pos[0, 0]], [cam_pos[0, 1]], [cam_pos[0, 2]],
                  color="#22cc44", s=90, depthshade=False, zorder=11, label="Start")
    ax_tr.scatter([cam_pos[-1, 0]], [cam_pos[-1, 1]], [cam_pos[-1, 2]],
                  color="#ee2233", s=90, depthshade=False, zorder=11, label="End")
    leg = ax_tr.legend(loc="upper right", facecolor=bg, edgecolor="none", labelcolor=text_color)
    ax_tr.set_title("Camera trajectory + frustums (107 frames)", color=text_color, fontsize=12, pad=4)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=200, facecolor=bg, bbox_inches="tight")
    plt.close(fig)
    print(f"saved: {out_path}")


if __name__ == "__main__":
    main()
