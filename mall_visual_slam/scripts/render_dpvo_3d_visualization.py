#!/usr/bin/env python3
"""生成 DPVO 增强结果的三维可视化视频。

这个脚本不会覆盖现有二维增强视频。它读取已经生成的 DPVO 增强结果：

- tracking_quality.csv
- keyframes/keyframes.json
- tracking_events.json
- relocalization_results.json
- DPVO TUM 轨迹文件

然后输出一个独立的 3D 回放结果：

- visualization_3d/dpvo_enhanced_3d.mp4
- visualization_3d/snapshots/*.jpg
- visualization_3d/summary.json

注意：如果当前没有 DPVO 点云 PLY，这个脚本会先画 3D 轨迹、关键帧相机、
当前相机视锥和重定位状态。若后续重新跑 DPVO 并保存 PLY，可以用
--pointcloud 指定点云文件，画面会更接近作者 demo 中的三维场景效果。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import cv2

os.environ.setdefault("MPLCONFIGDIR", "/home/ros/ros2_orbslam3/.runtime/matplotlib_dpvo_3d")
Path(os.environ["MPLCONFIGDIR"]).mkdir(parents=True, exist_ok=True)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


REPO = Path("/home/ros/ros2_orbslam3")
DEFAULT_ENHANCED_DIR = REPO / "output" / "dpvo_enhanced" / "mall_dpvo"
DEFAULT_TRAJECTORY = REPO / "Opensource code" / "DPVO-main" / "saved_trajectories" / "mall_dpvo.txt"


@dataclass
class PoseRow:
    """一帧 DPVO 位姿和增强质量信息。"""

    sample_index: int
    timestamp: float
    position: np.ndarray
    quaternion_xyzw: np.ndarray
    source_frame: int = 0
    source_time_sec: float = 0.0
    quality_score: float = 1.0
    quality_state: str = "good"
    mask_ratio: float = 0.0
    blur_laplacian: float = 0.0


def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(description="Render DPVO enhanced result as a 3D video")
    parser.add_argument("--enhanced-dir", default=str(DEFAULT_ENHANCED_DIR), help="DPVO 增强结果目录")
    parser.add_argument("--trajectory", default=str(DEFAULT_TRAJECTORY), help="DPVO TUM 轨迹文件")
    parser.add_argument("--output-dir", default="", help="3D 可视化输出目录；空表示 enhanced-dir/visualization_3d")
    parser.add_argument("--pointcloud", default="auto",
                        help="可选 PLY 点云路径；auto 会寻找 DPVO demo 的 <name>.ply；none 表示不加载")
    parser.add_argument("--name", default="mall_dpvo", help="运行名称，用于自动寻找 DPVO PLY")
    parser.add_argument("--every", type=int, default=4, help="每隔多少个 DPVO 样本渲染一帧 3D 视频")
    parser.add_argument("--fps", type=float, default=12.0, help="输出 3D 视频帧率")
    parser.add_argument("--width", type=int, default=1280, help="输出 3D 视频宽度")
    parser.add_argument("--height", type=int, default=720, help="输出 3D 视频高度")
    parser.add_argument("--snapshot-count", type=int, default=10, help="保存多少张 3D 截图")
    parser.add_argument("--max-points", type=int, default=70000, help="点云最多绘制多少个点")
    parser.add_argument("--align-world", choices=["raw", "trajectory_pca"], default="trajectory_pca",
                        help="三维显示坐标对齐方式；trajectory_pca 会把主要运动平面校正为水平")
    parser.add_argument("--first-turn", choices=["keep", "left", "right"], default="keep",
                        help="显示层校正首个明显转弯方向；left 会把第一段明显转弯显示为左转")
    parser.add_argument("--mirror-y", action="store_true",
                        help="强制左右镜像显示；用于修正没有真值地图时的左/右方向反转")
    parser.add_argument("--show", action="store_true", help="渲染时实时显示 3D 回放窗口，按 q 或 Esc 退出")
    return parser.parse_args()


def normalize_quaternion(quaternion_xyzw: np.ndarray) -> np.ndarray:
    """归一化四元数。"""
    norm = float(np.linalg.norm(quaternion_xyzw))
    if norm <= 1e-12:
        return np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return quaternion_xyzw.astype(np.float64) / norm


def quaternion_to_rotation_matrix(quaternion_xyzw: np.ndarray) -> np.ndarray:
    """把 xyzw 四元数转换为旋转矩阵。"""
    x, y, z, w = normalize_quaternion(quaternion_xyzw)
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def rotation_matrix_to_quaternion_xyzw(rotation: np.ndarray) -> np.ndarray:
    """把旋转矩阵转换为 xyzw 四元数。"""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(max(1e-12, 1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / s
        x = 0.25 * s
        y = (rotation[0, 1] + rotation[1, 0]) / s
        z = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(max(1e-12, 1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / s
        x = (rotation[0, 1] + rotation[1, 0]) / s
        y = 0.25 * s
        z = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(max(1e-12, 1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / s
        x = (rotation[0, 2] + rotation[2, 0]) / s
        y = (rotation[1, 2] + rotation[2, 1]) / s
        z = 0.25 * s
    return normalize_quaternion(np.asarray([x, y, z, w], dtype=np.float64))


def estimate_trajectory_pca_alignment(positions: np.ndarray) -> np.ndarray:
    """估计轨迹显示坐标系，把主要运动平面校正为水平。

    DPVO/单目 VO 的世界坐标没有真实重力方向。这里用轨迹 PCA 做显示层面的
    对齐：最大方差方向作为水平 x，第二大方差方向作为水平 y，最小方差方向
    作为竖直 z。这样能避免把“前进方向”误画成“向上爬”。
    """
    centered = positions - positions.mean(axis=0, keepdims=True)
    cov = centered.T @ centered / max(1, len(centered) - 1)
    _eigenvalues, eigenvectors = np.linalg.eigh(cov)
    order = np.argsort(_eigenvalues)[::-1]
    basis = eigenvectors[:, order]
    if np.linalg.det(basis) < 0:
        basis[:, 2] *= -1.0

    transformed = centered @ basis
    # 让整体前进方向大致朝 +x，减少每次渲染方向随机翻转。
    if transformed[-1, 0] < transformed[0, 0]:
        basis[:, 0] *= -1.0
    transformed = centered @ basis
    # 让起点附近的竖直高度靠近 0，若终点整体向上而用户实际下楼，可用显示层翻转。
    # 这里不强行判断上下楼，只保证主运动方向不再成为竖直方向。
    if np.linalg.det(basis) < 0:
        basis[:, 1] *= -1.0
    return basis


def cumulative_distances(points_xy: np.ndarray) -> np.ndarray:
    """计算二维轨迹的累计路径长度。"""
    if len(points_xy) == 0:
        return np.empty((0,), dtype=np.float64)
    steps = np.linalg.norm(np.diff(points_xy, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(steps)])


def point_at_path_fraction(points_xy: np.ndarray, cumulative: np.ndarray, fraction: float) -> np.ndarray:
    """取累计路径百分比处的二维点。"""
    if len(points_xy) == 0:
        return np.zeros((2,), dtype=np.float64)
    target = float(cumulative[-1]) * fraction
    index = int(np.searchsorted(cumulative, target, side="left"))
    index = min(max(index, 0), len(points_xy) - 1)
    return points_xy[index]


def estimate_first_turn_cross(aligned_positions: np.ndarray) -> float:
    """估计轨迹开头的二维转向符号。

    在显示坐标里，约定沿 +x 前进时，二维 x-y 平面中的正叉乘代表左转。
    这里使用累计路径的 3%、12%、28% 三个位置估计首个明显转弯。
    """
    points_xy = aligned_positions[:, :2]
    cumulative = cumulative_distances(points_xy)
    if len(cumulative) < 3 or cumulative[-1] <= 1e-9:
        return 0.0
    p0 = point_at_path_fraction(points_xy, cumulative, 0.03)
    p1 = point_at_path_fraction(points_xy, cumulative, 0.12)
    p2 = point_at_path_fraction(points_xy, cumulative, 0.28)
    v1 = p1 - p0
    v2 = p2 - p1
    return float(v1[0] * v2[1] - v1[1] * v2[0])


def apply_world_alignment(
    rows: list[PoseRow],
    pointcloud: tuple[np.ndarray, np.ndarray] | None,
    mode: str,
    first_turn: str,
    mirror_y: bool,
) -> tuple[np.ndarray, tuple[np.ndarray, np.ndarray] | None, dict]:
    """对轨迹、姿态和可选点云应用显示坐标对齐。"""
    positions = np.asarray([row.position for row in rows], dtype=np.float64)
    if mode == "raw":
        return positions, pointcloud, {
            "mode": "raw",
            "first_turn": first_turn,
            "first_turn_cross_before": 0.0,
            "first_turn_cross_after": 0.0,
            "turn_flip_applied": False,
            "mirror_y": mirror_y,
            "note": "使用 DPVO 原始坐标显示",
        }

    basis = estimate_trajectory_pca_alignment(positions)
    origin = positions[0].copy()
    aligned_positions = (positions - origin[None, :]) @ basis
    first_turn_cross_before = estimate_first_turn_cross(aligned_positions)
    turn_flip_applied = False
    if first_turn == "left" and first_turn_cross_before < 0:
        basis[:, 1] *= -1.0
        basis[:, 2] *= -1.0
        turn_flip_applied = True
    elif first_turn == "right" and first_turn_cross_before > 0:
        basis[:, 1] *= -1.0
        basis[:, 2] *= -1.0
        turn_flip_applied = True
    if mirror_y:
        basis[:, 1] *= -1.0
        basis[:, 2] *= -1.0
    aligned_positions = (positions - origin[None, :]) @ basis
    first_turn_cross_after = estimate_first_turn_cross(aligned_positions)

    for row, aligned_position in zip(rows, aligned_positions):
        old_rotation = quaternion_to_rotation_matrix(row.quaternion_xyzw)
        aligned_rotation = basis.T @ old_rotation
        row.position = aligned_position
        row.quaternion_xyzw = rotation_matrix_to_quaternion_xyzw(aligned_rotation)

    aligned_pointcloud = pointcloud
    if pointcloud is not None:
        points, colors = pointcloud
        aligned_pointcloud = ((points - origin[None, :]) @ basis, colors)

    return aligned_positions, aligned_pointcloud, {
        "mode": mode,
        "first_turn": first_turn,
        "first_turn_cross_before": first_turn_cross_before,
        "first_turn_cross_after": first_turn_cross_after,
        "turn_flip_applied": turn_flip_applied,
        "mirror_y": mirror_y,
        "origin": origin.tolist(),
        "basis_columns": basis.tolist(),
        "note": "使用轨迹 PCA 将主要运动平面校正为水平显示坐标，避免把前进方向误画成竖直方向。",
    }


def read_tum_trajectory(path: Path) -> list[PoseRow]:
    """读取 DPVO 保存的 TUM 轨迹。"""
    rows: list[PoseRow] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        values = [float(item) for item in line.split()]
        if len(values) != 8:
            continue
        timestamp, x, y, z, qx, qy, qz, qw = values
        rows.append(
            PoseRow(
                sample_index=int(round(timestamp)),
                timestamp=timestamp,
                position=np.asarray([x, y, z], dtype=np.float64),
                quaternion_xyzw=normalize_quaternion(np.asarray([qx, qy, qz, qw], dtype=np.float64)),
            )
        )
    if not rows:
        raise RuntimeError(f"轨迹文件为空或无法解析: {path}")
    return rows


def merge_quality(rows: list[PoseRow], enhanced_dir: Path) -> None:
    """把 tracking_quality.csv 中的质量信息合并到轨迹样本里。"""
    quality_path = enhanced_dir / "tracking_quality.csv"
    if not quality_path.exists():
        return
    by_sample = {row.sample_index: row for row in rows}
    with quality_path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for item in reader:
            sample_index = int(float(item["sample_index"]))
            row = by_sample.get(sample_index)
            if row is None:
                continue
            row.source_frame = int(float(item.get("source_frame", 0) or 0))
            row.source_time_sec = float(item.get("source_time_sec", 0.0) or 0.0)
            row.quality_score = float(item.get("quality_score", 1.0) or 1.0)
            row.quality_state = item.get("quality_state", "good") or "good"
            row.mask_ratio = float(item.get("mask_ratio", 0.0) or 0.0)
            row.blur_laplacian = float(item.get("blur_laplacian", 0.0) or 0.0)


def read_json(path: Path, default):
    """读取 JSON 文件；不存在时返回默认值。"""
    if not path.exists():
        return default
    return json.loads(path.read_text(encoding="utf-8"))


def build_event_lookup(events: list[dict]) -> dict[int, dict]:
    """把 weak/lost 事件展开成 sample_index -> event。"""
    lookup: dict[int, dict] = {}
    for event in events:
        start = int(event.get("start_sample", 0))
        end = int(event.get("end_sample", start))
        for sample_index in range(start, end + 1):
            lookup[sample_index] = event
    return lookup


def resolve_pointcloud(pointcloud: str, name: str, enhanced_dir: Path) -> Path | None:
    """解析点云路径。"""
    if pointcloud.lower() == "none":
        return None
    if pointcloud.lower() != "auto":
        path = Path(pointcloud)
        return path if path.is_absolute() else REPO / path

    candidates = [
        enhanced_dir / f"{name}.ply",
        enhanced_dir / "pointcloud.ply",
        REPO / "Opensource code" / "DPVO-main" / f"{name}.ply",
        REPO / "Opensource code" / "DPVO-main" / "result.ply",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def read_ply_points(path: Path, max_points: int) -> tuple[np.ndarray, np.ndarray] | None:
    """读取简单 PLY 点云，优先使用 plyfile，失败时退回到文本解析。"""
    if path is None or not path.exists():
        return None
    try:
        from plyfile import PlyData

        ply = PlyData.read(str(path))
        vertex = ply["vertex"]
        points = np.stack([vertex["x"], vertex["y"], vertex["z"]], axis=1).astype(np.float64)
        if {"red", "green", "blue"}.issubset(vertex.data.dtype.names or []):
            colors = np.stack([vertex["red"], vertex["green"], vertex["blue"]], axis=1).astype(np.float64) / 255.0
        else:
            colors = np.full_like(points, 0.7)
    except Exception:
        points, colors = read_ascii_ply_points(path)

    if points.size == 0:
        return None
    finite = np.all(np.isfinite(points), axis=1)
    points = points[finite]
    colors = colors[finite]
    if len(points) > max_points:
        rng = np.random.default_rng(7)
        indices = rng.choice(len(points), size=max_points, replace=False)
        points = points[indices]
        colors = colors[indices]
    return points, colors


def read_ascii_ply_points(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """读取 ASCII PLY 点云。"""
    lines = path.read_text(encoding="utf-8", errors="ignore").splitlines()
    vertex_count = 0
    header_end = 0
    properties: list[str] = []
    in_vertex = False
    for index, line in enumerate(lines):
        if line.startswith("element vertex"):
            vertex_count = int(line.split()[-1])
            in_vertex = True
        elif line.startswith("element ") and not line.startswith("element vertex"):
            in_vertex = False
        elif in_vertex and line.startswith("property"):
            properties.append(line.split()[-1])
        elif line.strip() == "end_header":
            header_end = index + 1
            break

    rows = []
    for line in lines[header_end: header_end + vertex_count]:
        parts = line.split()
        if len(parts) < 3:
            continue
        rows.append([float(item) for item in parts[: len(properties)]])
    if not rows:
        return np.empty((0, 3)), np.empty((0, 3))

    array = np.asarray(rows, dtype=np.float64)
    prop_to_index = {name: index for index, name in enumerate(properties)}
    points = np.stack(
        [
            array[:, prop_to_index.get("x", 0)],
            array[:, prop_to_index.get("y", 1)],
            array[:, prop_to_index.get("z", 2)],
        ],
        axis=1,
    )
    if {"red", "green", "blue"}.issubset(prop_to_index):
        colors = np.stack(
            [
                array[:, prop_to_index["red"]],
                array[:, prop_to_index["green"]],
                array[:, prop_to_index["blue"]],
            ],
            axis=1,
        ) / 255.0
    else:
        colors = np.full_like(points, 0.7)
    return points, colors


def state_color(state: str) -> str:
    """返回 matplotlib 颜色。"""
    if state == "good":
        return "#64d66f"
    if state == "weak":
        return "#ffb000"
    if state == "lost":
        return "#ff4d5e"
    return "#c8c8c8"


def compute_bounds(positions: np.ndarray, pointcloud: np.ndarray | None) -> tuple[np.ndarray, np.ndarray]:
    """计算三维显示范围。"""
    items = [positions]
    if pointcloud is not None and len(pointcloud):
        # 点云可能有少量离群点，使用分位数避免画面被拉得太远。
        low = np.percentile(pointcloud, 2, axis=0)
        high = np.percentile(pointcloud, 98, axis=0)
        clipped = pointcloud[np.all((pointcloud >= low) & (pointcloud <= high), axis=1)]
        if len(clipped):
            items.append(clipped)
    data = np.concatenate(items, axis=0)
    mins = data.min(axis=0)
    maxs = data.max(axis=0)
    center = (mins + maxs) / 2.0
    radius = float(np.max(maxs - mins)) * 0.58
    radius = max(radius, 0.5)
    return center - radius, center + radius


def set_axes_style(ax, lower: np.ndarray, upper: np.ndarray, frame_number: int) -> None:
    """设置三维坐标系风格。"""
    ax.set_xlim(lower[0], upper[0])
    ax.set_ylim(lower[1], upper[1])
    ax.set_zlim(lower[2], upper[2])
    ax.set_facecolor("#05070c")
    ax.grid(True, color="#222a33", linewidth=0.6)
    ax.xaxis.pane.set_facecolor((0.02, 0.03, 0.05, 0.9))
    ax.yaxis.pane.set_facecolor((0.02, 0.03, 0.05, 0.9))
    ax.zaxis.pane.set_facecolor((0.02, 0.03, 0.05, 0.9))
    ax.tick_params(colors="#9ba6b2", labelsize=7)
    ax.set_xlabel("x", color="#aab4c0")
    ax.set_ylabel("y", color="#aab4c0")
    ax.set_zlabel("z", color="#aab4c0")
    ax.view_init(elev=27 + 4 * math.sin(frame_number / 80.0), azim=-58 + frame_number * 0.12)


def draw_floor_grid(ax, lower: np.ndarray, upper: np.ndarray) -> None:
    """没有点云时画一个暗色地面网格，增强三维感。"""
    z = lower[2]
    xs = np.linspace(lower[0], upper[0], 9)
    ys = np.linspace(lower[1], upper[1], 9)
    for x in xs:
        ax.plot([x, x], [lower[1], upper[1]], [z, z], color="#17202a", linewidth=0.6)
    for y in ys:
        ax.plot([lower[0], upper[0]], [y, y], [z, z], color="#17202a", linewidth=0.6)


def draw_camera_frustum(
    ax,
    position: np.ndarray,
    quaternion_xyzw: np.ndarray,
    scale: float,
    color: str,
    alpha: float = 1.0,
    linewidth: float = 1.5,
) -> None:
    """画一个简化相机视锥。"""
    rotation = quaternion_to_rotation_matrix(quaternion_xyzw)
    corners = np.asarray(
        [
            [0.65, 0.42, 1.0],
            [-0.65, 0.42, 1.0],
            [-0.65, -0.42, 1.0],
            [0.65, -0.42, 1.0],
        ],
        dtype=np.float64,
    ) * scale
    corners_world = position[None, :] + corners @ rotation.T
    for corner in corners_world:
        ax.plot(
            [position[0], corner[0]],
            [position[1], corner[1]],
            [position[2], corner[2]],
            color=color,
            alpha=alpha,
            linewidth=linewidth,
        )
    closed = np.vstack([corners_world, corners_world[0]])
    ax.plot(closed[:, 0], closed[:, 1], closed[:, 2], color=color, alpha=alpha, linewidth=linewidth)


def draw_text(fig, x: float, y: float, text: str, color: str = "#edf2f7", size: int = 12) -> None:
    """在图像上画 HUD 文字。"""
    fig.text(x, y, text, color=color, fontsize=size, family="DejaVu Sans")


def yes_no(value: bool) -> str:
    """把布尔值转换成中文的是/否。"""
    return "是" if value else "否"


def format_alignment_label(alignment_info: dict) -> str:
    """生成适合写在图片上的显示坐标校正说明。"""
    mirror_label = "on" if bool(alignment_info.get("mirror_y")) else "off"
    return (
        f"align={alignment_info.get('mode')} | "
        f"first_turn={alignment_info.get('first_turn')} | "
        f"mirror_y={mirror_label}"
    )


def render_frame(
    fig,
    ax,
    rows: list[PoseRow],
    index: int,
    positions: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    keyframes: list[dict],
    event_lookup: dict[int, dict],
    relocalization_by_event: dict[int, dict],
    pointcloud: tuple[np.ndarray, np.ndarray] | None,
    frustum_scale: float,
) -> np.ndarray:
    """渲染单帧 3D 画面并返回 BGR 图像。"""
    ax.clear()
    fig.texts.clear()
    fig.patch.set_facecolor("#05070c")
    set_axes_style(ax, lower, upper, index)

    row = rows[index]
    if pointcloud is not None:
        points, colors = pointcloud
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=colors, s=0.25, alpha=0.18, linewidths=0)
    else:
        draw_floor_grid(ax, lower, upper)

    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], color="#26313d", linewidth=1.2, alpha=0.8)
    ax.plot(positions[: index + 1, 0], positions[: index + 1, 1], positions[: index + 1, 2],
            color="#4fd1c5", linewidth=2.4, alpha=0.95)

    # 用稀疏散点显示历史质量状态，避免三维画面过于拥挤。
    history_step = max(1, len(rows) // 420)
    history_indices = np.arange(0, index + 1, history_step)
    if len(history_indices):
        ax.scatter(
            positions[history_indices, 0],
            positions[history_indices, 1],
            positions[history_indices, 2],
            c=[state_color(rows[int(i)].quality_state) for i in history_indices],
            s=10,
            alpha=0.9,
            linewidths=0,
        )

    keyframe_samples = {int(keyframe["sample_index"]): keyframe for keyframe in keyframes}
    row_by_sample = {item.sample_index: item for item in rows}
    keyframe_positions = []
    for keyframe in keyframes:
        key_row = row_by_sample.get(int(keyframe["sample_index"]))
        if key_row is not None:
            keyframe_positions.append(key_row.position)
    if keyframe_positions:
        keyframe_positions_array = np.asarray(keyframe_positions)
        ax.scatter(
            keyframe_positions_array[:, 0],
            keyframe_positions_array[:, 1],
            keyframe_positions_array[:, 2],
            marker="^",
            s=20,
            color="#f56565",
            alpha=0.9,
            label="keyframes",
        )

    # 当前相机和少量关键帧相机视锥。
    for keyframe in keyframes[:: max(1, len(keyframes) // 18)]:
        key_row = row_by_sample.get(int(keyframe["sample_index"]))
        if key_row is not None:
            draw_camera_frustum(ax, key_row.position, key_row.quaternion_xyzw,
                                frustum_scale * 0.55, "#805ad5", alpha=0.25, linewidth=0.8)
    draw_camera_frustum(ax, row.position, row.quaternion_xyzw,
                        frustum_scale, state_color(row.quality_state), alpha=1.0, linewidth=2.2)
    ax.scatter([row.position[0]], [row.position[1]], [row.position[2]],
               s=90, color=state_color(row.quality_state), edgecolors="#ffffff", linewidths=1.2)

    event = event_lookup.get(row.sample_index)
    relocalization = None
    best = None
    if event is not None:
        relocalization = relocalization_by_event.get(int(event["event_id"]))
        best = (relocalization or {}).get("best_candidate")
    if best:
        keyframe = keyframe_samples.get(int(best.get("keyframe_sample_index", -1)))
        key_row = row_by_sample.get(int(best.get("keyframe_sample_index", -1)))
        if keyframe is not None and key_row is not None:
            line_color = "#68d391" if (relocalization or {}).get("verified") else "#fc8181"
            ax.plot(
                [row.position[0], key_row.position[0]],
                [row.position[1], key_row.position[1]],
                [row.position[2], key_row.position[2]],
                color=line_color,
                linewidth=2.0,
                linestyle="--",
                alpha=0.95,
            )

    draw_text(fig, 0.035, 0.94, "DPVO Enhanced 3D Tracking Replay", "#f7fafc", 16)
    draw_text(fig, 0.035, 0.90, f"sample {row.sample_index} | frame {row.source_frame} | time {row.source_time_sec:.2f}s",
              "#cbd5e0", 11)
    draw_text(fig, 0.035, 0.865, f"state {row.quality_state} | quality {row.quality_score:.3f} | mask {row.mask_ratio:.3f}",
              state_color(row.quality_state), 12)
    draw_text(fig, 0.035, 0.83, f"keyframes {len(keyframes)} | trajectory samples {len(rows)}",
              "#a0aec0", 10)

    if relocalization:
        verified_text = "verified" if relocalization.get("verified") else "not verified"
        status = relocalization.get("best_status", "n/a")
        inliers = (best or {}).get("essential_inliers", "n/a")
        ratio = (best or {}).get("essential_inlier_ratio", "n/a")
        if isinstance(ratio, float):
            ratio = f"{ratio:.3f}"
        draw_text(fig, 0.68, 0.94, f"event {relocalization.get('event_id')} | {verified_text}",
                  "#68d391" if relocalization.get("verified") else "#fc8181", 12)
        draw_text(fig, 0.68, 0.905, f"status: {status}", "#e2e8f0", 10)
        draw_text(fig, 0.68, 0.875, f"inliers: {inliers} | ratio: {ratio}", "#e2e8f0", 10)
    else:
        draw_text(fig, 0.68, 0.94, "tracking: stable/no event", "#68d391", 12)

    pointcloud_note = "point cloud: loaded" if pointcloud is not None else "point cloud: none, trajectory-only 3D"
    draw_text(fig, 0.68, 0.06, pointcloud_note, "#718096", 9)
    draw_text(fig, 0.035, 0.06, "Green=good  Yellow=weak  Red=lost  Purple=keyframe cameras", "#718096", 9)

    fig.canvas.draw()
    rgb = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


def save_route_plots(
    output_dir: Path,
    rows: list[PoseRow],
    positions: np.ndarray,
    keyframes: list[dict],
    alignment_info: dict,
) -> dict:
    """保存静态 3D 路线图和俯视路线图。"""
    route_3d_path = output_dir / "dpvo_3d_route.png"
    top_path = output_dir / "dpvo_top_route.png"
    row_by_sample = {row.sample_index: row for row in rows}
    keyframe_positions = []
    for keyframe in keyframes:
        row = row_by_sample.get(int(keyframe.get("sample_index", -1)))
        if row is not None:
            keyframe_positions.append(row.position)
    keyframe_positions = np.asarray(keyframe_positions, dtype=np.float64) if keyframe_positions else np.empty((0, 3))

    fig = plt.figure(figsize=(12.8, 7.2), dpi=140)
    ax = fig.add_subplot(111, projection="3d")
    fig.patch.set_facecolor("#05070c")
    set_axes_style(ax, *compute_bounds(positions, None), frame_number=0)
    draw_floor_grid(ax, *compute_bounds(positions, None))
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], color="#4fd1c5", linewidth=2.4)
    ax.scatter([positions[0, 0]], [positions[0, 1]], [positions[0, 2]], color="#68d391", s=70, label="start")
    ax.scatter([positions[-1, 0]], [positions[-1, 1]], [positions[-1, 2]], color="#fc8181", s=70, label="end")
    if len(keyframe_positions):
        ax.scatter(keyframe_positions[:, 0], keyframe_positions[:, 1], keyframe_positions[:, 2],
                   marker="^", color="#805ad5", s=18, alpha=0.9, label="keyframes")
    ax.set_title(
        f"DPVO 3D Route | {format_alignment_label(alignment_info)}",
        color="#edf2f7",
        pad=18,
    )
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(route_3d_path, facecolor=fig.get_facecolor(), dpi=160)
    plt.close(fig)

    fig2, ax2 = plt.subplots(figsize=(12.8, 7.2), dpi=140)
    fig2.patch.set_facecolor("#05070c")
    ax2.set_facecolor("#05070c")
    ax2.plot(positions[:, 0], positions[:, 1], color="#4fd1c5", linewidth=2.2)
    ax2.scatter([positions[0, 0]], [positions[0, 1]], color="#68d391", s=70, label="start", zorder=4)
    ax2.scatter([positions[-1, 0]], [positions[-1, 1]], color="#fc8181", s=70, label="end", zorder=4)
    if len(keyframe_positions):
        ax2.scatter(keyframe_positions[:, 0], keyframe_positions[:, 1],
                    marker="^", color="#805ad5", s=14, alpha=0.75, label="keyframes", zorder=3)
    # 画几个方向箭头，帮助从上方判断第一段是左转还是右转。
    arrow_indices = np.linspace(0, len(positions) - 2, num=10, dtype=np.int32)
    for idx in arrow_indices:
        step = positions[min(idx + 8, len(positions) - 1), :2] - positions[idx, :2]
        if np.linalg.norm(step) > 1e-9:
            ax2.arrow(
                positions[idx, 0],
                positions[idx, 1],
                step[0] * 0.6,
                step[1] * 0.6,
                color="#f6ad55",
                width=0.01,
                head_width=0.08,
                alpha=0.75,
                length_includes_head=True,
            )
    ax2.set_aspect("equal", adjustable="box")
    ax2.grid(True, color="#222a33", linewidth=0.6)
    ax2.tick_params(colors="#9ba6b2", labelsize=9)
    ax2.set_xlabel("aligned x / forward", color="#aab4c0")
    ax2.set_ylabel("aligned y / left-right", color="#aab4c0")
    ax2.set_title(
        "DPVO Top Route Check: x=forward, y=left/right display axis",
        color="#edf2f7",
        pad=14,
    )
    ax2.text(
        0.02,
        0.02,
        f"{format_alignment_label(alignment_info)} | "
        f"turn_cross_before={float(alignment_info.get('first_turn_cross_before', 0.0)):.6f} | "
        f"after={float(alignment_info.get('first_turn_cross_after', 0.0)):.6f}",
        color="#cbd5e0",
        transform=ax2.transAxes,
    )
    ax2.legend(loc="upper right")
    fig2.tight_layout()
    fig2.savefig(top_path, facecolor=fig2.get_facecolor(), dpi=160)
    plt.close(fig2)

    return {
        "route_3d_png": str(route_3d_path),
        "top_route_png": str(top_path),
    }


def select_frame_indices(total: int, every: int) -> list[int]:
    """选择要渲染的视频帧索引。"""
    indices = list(range(0, total, max(1, every)))
    if indices[-1] != total - 1:
        indices.append(total - 1)
    return indices


def select_snapshot_indices(frame_indices: list[int], count: int) -> set[int]:
    """选择要保存截图的索引。"""
    if count <= 0:
        return set()
    if len(frame_indices) <= count:
        return set(frame_indices)
    selected = np.linspace(0, len(frame_indices) - 1, count, dtype=np.int32)
    return {frame_indices[int(index)] for index in selected}


def main() -> None:
    """脚本入口。"""
    args = parse_args()
    enhanced_dir = Path(args.enhanced_dir)
    output_dir = Path(args.output_dir) if args.output_dir else enhanced_dir / "visualization_3d"
    snapshot_dir = output_dir / "snapshots"
    output_dir.mkdir(parents=True, exist_ok=True)
    snapshot_dir.mkdir(parents=True, exist_ok=True)

    rows = read_tum_trajectory(Path(args.trajectory))
    merge_quality(rows, enhanced_dir)

    keyframes = read_json(enhanced_dir / "keyframes" / "keyframes.json", [])
    events = read_json(enhanced_dir / "tracking_events.json", [])
    relocalization_results = read_json(enhanced_dir / "relocalization_results.json", [])
    event_lookup = build_event_lookup(events)
    relocalization_by_event = {int(item["event_id"]): item for item in relocalization_results}

    pointcloud_path = resolve_pointcloud(args.pointcloud, args.name, enhanced_dir)
    pointcloud = read_ply_points(pointcloud_path, max_points=max(1, args.max_points)) if pointcloud_path else None
    positions, pointcloud, alignment_info = apply_world_alignment(
        rows,
        pointcloud,
        args.align_world,
        args.first_turn,
        args.mirror_y,
    )
    pointcloud_points = pointcloud[0] if pointcloud is not None else None

    lower, upper = compute_bounds(positions, pointcloud_points)
    frustum_scale = max(0.03, float(np.max(upper - lower)) * 0.035)

    width = max(640, int(args.width))
    height = max(480, int(args.height))
    fps = max(1.0, float(args.fps))
    frame_indices = select_frame_indices(len(rows), args.every)
    snapshot_indices = select_snapshot_indices(frame_indices, args.snapshot_count)
    route_plot_stats = save_route_plots(output_dir, rows, positions, keyframes, alignment_info)

    video_path = output_dir / "dpvo_enhanced_3d.mp4"
    writer = cv2.VideoWriter(
        str(video_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (width, height),
    )
    if not writer.isOpened():
        raise RuntimeError(f"无法创建 3D 可视化视频: {video_path}")

    show_enabled = bool(args.show)
    if show_enabled and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        print("未检测到 DISPLAY/WAYLAND_DISPLAY，实时 3D 窗口已关闭；仍会保存 MP4。")
        show_enabled = False

    fig = plt.figure(figsize=(width / 100.0, height / 100.0), dpi=100)
    ax = fig.add_subplot(111, projection="3d")
    fig.subplots_adjust(left=0.02, right=0.98, top=0.98, bottom=0.02)

    written = 0
    snapshots = 0
    try:
        for index in frame_indices:
            frame = render_frame(
                fig,
                ax,
                rows,
                index,
                positions,
                lower,
                upper,
                keyframes,
                event_lookup,
                relocalization_by_event,
                pointcloud,
                frustum_scale,
            )
            if frame.shape[1] != width or frame.shape[0] != height:
                frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
            writer.write(frame)
            written += 1

            if index in snapshot_indices:
                row = rows[index]
                snapshot_path = snapshot_dir / f"sample_{row.sample_index:06d}_3d.jpg"
                if cv2.imwrite(str(snapshot_path), frame):
                    snapshots += 1

            if show_enabled:
                cv2.imshow("DPVO Enhanced 3D Viewer", frame)
                key = cv2.waitKey(max(1, int(1000 / fps))) & 0xFF
                if key in (27, ord("q")):
                    break
    finally:
        writer.release()
        plt.close(fig)
        if show_enabled:
            cv2.destroyWindow("DPVO Enhanced 3D Viewer")

    summary = {
        "output_dir": str(output_dir),
        "video": str(video_path),
        "snapshots_dir": str(snapshot_dir),
        "frames_written": written,
        "fps": fps,
        "width": width,
        "height": height,
        "trajectory_samples": len(rows),
        "render_every": max(1, args.every),
        "keyframes": len(keyframes),
        "events": len(events),
        "pointcloud_path": str(pointcloud_path) if pointcloud_path else "",
        "pointcloud_loaded": pointcloud is not None,
        "pointcloud_points": int(len(pointcloud[0])) if pointcloud is not None else 0,
        "world_alignment": alignment_info,
        "route_plots": route_plot_stats,
        "note": "这是 DPVO 增强结果的三维可视化回放；没有点云时显示轨迹/相机/关键帧，不等同于完整三维场景重建。",
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    lines = [
        "DPVO 3D 增强可视化完成",
        "=======================",
        f"输出目录: {output_dir}",
        f"3D 视频: {video_path}",
        f"截图目录: {snapshot_dir}",
        f"写入帧数: {written}",
        f"输出帧率: {fps}",
        f"关键帧数量: {len(keyframes)}",
        f"质量事件数量: {len(events)}",
        f"点云文件: {pointcloud_path if pointcloud_path else '未使用'}",
        f"点云是否加载: {pointcloud is not None}",
        f"世界坐标对齐: {alignment_info.get('mode')}",
        f"首个转弯校正: {alignment_info.get('first_turn')}",
        f"左右镜像显示校正: {yes_no(bool(alignment_info.get('mirror_y')))}",
        f"转向估计值: 校正前 {float(alignment_info.get('first_turn_cross_before', 0.0)):.6f}, "
        f"校正后 {float(alignment_info.get('first_turn_cross_after', 0.0)):.6f}",
        f"3D 路线图: {route_plot_stats.get('route_3d_png')}",
        f"俯视路线图: {route_plot_stats.get('top_route_png')}",
        "",
        "说明：当前结果保留二维增强视频，同时新增这个 3D 回放。",
        "默认会用 trajectory_pca 把主要运动平面校正成水平，避免把前进方向误画成向上。",
        "左右镜像只影响显示坐标，不会修改 DPVO 原始轨迹文件；它用于在没有真值地图/IMU 时按实际路线校正左转和右转方向。",
        "如果要接近作者 live demo 中的场景重建效果，需要额外保存/加载 DPVO PLY 或 VGGT/3DGS 点云。",
    ]
    (output_dir / "summary.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"DPVO 3D 增强可视化完成，输出目录: {output_dir}")


if __name__ == "__main__":
    main()
