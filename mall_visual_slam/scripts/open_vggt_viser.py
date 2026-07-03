#!/usr/bin/env python3
"""
打开 VGGT 推理结果的 3D 可视化页面。

这个脚本读取前面已经生成好的 VGGT 结果目录：
1. predictions.npz
2. images/
3. summary.json

然后启动一个 Viser 服务器，在浏览器中显示：
1. 点云
2. 相机位姿
3. 当前窗口对应的图像平面

这样看到的效果会更接近论文里的 3D 演示，而不是单纯的深度图预览。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import viser
import viser.transforms as viser_tf


REPO = Path("/home/ros/ros2_orbslam3")
DEFAULT_RESULT_DIR = REPO / "output" / "vggt_input_video_show"
DEFAULT_FULL_VIEWER_DIR = DEFAULT_RESULT_DIR / "full_viewer"


def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(description="Open a Viser 3D viewer for VGGT outputs")
    parser.add_argument("--result-dir", default=str(DEFAULT_RESULT_DIR), help="VGGT 输出目录")
    parser.add_argument("--port", type=int, default=8080, help="Viser 服务端口")
    parser.add_argument("--show-depth", action="store_true", help="在相机平面里显示深度图而不是原图")
    parser.add_argument("--max-points", type=int, default=100000, help="最多显示多少个点")
    parser.add_argument("--conf-percentile", type=float, default=85.0, help="保留的置信度百分位")
    parser.add_argument(
        "--aggregate-windows",
        action="store_true",
        help=(
            "聚合 windows/window_xxxx/vggt_points.ply 显示完整视频的所有窗口。"
            "注意：这种模式只是把局部坐标按时间排开，并不是真正的全局重建；"
            "想看真实 3D 场景请改用 --aligned。"
        ),
    )
    parser.add_argument(
        "--aligned",
        action="store_true",
        help=(
            "使用 scripts/aggregate_vggt_aligned.py 产出的全局对齐点云。"
            "这是默认推荐的看 3D 场景的方式：所有窗口已经统一到 window-0 的坐标系，"
            "并按置信度/边缘/voxel 过滤，渲染出来是一张干净点云。"
        ),
    )
    parser.add_argument(
        "--aligned-subdir",
        default="aligned_full",
        help="对齐结果在 result-dir 下的子目录名",
    )
    parser.add_argument(
        "--points-per-window",
        type=int,
        default=5000,
        help="聚合模式下每个窗口最多保留多少个点",
    )
    parser.add_argument(
        "--layout",
        choices=["timeline", "overlay"],
        default="timeline",
        help="聚合窗口的摆放方式：timeline 按时间排开，overlay 直接叠放",
    )
    parser.add_argument(
        "--full-output-dir",
        default=str(DEFAULT_FULL_VIEWER_DIR),
        help="完整聚合结果的保存目录",
    )
    parser.add_argument(
        "--image-planes",
        action="store_true",
        help="聚合模式下把抽样原图作为 3D 图像面板放到场景里，更接近论文/官方 demo 的展示风格",
    )
    parser.add_argument(
        "--image-every",
        type=int,
        default=8,
        help="聚合模式下每隔多少张抽样帧显示一张原图面板",
    )
    parser.add_argument(
        "--image-scale",
        type=float,
        default=0.55,
        help="聚合模式下原图面板的显示高度",
    )
    parser.add_argument(
        "--image-y",
        type=float,
        default=-1.25,
        help="聚合模式下原图面板在 Y 轴方向的位置",
    )
    parser.add_argument(
        "--image-z",
        type=float,
        default=0.55,
        help="聚合模式下原图面板在 Z 轴方向的位置",
    )
    return parser.parse_args()


def load_predictions(result_dir: Path) -> dict:
    """读取 VGGT 推理结果。"""
    pred_path = result_dir / "predictions.npz"
    if not pred_path.exists():
        raise FileNotFoundError(f"找不到预测文件: {pred_path}")

    with np.load(pred_path, allow_pickle=False) as data:
        predictions = {key: data[key] for key in data.files}
    return predictions


def camera_centers_from_extrinsic(extrinsic: np.ndarray) -> np.ndarray:
    """由 world-to-camera 外参计算相机中心。"""
    centers = []
    for pose in extrinsic:
        rotation = pose[:, :3]
        translation = pose[:, 3]
        centers.append(-rotation.T @ translation)
    return np.asarray(centers, dtype=np.float32)


def read_ascii_ply(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """读取本项目导出的 ASCII PLY 点云。"""
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
        raise ValueError(f"无法读取 PLY 顶点数量: {path}")
    if vertex_count == 0:
        return np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.uint8)

    data = np.loadtxt(path, skiprows=header_lines, max_rows=vertex_count, dtype=np.float32)
    if data.ndim == 1:
        data = data[None, :]
    points = data[:, :3].astype(np.float32)
    colors = np.clip(data[:, 3:6], 0, 255).astype(np.uint8)
    return points, colors


def resolve_repo_path(path_text: str) -> Path:
    """把 JSON 里的相对路径解析成项目中的绝对路径。"""
    path = Path(path_text)
    if path.is_absolute():
        return path
    return REPO / path


def load_frame_meta(result_dir: Path) -> list[dict]:
    """读取抽帧元数据，用于在 3D 场景中放置原始图像面板。"""
    meta_path = result_dir / "frame_meta.json"
    if not meta_path.exists():
        return []
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    frames = meta.get("frames", [])
    return frames if isinstance(frames, list) else []


def select_image_frames(frames: list[dict], every: int) -> list[dict]:
    """从所有抽样帧里挑一部分显示，避免一次性贴太多图导致网页卡顿。"""
    if not frames:
        return []

    every = max(1, every)
    selected = [frame for index, frame in enumerate(frames) if index % every == 0]

    # 总是保留最后一帧，方便知道视频终点。
    if selected[-1].get("sample_index") != frames[-1].get("sample_index"):
        selected.append(frames[-1])
    return selected


def read_rgb_image(path: Path, max_width: int = 480) -> np.ndarray | None:
    """读取图片并转成 RGB，必要时缩小，降低浏览器传输和渲染压力。"""
    image_bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image_bgr is None:
        return None

    height, width = image_bgr.shape[:2]
    if width > max_width:
        scale = max_width / float(width)
        image_bgr = cv2.resize(
            image_bgr,
            (max_width, max(1, int(round(height * scale)))),
            interpolation=cv2.INTER_AREA,
        )

    return cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)


def save_ascii_ply(points: np.ndarray, colors: np.ndarray, path: Path) -> None:
    """保存聚合后的 ASCII PLY 点云。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for point, color in zip(points, colors):
            f.write(
                f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} "
                f"{int(color[0])} {int(color[1])} {int(color[2])}\n"
            )


def aggregate_window_point_clouds(
    result_dir: Path,
    output_dir: Path,
    points_per_window: int,
    max_points: int,
    layout: str,
) -> dict:
    """聚合所有窗口的 PLY 点云，并保存完整可视化结果。"""
    window_dirs = sorted((result_dir / "windows").glob("window_*"))
    ply_paths = [window_dir / "vggt_points.ply" for window_dir in window_dirs if (window_dir / "vggt_points.ply").exists()]
    if not ply_paths:
        raise FileNotFoundError(f"没有找到窗口点云: {result_dir / 'windows'}")

    rng = np.random.default_rng(42)
    all_points = []
    all_colors = []
    window_meta = []
    timeline_spacing = 1.0

    for window_index, ply_path in enumerate(ply_paths):
        points, colors = read_ascii_ply(ply_path)
        if len(points) == 0:
            continue

        if points_per_window > 0 and len(points) > points_per_window:
            selected = rng.choice(len(points), size=points_per_window, replace=False)
            points = points[selected]
            colors = colors[selected]

        finite = np.isfinite(points).all(axis=1)
        points = points[finite]
        colors = colors[finite]
        if len(points) == 0:
            continue

        center = np.mean(points, axis=0)
        points = points - center

        if layout == "timeline":
            points[:, 0] += window_index * timeline_spacing

        all_points.append(points)
        all_colors.append(colors)
        window_meta.append(
            {
                "window_index": window_index,
                "source_ply": str(ply_path),
                "kept_points": int(len(points)),
                "layout_offset_x": float(window_index * timeline_spacing if layout == "timeline" else 0.0),
            }
        )

    if not all_points:
        raise RuntimeError("所有窗口点云都为空，无法聚合。")

    points = np.concatenate(all_points, axis=0)
    colors = np.concatenate(all_colors, axis=0)

    if max_points > 0 and len(points) > max_points:
        selected = rng.choice(len(points), size=max_points, replace=False)
        points = points[selected]
        colors = colors[selected]

    output_dir.mkdir(parents=True, exist_ok=True)
    ply_path = output_dir / "vggt_full_windows_aggregated.ply"
    meta_path = output_dir / "vggt_full_windows_aggregated.json"
    save_ascii_ply(points, colors, ply_path)

    meta = {
        "result_dir": str(result_dir),
        "layout": layout,
        "num_windows_found": len(ply_paths),
        "num_windows_used": len(window_meta),
        "points_per_window": points_per_window,
        "max_points": max_points,
        "aggregated_points": int(len(points)),
        "aggregated_ply": str(ply_path),
        "window_meta": window_meta,
        "note": "VGGT 每个窗口是局部坐标，本文件是完整窗口聚合可视化，不等同于严格全局一致地图。",
    }
    meta_path.write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
    return meta


def add_paper_style_image_planes(
    server: viser.ViserServer,
    result_dir: Path,
    args: argparse.Namespace,
) -> tuple[list, list]:
    """在完整聚合点云旁边添加原图面板，让场景更接近论文/官方 demo 的展示风格。"""
    frames_meta = load_frame_meta(result_dir)
    selected_frames = select_image_frames(frames_meta, args.image_every)
    image_handles = []
    label_handles = []

    if not selected_frames:
        print("没有找到 frame_meta.json 中的抽样帧信息，跳过图像面板。")
        return image_handles, label_handles

    # add_image 的本地平面默认在 XY 面。绕 X 轴旋转 90 度后，
    # 图像会成为竖直面板，并朝向 -Y，便于和时间轴点云一起观察。
    vertical_wxyz = np.array([0.70710678, 0.70710678, 0.0, 0.0], dtype=np.float32)

    for frame in selected_frames:
        sample_index = int(frame.get("sample_index", 0))
        source_frame = int(frame.get("source_frame", 0))
        source_time_sec = float(frame.get("source_time_sec", 0.0))
        image_path = resolve_repo_path(str(frame.get("image_path", "")))
        image_rgb = read_rgb_image(image_path)
        if image_rgb is None:
            print(f"跳过无法读取的图像: {image_path}")
            continue

        height, width = image_rgb.shape[:2]
        render_height = float(args.image_scale)
        render_width = render_height * width / max(float(height), 1.0)

        # timeline 模式下点云大致按 window_index 放在 x 轴，sample_index 与其对应。
        position = np.array([float(sample_index), float(args.image_y), float(args.image_z)], dtype=np.float32)

        image_handle = server.scene.add_image(
            name=f"paper_style_images/frame_{sample_index:05d}",
            image=image_rgb,
            render_width=render_width,
            render_height=render_height,
            format="jpeg",
            jpeg_quality=85,
            wxyz=vertical_wxyz,
            position=position,
            visible=True,
        )
        label_handle = server.scene.add_label(
            name=f"paper_style_images/frame_{sample_index:05d}/label",
            text=f"t={source_time_sec:.1f}s  frame={source_frame}",
            position=position + np.array([0.0, -0.02, render_height * 0.62], dtype=np.float32),
            visible=True,
        )
        image_handles.append(image_handle)
        label_handles.append(label_handle)

    print(f"已添加论文风格原图面板: {len(image_handles)} 张")
    return image_handles, label_handles


def normalize_depth(depth: np.ndarray) -> np.ndarray:
    """把深度归一化成 0 到 1 方便显示。"""
    valid = np.isfinite(depth) & (depth > 0)
    if not np.any(valid):
        return np.zeros_like(depth, dtype=np.float32)
    lo, hi = np.percentile(depth[valid], [2, 98])
    return np.clip((depth - lo) / max(float(hi - lo), 1e-6), 0, 1).astype(np.float32)


def make_camera_image(predictions: dict, show_depth: bool) -> np.ndarray:
    """生成给相机锥体使用的 RGB 图像。"""
    if show_depth:
        depth = predictions["depth"][0, ..., 0]
        depth_norm = normalize_depth(depth)
        img = cv2.applyColorMap((depth_norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return img

    images = predictions["images"]
    if images.ndim == 4 and images.shape[1] == 3:
        image = np.transpose(images[0], (1, 2, 0))
    else:
        image = images[0]
    # VGGT 保存的 images 本来就是 RGB，这里不要再做 BGR->RGB 交换。
    return np.clip(image * 255.0, 0, 255).astype(np.uint8)


def build_scene(predictions: dict, result_dir: Path, args: argparse.Namespace) -> None:
    """把 VGGT 结果转成 Viser 里的 3D 场景。"""
    print(f"Starting viser server on port {args.port}")
    server = viser.ViserServer(host="0.0.0.0", port=args.port)
    server.gui.configure_theme(titlebar_content=None, control_layout="collapsible")

    if "world_points_from_depth" in predictions:
        points = predictions["world_points_from_depth"]
    elif "world_points" in predictions:
        points = predictions["world_points"]
    else:
        raise ValueError("预测结果里没有 world_points_from_depth 或 world_points")

    conf = predictions.get("depth_conf")
    if conf is None:
        conf = predictions.get("world_points_conf")
    if conf is None:
        conf = np.ones(points.shape[:-1], dtype=np.float32)

    images = predictions["images"]
    extrinsic = predictions["extrinsic"]
    intrinsic = predictions["intrinsic"]

    points_flat = points.reshape(-1, 3)
    colors = images.transpose(0, 2, 3, 1).reshape(-1, 3)
    colors = np.clip(colors * 255.0, 0, 255).astype(np.uint8)
    conf_flat = conf.reshape(-1)

    finite = np.isfinite(points_flat).all(axis=1) & np.isfinite(conf_flat)
    threshold = np.percentile(conf_flat[finite], args.conf_percentile)
    keep = finite & (conf_flat >= threshold)
    keep_indices = np.flatnonzero(keep)
    if len(keep_indices) > args.max_points:
        rng = np.random.default_rng(42)
        keep_indices = rng.choice(keep_indices, size=args.max_points, replace=False)

    points_keep = points_flat[keep_indices]
    colors_keep = colors[keep_indices]

    scene_center = np.mean(points_keep, axis=0)
    points_keep = points_keep - scene_center

    point_cloud = server.scene.add_point_cloud(
        name="vggt_point_cloud",
        points=points_keep,
        colors=colors_keep,
        point_size=0.003,
        point_shape="circle",
    )

    cam_img = make_camera_image(predictions, args.show_depth)
    cam_img_h, cam_img_w = cam_img.shape[:2]
    fx = float(intrinsic[0, 0, 0])
    fy = float(intrinsic[0, 1, 1])
    fov_y = 2.0 * np.arctan2(cam_img_h / 2.0, fy)
    aspect = cam_img_w / cam_img_h

    frames = []
    frustums = []
    for idx in range(len(extrinsic)):
        pose = extrinsic[idx]
        world_to_camera = np.eye(4, dtype=np.float32)
        world_to_camera[:3, :4] = pose
        camera_to_world = np.linalg.inv(world_to_camera)
        camera_to_world[:3, 3] -= scene_center
        se3 = viser_tf.SE3.from_matrix(camera_to_world[:3, :])

        frame = server.scene.add_frame(
            f"camera_{idx}",
            wxyz=se3.rotation().wxyz,
            position=se3.translation(),
            axes_length=0.05,
            axes_radius=0.002,
            origin_radius=0.002,
        )
        frames.append(frame)

        frustum = server.scene.add_camera_frustum(
            f"camera_{idx}/frustum",
            fov=fov_y,
            aspect=aspect,
            scale=0.06,
            image=cam_img,
            line_width=1.0,
        )
        frustums.append(frustum)

        @frustum.on_click
        def _(_event, frame=frame):  # noqa: B023
            for client in server.get_clients().values():
                client.camera.wxyz = frame.wxyz
                client.camera.position = frame.position

    show_cameras = server.gui.add_checkbox("显示相机", initial_value=True)
    show_point_cloud = server.gui.add_checkbox("显示点云", initial_value=True)
    frame_selector = server.gui.add_dropdown(
        "查看相机",
        options=["All"] + [str(i) for i in range(len(frames))],
        initial_value="All",
    )

    @show_cameras.on_update
    def _(_event) -> None:
        for frame in frames:
            frame.visible = show_cameras.value
        for frustum in frustums:
            frustum.visible = show_cameras.value

    @show_point_cloud.on_update
    def _(_event) -> None:
        point_cloud.visible = show_point_cloud.value

    @frame_selector.on_update
    def _(_event) -> None:
        if frame_selector.value == "All":
            for i, frame in enumerate(frames):
                frame.visible = show_cameras.value
                frustums[i].visible = show_cameras.value
        else:
            selected = int(frame_selector.value)
            for i, frame in enumerate(frames):
                visible = show_cameras.value and i == selected
                frame.visible = visible
                frustums[i].visible = visible

    print(f"Viser 3D viewer 已启动。请在浏览器打开: http://localhost:{args.port}")
    print("如果你在 Windows 浏览器里打开，也通常可以直接访问同一个 localhost 地址。")


def build_aggregated_scene(result_dir: Path, args: argparse.Namespace) -> None:
    """显示完整窗口聚合点云。"""
    output_dir = Path(args.full_output_dir)
    meta = aggregate_window_point_clouds(
        result_dir=result_dir,
        output_dir=output_dir,
        points_per_window=args.points_per_window,
        max_points=args.max_points,
        layout=args.layout,
    )

    points, colors = read_ascii_ply(Path(meta["aggregated_ply"]))
    scene_center = np.mean(points, axis=0)
    points = points - scene_center

    print(f"Starting viser server on port {args.port}")
    server = viser.ViserServer(host="0.0.0.0", port=args.port)
    server.gui.configure_theme(titlebar_content=None, control_layout="collapsible")

    point_cloud = server.scene.add_point_cloud(
        name="full_window_aggregated_point_cloud",
        points=points,
        colors=colors,
        point_size=0.004,
        point_shape="circle",
    )

    image_handles = []
    label_handles = []
    if args.image_planes:
        image_handles, label_handles = add_paper_style_image_planes(server, result_dir, args)

    show_point_cloud = server.gui.add_checkbox("显示完整窗口点云", initial_value=True)
    show_image_planes = server.gui.add_checkbox("显示原图面板", initial_value=bool(image_handles))
    show_image_labels = server.gui.add_checkbox("显示图像标签", initial_value=bool(label_handles))

    @show_point_cloud.on_update
    def _(_event) -> None:
        point_cloud.visible = show_point_cloud.value

    @show_image_planes.on_update
    def _(_event) -> None:
        for handle in image_handles:
            handle.visible = show_image_planes.value

    @show_image_labels.on_update
    def _(_event) -> None:
        for handle in label_handles:
            handle.visible = show_image_labels.value

    print(f"完整窗口聚合点云已保存: {meta['aggregated_ply']}")
    print(f"完整窗口聚合摘要已保存: {output_dir / 'vggt_full_windows_aggregated.json'}")
    print(f"聚合窗口数: {meta['num_windows_used']}，聚合点数: {meta['aggregated_points']}")
    if image_handles:
        print("当前 viewer 已启用论文风格原图面板，可在左侧 GUI 中开关显示。")
    print(f"Viser 完整 3D viewer 已启动。请在浏览器打开: http://localhost:{args.port}")
    print("说明：这是所有窗口的聚合可视化；VGGT 分窗口输出不是严格全局一致地图。")


def build_aligned_scene(result_dir: Path, args: argparse.Namespace) -> None:
    """显示 aggregate_vggt_aligned.py 输出的全局对齐点云 + 相机轨迹。"""
    aligned_dir = result_dir / args.aligned_subdir
    aligned_ply = aligned_dir / "aligned_full_scene.ply"
    aligned_traj = aligned_dir / "aligned_camera_trajectory.json"
    aligned_summary_path = aligned_dir / "aligned_full_summary.json"

    if not aligned_ply.exists():
        raise FileNotFoundError(
            f"找不到对齐点云: {aligned_ply}\n"
            "请先运行: python scripts/aggregate_vggt_aligned.py "
            f"--result-dir {result_dir}\n"
            "或重跑 scripts/run_vggt_video.py（新版会在末尾自动调用聚合）。"
        )

    points, colors = read_ascii_ply(aligned_ply)
    if len(points) == 0:
        raise RuntimeError(f"对齐点云为空: {aligned_ply}")

    if aligned_summary_path.exists():
        summary = json.loads(aligned_summary_path.read_text(encoding="utf-8"))
        print(
            f"对齐点云: {summary.get('aggregated_points', len(points))} 点，"
            f"基于 {summary.get('num_aligned_windows', '?')} 个对齐窗口。"
        )

    scene_center = np.mean(points, axis=0)
    points = points - scene_center

    print(f"Starting viser server on port {args.port}")
    server = viser.ViserServer(host="0.0.0.0", port=args.port)
    server.gui.configure_theme(titlebar_content=None, control_layout="collapsible")

    point_cloud = server.scene.add_point_cloud(
        name="aligned_full_scene",
        points=points,
        colors=colors,
        point_size=0.005,
        point_shape="circle",
    )

    trajectory_handles = []
    frustum_handles = []
    if aligned_traj.exists():
        traj_data = json.loads(aligned_traj.read_text(encoding="utf-8"))
        cameras = traj_data.get("cameras", [])
        # 取每隔几帧画一个相机，避免一次性塞几百个 frustum。
        every = max(1, len(cameras) // 60 or 1)
        for cam_idx, cam in enumerate(cameras):
            if cam_idx % every != 0:
                continue
            position = np.asarray(cam["position"], dtype=np.float32) - scene_center
            rotation = np.asarray(cam["rotation"], dtype=np.float32)
            pose_4x4 = np.eye(4, dtype=np.float32)
            pose_4x4[:3, :3] = rotation
            pose_4x4[:3, 3] = position
            try:
                se3 = viser_tf.SE3.from_matrix(pose_4x4[:3, :])
            except Exception:  # noqa: BLE001
                continue
            frame = server.scene.add_frame(
                f"aligned_cameras/frame_{cam_idx:05d}",
                wxyz=se3.rotation().wxyz,
                position=se3.translation(),
                axes_length=0.04,
                axes_radius=0.0015,
                origin_radius=0.0015,
            )
            frustum = server.scene.add_camera_frustum(
                f"aligned_cameras/frame_{cam_idx:05d}/frustum",
                fov=1.0,
                aspect=1.0,
                scale=0.04,
                line_width=1.0,
            )
            trajectory_handles.append(frame)
            frustum_handles.append(frustum)

        # 用 add_spline_catmull_rom 把相机中心串成一条轨迹线。
        all_positions = np.asarray(
            [np.asarray(c["position"], dtype=np.float32) - scene_center for c in cameras],
            dtype=np.float32,
        )
        if len(all_positions) >= 2:
            server.scene.add_point_cloud(
                name="aligned_trajectory_line",
                points=all_positions,
                colors=np.tile(np.array([255, 200, 0], dtype=np.uint8), (len(all_positions), 1)),
                point_size=0.008,
                point_shape="circle",
            )

    show_points = server.gui.add_checkbox("显示点云", initial_value=True)
    show_traj = server.gui.add_checkbox("显示相机", initial_value=True)

    @show_points.on_update
    def _(_event) -> None:
        point_cloud.visible = show_points.value

    @show_traj.on_update
    def _(_event) -> None:
        for h in trajectory_handles + frustum_handles:
            h.visible = show_traj.value

    print(f"对齐 3D 场景已加载: {aligned_ply}")
    if aligned_traj.exists():
        print(f"全局相机轨迹: {aligned_traj}")
    print(f"Viser 3D viewer 已启动。请在浏览器打开: http://localhost:{args.port}")


def main() -> None:
    """脚本入口。"""
    args = parse_args()
    result_dir = Path(args.result_dir)

    summary_path = result_dir / "summary.json"
    if summary_path.exists():
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        print(f"读取到结果摘要: {summary_path}")
        print(f"处理模式: {summary.get('mode', 'unknown')}")

    if args.aligned:
        build_aligned_scene(result_dir, args)
    elif args.aggregate_windows:
        build_aggregated_scene(result_dir, args)
    else:
        predictions = load_predictions(result_dir)
        build_scene(predictions, result_dir, args)

    # 保持进程运行，直到用户手动 Ctrl+C。
    try:
        import time

        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Viser viewer 已退出。")


if __name__ == "__main__":
    main()
