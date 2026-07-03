#!/usr/bin/env python3
"""
在本项目的第一人称视频上运行 VGGT，并提供分块处理和可视化预览。

VGGT 不是传统意义上的实时视觉里程计，它更像一个“多视图几何推理器”：
一次输入一组图片，然后预测这组图片的相机参数、深度图和三维点。
因此本脚本采用工程上更稳的方式处理长视频：

1. 按时间顺序从视频里抽帧；
2. 每收集到一个小窗口就调用一次 VGGT；
3. 保存每个窗口的深度图、置信度图、点云和相机中心；
4. 可选用 OpenCV 窗口实时显示“原图 + 深度 + 置信度”；
5. 同时保存一个 preview.mp4，方便在没有 GUI 的 WSL 环境里回看。

默认参数偏保守，适配 RTX 4050 Laptop GPU 的 6GB 显存。
"""

from __future__ import annotations

import argparse
import gc
import json
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

import cv2
import numpy as np
import torch


REPO = Path("/home/ros/ros2_orbslam3")
DEFAULT_VGGT_ROOT = REPO / "Opensource code" / "vggt-main" / "vggt-main"
DEFAULT_VIDEO = REPO / "resources" / "input_video.mp4"
DEFAULT_OUTPUT = REPO / "output" / "vggt_input_video"
DEFAULT_TORCH_HOME = REPO / ".cache" / "torch"
VGGT_WEIGHTS_URL = "https://huggingface.co/facebook/VGGT-1B/resolve/main/model.pt"


@dataclass
class FrameSample:
    """记录一帧被抽取出来后的必要信息。"""

    sample_index: int
    source_frame: int
    source_time_sec: float
    image_path: Path
    frame_bgr: np.ndarray


@dataclass
class WindowResult:
    """记录一个 VGGT 推理窗口的结果摘要。"""

    window_index: int
    sample_indices: list[int]
    source_frames: list[int]
    source_times_sec: list[float]
    output_dir: Path
    point_cloud_points: int
    camera_centers: list[list[float]]
    depth_mean: float
    depth_min: float
    depth_max: float
    elapsed_sec: float


def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(description="Run VGGT on input_video.mp4 with chunked preview")
    parser.add_argument("--video", default=str(DEFAULT_VIDEO), help="输入视频路径")
    parser.add_argument("--vggt-root", default=str(DEFAULT_VGGT_ROOT), help="VGGT 官方源码根目录")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT), help="输出目录")
    parser.add_argument("--start-sec", type=float, default=0.0, help="从视频第几秒开始处理")
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=0.0,
        help="处理多少秒；0 表示一直处理到视频结束",
    )
    parser.add_argument(
        "--frame-step",
        type=int,
        default=30,
        help="每隔多少个原始视频帧抽一帧；30 约等于每秒 1 帧",
    )
    parser.add_argument(
        "--num-frames",
        type=int,
        default=0,
        help="最多抽取多少帧；0 表示不限制，按 duration-sec 或视频结尾停止",
    )
    parser.add_argument(
        "--window-size",
        type=int,
        default=8,
        help=(
            "每个 VGGT 推理窗口包含多少张抽样帧。"
            "VGGT 是多视图模型，2 帧基本退化成单目，建议至少 4 帧，"
            "6GB 显存常用 4-8。"
        ),
    )
    parser.add_argument(
        "--window-stride",
        type=int,
        default=4,
        help=(
            "窗口向前滑动多少张抽样帧。"
            "必须小于 window-size 才能在相邻窗口之间留共享帧，"
            "后处理的全局对齐依赖这些共享帧（推荐 stride = window-size / 2）。"
        ),
    )
    parser.add_argument("--preprocess", choices=["pad", "crop"], default="pad", help="VGGT 输入预处理方式")
    parser.add_argument("--conf-percentile", type=float, default=85.0, help="点云导出时保留的置信度百分位")
    parser.add_argument("--max-points", type=int, default=60000, help="每个窗口点云最多导出多少个点")
    parser.add_argument("--weights", default="", help="可选：本地 model.pt 路径；为空则使用 .cache/torch")
    parser.add_argument("--cpu", action="store_true", help="强制使用 CPU，仅用于导入测试，不建议跑完整推理")
    parser.add_argument("--show", action="store_true", help="用 OpenCV 窗口实时显示原图、深度和置信度")
    parser.add_argument("--no-preview-video", action="store_true", help="不保存 preview.mp4")
    parser.add_argument("--preview-scale", type=float, default=0.75, help="显示/保存预览图时的缩放比例")
    parser.add_argument(
        "--keep-track-head",
        action="store_true",
        help="默认关闭 VGGT 点跟踪分支以节省显存；需要测试 tracks 时再打开",
    )
    parser.add_argument(
        "--keep-point-head",
        action="store_true",
        help="默认关闭 VGGT 直接点图分支以节省显存；点云会由深度图反投影生成",
    )
    parser.add_argument(
        "--save-window-npz",
        action="store_true",
        help="保存每个窗口的完整 predictions.npz；文件较大，默认只保存最后一个窗口",
    )
    parser.add_argument(
        "--no-aggregate",
        action="store_true",
        help="跑完所有窗口后跳过全局对齐聚合，仅保留每窗口的局部输出",
    )
    parser.add_argument(
        "--aggregate-voxel",
        type=float,
        default=0.02,
        help="对齐聚合时的 voxel 下采样尺寸（相对于场景尺度），0 表示不下采样",
    )
    parser.add_argument(
        "--aggregate-conf-percentile",
        type=float,
        default=70.0,
        help="对齐聚合时丢弃低置信度像素的百分位阈值",
    )
    parser.add_argument(
        "--aggregate-edge-margin",
        type=int,
        default=8,
        help="对齐聚合时每张深度图四周裁掉的像素数",
    )
    parser.add_argument(
        "--aggregate-depth-max-ratio",
        type=float,
        default=5.0,
        help="对齐聚合时按窗口深度中位数的多少倍截断远点，0 表示不截断",
    )
    parser.add_argument(
        "--aggregate-scale-mode",
        choices=["depth_median", "umeyama", "se3"],
        default="depth_median",
        help="对齐聚合时相邻窗口的尺度估计方式，详见 aggregate_vggt_aligned.py",
    )
    return parser.parse_args()


def prepare_imports(vggt_root: Path) -> None:
    """把 VGGT 源码目录加入 Python 搜索路径。"""
    if not vggt_root.exists():
        raise FileNotFoundError(f"找不到 VGGT 源码目录: {vggt_root}")
    sys.path.insert(0, str(vggt_root))


def clean_output_dir(output_dir: Path) -> None:
    """清理本脚本会反复生成的输出子目录和摘要文件。"""
    output_dir.mkdir(parents=True, exist_ok=True)
    for subdir_name in ("images", "previews", "windows"):
        subdir = output_dir / subdir_name
        if subdir.exists():
            for old_file in subdir.rglob("*"):
                if old_file.is_file():
                    old_file.unlink()
        subdir.mkdir(parents=True, exist_ok=True)
    for old_file in (
        "frame_meta.json",
        "summary.json",
        "summary.txt",
        "camera_centers.txt",
        "preview.mp4",
        "predictions.npz",
        "vggt_points.ply",
    ):
        path = output_dir / old_file
        if path.exists():
            path.unlink()


def choose_device(force_cpu: bool) -> tuple[torch.device, torch.dtype | None]:
    """选择推理设备和半精度类型。"""
    if force_cpu or not torch.cuda.is_available():
        return torch.device("cpu"), None

    major, _minor = torch.cuda.get_device_capability(0)
    dtype = torch.bfloat16 if major >= 8 else torch.float16
    return torch.device("cuda"), dtype


def load_vggt_model(
    device: torch.device,
    dtype: torch.dtype | None,
    weights_path: str,
    keep_track_head: bool,
    keep_point_head: bool,
):
    """初始化 VGGT，并加载本地或自动下载的权重。"""
    from vggt.models.vggt import VGGT

    # 默认关闭 point head 和 track head，是为了让 6GB 显存机器先稳定跑通。
    # 本项目主要用相机和深度，点云可由深度图反投影生成。
    model = VGGT(enable_point=keep_point_head, enable_track=keep_track_head)

    local_cached_weights = DEFAULT_TORCH_HOME / "hub" / "checkpoints" / "model.pt"
    if weights_path:
        resolved_weights = Path(weights_path)
        state_dict = torch.load(resolved_weights, map_location="cpu", weights_only=True, mmap=True)
    elif local_cached_weights.exists():
        resolved_weights = local_cached_weights
        state_dict = torch.load(resolved_weights, map_location="cpu", weights_only=True, mmap=True)
    else:
        resolved_weights = None
        DEFAULT_TORCH_HOME.mkdir(parents=True, exist_ok=True)
        os.environ.setdefault("TORCH_HOME", str(DEFAULT_TORCH_HOME))
        state_dict = torch.hub.load_state_dict_from_url(
            VGGT_WEIGHTS_URL,
            model_dir=str(DEFAULT_TORCH_HOME / "hub" / "checkpoints"),
            map_location="cpu",
        )

    model_keys = set(model.state_dict().keys())
    filtered_state_dict = {key: value for key, value in state_dict.items() if key in model_keys}
    ignored_keys = len(state_dict) - len(filtered_state_dict)
    incompatible = model.load_state_dict(filtered_state_dict, strict=False, assign=resolved_weights is not None)
    if incompatible.missing_keys:
        print(f"注意：缺失权重键数量 {len(incompatible.missing_keys)}")
    if ignored_keys or incompatible.unexpected_keys:
        print(f"注意：忽略未使用权重键数量 {ignored_keys + len(incompatible.unexpected_keys)}")

    del filtered_state_dict
    del state_dict
    gc.collect()

    model.eval()
    if device.type == "cuda" and dtype is not None:
        return model.to(device=device, dtype=dtype)
    return model.to(device)


def read_sampled_frames(
    video_path: Path,
    output_dir: Path,
    start_sec: float,
    duration_sec: float,
    frame_step: int,
    max_samples: int,
) -> tuple[list[FrameSample], dict]:
    """按设定步长从视频中抽样，并把抽样帧保存到 images 目录。"""
    if not video_path.exists():
        raise FileNotFoundError(f"找不到输入视频: {video_path}")
    if frame_step <= 0:
        raise ValueError("--frame-step 必须大于 0")

    images_dir = output_dir / "images"
    images_dir.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"OpenCV 无法打开视频: {video_path}")

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
    start_frame = max(0, int(round(start_sec * fps)))
    end_frame = total_frames
    if duration_sec > 0:
        end_frame = min(end_frame, start_frame + int(round(duration_sec * fps)))

    cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)

    samples: list[FrameSample] = []
    source_frame = start_frame
    while source_frame < end_frame:
        ok, frame_bgr = cap.read()
        if not ok:
            break

        if (source_frame - start_frame) % frame_step == 0:
            sample_index = len(samples)
            image_path = images_dir / f"frame_{sample_index:05d}_src_{source_frame:06d}.png"
            cv2.imwrite(str(image_path), frame_bgr)
            samples.append(
                FrameSample(
                    sample_index=sample_index,
                    source_frame=source_frame,
                    source_time_sec=source_frame / fps,
                    image_path=image_path,
                    frame_bgr=frame_bgr,
                )
            )
            if max_samples > 0 and len(samples) >= max_samples:
                break

        source_frame += 1

    cap.release()

    if not samples:
        raise RuntimeError("没有抽取到任何帧，请检查 start-sec、duration-sec、frame-step 或视频本身。")

    meta = {
        "video": str(video_path),
        "fps": fps,
        "total_frames": total_frames,
        "width": width,
        "height": height,
        "duration_sec": total_frames / fps if fps else 0.0,
        "start_sec": start_sec,
        "duration_limit_sec": duration_sec,
        "start_frame": start_frame,
        "end_frame": end_frame,
        "frame_step": frame_step,
        "max_samples": max_samples,
        "num_saved": len(samples),
        "frames": [
            {
                "sample_index": sample.sample_index,
                "source_frame": sample.source_frame,
                "source_time_sec": sample.source_time_sec,
                "image_path": str(sample.image_path),
            }
            for sample in samples
        ],
    }
    (output_dir / "frame_meta.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
    return samples, meta


def build_windows(samples: list[FrameSample], window_size: int, window_stride: int) -> list[list[FrameSample]]:
    """把抽样帧组织成滑动窗口。"""
    if window_size <= 0:
        raise ValueError("--window-size 必须大于 0")
    if window_stride <= 0:
        raise ValueError("--window-stride 必须大于 0")

    windows: list[list[FrameSample]] = []
    start = 0
    while start < len(samples):
        window = samples[start : start + window_size]
        if not window:
            break
        windows.append(window)
        if start + window_size >= len(samples):
            break
        start += window_stride
    return windows


def run_inference(
    model,
    frame_paths: list[Path],
    device: torch.device,
    dtype: torch.dtype | None,
    preprocess: str,
) -> dict[str, np.ndarray]:
    """调用 VGGT 前向推理，并返回 numpy 格式结果。"""
    from vggt.utils.geometry import unproject_depth_map_to_point_map
    from vggt.utils.load_fn import load_and_preprocess_images
    from vggt.utils.pose_enc import pose_encoding_to_extri_intri

    images = load_and_preprocess_images([str(p) for p in frame_paths], mode=preprocess).to(device)
    if images.dim() == 4:
        images = images.unsqueeze(0)
    if device.type == "cuda" and dtype is not None:
        images = images.to(dtype=dtype)

    with torch.no_grad():
        if device.type == "cuda":
            with torch.cuda.amp.autocast(dtype=dtype):
                # 不直接调用 model(images)，是为了避开官方 forward 里关闭 autocast
                # 后出现的 dtype 不一致问题；这里按官方顺序手动执行各模块。
                aggregated_tokens_list, patch_start_idx = model.aggregator(images)
                head_dtype = next(model.camera_head.parameters()).dtype
                aggregated_tokens_list = [
                    token.to(head_dtype) if isinstance(token, torch.Tensor) else None
                    for token in aggregated_tokens_list
                ]

                predictions = {}
                if model.camera_head is not None:
                    pose_enc_list = model.camera_head(aggregated_tokens_list)
                    predictions["pose_enc"] = pose_enc_list[-1]
                if model.depth_head is not None:
                    depth, depth_conf = model.depth_head(
                        aggregated_tokens_list,
                        images=images.to(head_dtype),
                        patch_start_idx=patch_start_idx,
                        frames_chunk_size=1,
                    )
                    predictions["depth"] = depth
                    predictions["depth_conf"] = depth_conf
                if model.point_head is not None:
                    pts3d, pts3d_conf = model.point_head(
                        aggregated_tokens_list,
                        images=images.to(head_dtype),
                        patch_start_idx=patch_start_idx,
                        frames_chunk_size=1,
                    )
                    predictions["world_points"] = pts3d
                    predictions["world_points_conf"] = pts3d_conf
                predictions["images"] = images
        else:
            predictions = model(images)

    extrinsic, intrinsic = pose_encoding_to_extri_intri(predictions["pose_enc"].float(), images.shape[-2:])
    predictions["extrinsic"] = extrinsic
    predictions["intrinsic"] = intrinsic

    numpy_predictions: dict[str, np.ndarray] = {}
    for key, value in predictions.items():
        if isinstance(value, torch.Tensor):
            numpy_predictions[key] = value.detach().float().cpu().numpy().squeeze(0)

    if "world_points" not in numpy_predictions:
        numpy_predictions["world_points_from_depth"] = unproject_depth_map_to_point_map(
            numpy_predictions["depth"],
            numpy_predictions["extrinsic"],
            numpy_predictions["intrinsic"],
        )

    del images
    del predictions
    if device.type == "cuda":
        torch.cuda.empty_cache()
    gc.collect()

    return numpy_predictions


def camera_centers_from_extrinsic(extrinsic: np.ndarray) -> np.ndarray:
    """由 OpenCV 约定的 world-to-camera 外参 [R|t] 计算相机中心 C=-R^T t。"""
    centers = []
    for pose in extrinsic:
        rotation = pose[:, :3]
        translation = pose[:, 3]
        centers.append(-rotation.T @ translation)
    return np.asarray(centers, dtype=np.float32)


def normalize_depth(depth_i: np.ndarray) -> np.ndarray:
    """把一张深度图归一化到 0 到 1，便于伪彩色显示。"""
    valid = np.isfinite(depth_i) & (depth_i > 0)
    if not np.any(valid):
        return np.zeros_like(depth_i, dtype=np.float32)
    lo, hi = np.percentile(depth_i[valid], [2, 98])
    return np.clip((depth_i - lo) / max(float(hi - lo), 1e-6), 0, 1).astype(np.float32)


def make_visual_panel(
    source_bgr: np.ndarray,
    depth: np.ndarray,
    depth_conf: np.ndarray,
    title: str,
    scale: float,
) -> np.ndarray:
    """把原图、深度图和置信度图拼成一张预览图。"""
    depth_i = depth[..., 0] if depth.ndim == 3 else depth
    depth_norm = normalize_depth(depth_i)

    conf_norm = depth_conf / max(float(np.percentile(depth_conf, 99)), 1e-6)
    conf_norm = np.clip(conf_norm, 0, 1)

    depth_color = cv2.applyColorMap((depth_norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    conf_color = cv2.applyColorMap((conf_norm * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)

    target_h, target_w = depth_color.shape[:2]
    source_resized = cv2.resize(source_bgr, (target_w, target_h), interpolation=cv2.INTER_AREA)
    panel = np.hstack([source_resized, depth_color, conf_color])

    cv2.putText(panel, "source", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(
        panel,
        "VGGT depth",
        (target_w + 12, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        panel,
        "confidence",
        (target_w * 2 + 12, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(panel, title, (12, target_h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

    if scale != 1.0:
        new_w = max(1, int(round(panel.shape[1] * scale)))
        new_h = max(1, int(round(panel.shape[0] * scale)))
        panel = cv2.resize(panel, (new_w, new_h), interpolation=cv2.INTER_AREA)
    return panel


def save_depth_previews(predictions: dict[str, np.ndarray], window_dir: Path) -> list[Path]:
    """保存一个窗口内所有帧的深度图和置信度图。"""
    preview_dir = window_dir / "previews"
    preview_dir.mkdir(parents=True, exist_ok=True)

    saved_paths: list[Path] = []
    depth = predictions["depth"]
    depth_conf = predictions["depth_conf"]
    for idx in range(depth.shape[0]):
        depth_norm = normalize_depth(depth[idx, ..., 0])
        conf_i = depth_conf[idx]
        conf_norm = conf_i / max(float(np.percentile(conf_i, 99)), 1e-6)
        conf_norm = np.clip(conf_norm, 0, 1)

        depth_color = cv2.applyColorMap((depth_norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
        conf_color = cv2.applyColorMap((conf_norm * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
        depth_path = preview_dir / f"depth_{idx:03d}.png"
        conf_path = preview_dir / f"confidence_{idx:03d}.png"
        cv2.imwrite(str(depth_path), depth_color)
        cv2.imwrite(str(conf_path), conf_color)
        saved_paths.extend([depth_path, conf_path])
    return saved_paths


def save_ascii_ply(points: np.ndarray, colors: np.ndarray, path: Path) -> None:
    """用 ASCII PLY 格式保存点云，避免额外依赖 open3d 或 trimesh。"""
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


def save_point_cloud(
    predictions: dict[str, np.ndarray],
    output_path: Path,
    conf_percentile: float,
    max_points: int,
) -> int:
    """从 VGGT 结果导出一个置信度筛选后的简易点云。"""
    if "world_points" in predictions:
        points = predictions["world_points"].reshape(-1, 3)
        conf = predictions["world_points_conf"].reshape(-1)
    else:
        points = predictions["world_points_from_depth"].reshape(-1, 3)
        conf = predictions["depth_conf"].reshape(-1)

    images = predictions["images"].transpose(0, 2, 3, 1).reshape(-1, 3)
    colors = np.clip(images * 255.0, 0, 255).astype(np.uint8)

    finite = np.isfinite(points).all(axis=1) & np.isfinite(conf)
    if not np.any(finite):
        return 0

    threshold = np.percentile(conf[finite], conf_percentile)
    keep = finite & (conf >= threshold)
    keep_indices = np.flatnonzero(keep)
    if len(keep_indices) > max_points:
        rng = np.random.default_rng(42)
        keep_indices = rng.choice(keep_indices, size=max_points, replace=False)

    save_ascii_ply(points[keep_indices], colors[keep_indices], output_path)
    return int(len(keep_indices))


def save_window_outputs(
    predictions: dict[str, np.ndarray],
    window: list[FrameSample],
    output_dir: Path,
    window_index: int,
    conf_percentile: float,
    max_points: int,
    save_window_npz: bool,
) -> WindowResult:
    """保存一个窗口的推理结果，并返回摘要。"""
    window_dir = output_dir / "windows" / f"window_{window_index:04d}"
    window_dir.mkdir(parents=True, exist_ok=True)

    # 全局对齐脚本需要的最小数据：extrinsic/intrinsic/depth/depth_conf。
    # 这些字段加起来约 4-6MB（窗口 8 帧 518x518），可以接受。
    # `images` 已经在 output_dir/images 下，不再重复存。
    keys_for_alignment = ("extrinsic", "intrinsic", "depth", "depth_conf")
    alignment_payload = {k: predictions[k] for k in keys_for_alignment if k in predictions}
    np.savez_compressed(window_dir / "predictions.npz", **alignment_payload)

    # 兼容旧脚本：output_dir 根下保留最后一次的 predictions.npz，供 open_vggt_viser.py 非聚合模式使用。
    np.savez_compressed(output_dir / "predictions.npz", **predictions)
    if save_window_npz:
        # 用户显式要求时再保留完整 predictions（带 images）。
        np.savez_compressed(window_dir / "predictions_full.npz", **predictions)

    save_depth_previews(predictions, window_dir)
    point_count = save_point_cloud(predictions, window_dir / "vggt_points.ply", conf_percentile, max_points)
    if window_index == 0:
        save_point_cloud(predictions, output_dir / "vggt_points.ply", conf_percentile, max_points)

    centers = camera_centers_from_extrinsic(predictions["extrinsic"])
    trajectory_lines = ["# sample_index source_frame source_time_sec tx ty tz\n"]
    for sample, center in zip(window, centers):
        trajectory_lines.append(
            f"{sample.sample_index} {sample.source_frame} {sample.source_time_sec:.6f} "
            f"{center[0]:.8f} {center[1]:.8f} {center[2]:.8f}\n"
        )
    (window_dir / "camera_centers.txt").write_text("".join(trajectory_lines), encoding="utf-8")

    window_summary = {
        "window_index": window_index,
        "sample_indices": [sample.sample_index for sample in window],
        "source_frames": [sample.source_frame for sample in window],
        "source_times_sec": [sample.source_time_sec for sample in window],
        "frame_paths": [str(sample.image_path) for sample in window],
        "depth_shape": list(predictions["depth"].shape),
        "world_points_shape": list(
            predictions["world_points"].shape
            if "world_points" in predictions
            else predictions["world_points_from_depth"].shape
        ),
        "point_cloud_source": "world_points" if "world_points" in predictions else "world_points_from_depth",
        "depth_min": float(np.nanmin(predictions["depth"])),
        "depth_max": float(np.nanmax(predictions["depth"])),
        "depth_mean": float(np.nanmean(predictions["depth"])),
        "depth_conf_percentiles": {
            "p10": float(np.percentile(predictions["depth_conf"], 10)),
            "p50": float(np.percentile(predictions["depth_conf"], 50)),
            "p90": float(np.percentile(predictions["depth_conf"], 90)),
        },
        "camera_centers": centers.tolist(),
        # 全局对齐需要所有帧的位姿，而不是只有第一帧；为了向后兼容也保留 first_*。
        "extrinsics": predictions["extrinsic"].tolist(),
        "intrinsics": predictions["intrinsic"].tolist(),
        "first_intrinsic": predictions["intrinsic"][0].tolist(),
        "first_extrinsic": predictions["extrinsic"][0].tolist(),
        "exported_ply_points": point_count,
    }
    (window_dir / "summary.json").write_text(json.dumps(window_summary, ensure_ascii=False, indent=2), encoding="utf-8")

    return WindowResult(
        window_index=window_index,
        sample_indices=window_summary["sample_indices"],
        source_frames=window_summary["source_frames"],
        source_times_sec=window_summary["source_times_sec"],
        output_dir=window_dir,
        point_cloud_points=point_count,
        camera_centers=centers.tolist(),
        depth_mean=window_summary["depth_mean"],
        depth_min=window_summary["depth_min"],
        depth_max=window_summary["depth_max"],
        elapsed_sec=0.0,
    )


def append_preview_frames(
    predictions: dict[str, np.ndarray],
    window: list[FrameSample],
    window_index: int,
    show: bool,
    scale: float,
) -> Iterator[np.ndarray]:
    """显示并保存一个窗口内每一帧的可视化拼图。"""
    depth = predictions["depth"]
    depth_conf = predictions["depth_conf"]

    for local_idx, sample in enumerate(window):
        title = (
            f"window {window_index:04d} | sample {sample.sample_index:05d} | "
            f"src frame {sample.source_frame} | {sample.source_time_sec:.2f}s"
        )
        panel = make_visual_panel(sample.frame_bgr, depth[local_idx], depth_conf[local_idx], title, scale)

        if show:
            cv2.imshow("VGGT preview: source | depth | confidence", panel)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                raise KeyboardInterrupt("用户关闭 VGGT 预览窗口")

        yield panel


def create_preview_writer(path: Path, first_panel: np.ndarray) -> cv2.VideoWriter:
    """根据第一张预览图创建 MP4 写入器。"""
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(path), fourcc, 4.0, (first_panel.shape[1], first_panel.shape[0]))
    if not writer.isOpened():
        raise RuntimeError(f"无法创建预览视频: {path}")
    return writer


def write_overall_summary(
    output_dir: Path,
    frame_meta: dict,
    window_results: list[WindowResult],
    args: argparse.Namespace,
    total_elapsed_sec: float,
) -> None:
    """写入整段运行的 JSON 和文本摘要。"""
    all_camera_lines = ["# window sample_index source_frame source_time_sec tx ty tz\n"]
    for result in window_results:
        for sample_index, source_frame, source_time, center in zip(
            result.sample_indices,
            result.source_frames,
            result.source_times_sec,
            result.camera_centers,
        ):
            all_camera_lines.append(
                f"{result.window_index} {sample_index} {source_frame} {source_time:.6f} "
                f"{center[0]:.8f} {center[1]:.8f} {center[2]:.8f}\n"
            )
    (output_dir / "camera_centers.txt").write_text("".join(all_camera_lines), encoding="utf-8")

    summary = {
        "mode": "chunked_video_preview",
        "video": frame_meta.get("video"),
        "video_fps": frame_meta.get("fps"),
        "video_total_frames": frame_meta.get("total_frames"),
        "video_duration_sec": frame_meta.get("duration_sec"),
        "processed_sample_frames": frame_meta.get("num_saved"),
        "window_size": args.window_size,
        "window_stride": args.window_stride,
        "num_windows": len(window_results),
        "frame_step": args.frame_step,
        "start_sec": args.start_sec,
        "duration_limit_sec": args.duration_sec,
        "preprocess": args.preprocess,
        "point_cloud_source": "world_points" if args.keep_point_head else "world_points_from_depth",
        "preview_video": str(output_dir / "preview.mp4") if not args.no_preview_video else "",
        "total_elapsed_sec": total_elapsed_sec,
        "windows": [
            {
                "window_index": result.window_index,
                "sample_indices": result.sample_indices,
                "source_frames": result.source_frames,
                "source_times_sec": result.source_times_sec,
                "output_dir": str(result.output_dir),
                "point_cloud_points": result.point_cloud_points,
                "depth_min": result.depth_min,
                "depth_mean": result.depth_mean,
                "depth_max": result.depth_max,
                "elapsed_sec": result.elapsed_sec,
            }
            for result in window_results
        ],
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")

    readable = [
        "VGGT 视频分块测试摘要\n",
        f"输入视频: {frame_meta.get('video')}\n",
        f"视频信息: {frame_meta.get('width')}x{frame_meta.get('height')}, "
        f"{frame_meta.get('fps'):.4f} FPS, 总帧数 {frame_meta.get('total_frames')}\n",
        f"处理范围: start-sec={args.start_sec}, duration-sec={args.duration_sec or '到视频结束'}\n",
        f"抽帧间隔: 每 {args.frame_step} 个原始帧取 1 帧\n",
        f"抽样帧数: {frame_meta.get('num_saved')}\n",
        f"窗口设置: window-size={args.window_size}, window-stride={args.window_stride}\n",
        f"完成窗口数: {len(window_results)}\n",
        f"总耗时: {total_elapsed_sec:.2f} 秒\n",
        f"预览视频: {summary['preview_video'] or '未保存'}\n",
        "相机中心汇总见 camera_centers.txt\n",
        "每个窗口的深度图、置信度图和点云见 windows/window_xxxx/\n",
    ]
    (output_dir / "summary.txt").write_text("".join(readable), encoding="utf-8")


def main() -> None:
    """脚本入口。"""
    args = parse_args()
    vggt_root = Path(args.vggt_root)
    video_path = Path(args.video)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    prepare_imports(vggt_root)
    clean_output_dir(output_dir)

    if args.show and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        print("注意：当前没有可用图形显示环境，--show 将自动关闭；将改为保存 preview.mp4。")
        args.show = False

    device, dtype = choose_device(args.cpu)
    print(f"使用设备: {device}")
    if dtype is not None:
        print(f"使用半精度类型: {dtype}")

    print("正在加载 VGGT 模型和权重，这一步第一次会比较慢。")
    model = load_vggt_model(device, dtype, args.weights, args.keep_track_head, args.keep_point_head)

    samples, frame_meta = read_sampled_frames(
        video_path,
        output_dir,
        args.start_sec,
        args.duration_sec,
        args.frame_step,
        args.num_frames,
    )
    windows = build_windows(samples, args.window_size, args.window_stride)
    print(f"已抽取 {len(samples)} 帧，生成 {len(windows)} 个 VGGT 推理窗口。")

    preview_writer: cv2.VideoWriter | None = None
    preview_path = output_dir / "preview.mp4"
    window_results: list[WindowResult] = []
    total_start = time.time()

    try:
        for window_index, window in enumerate(windows):
            window_start = time.time()
            frame_paths = [sample.image_path for sample in window]
            print(
                f"[{window_index + 1}/{len(windows)}] "
                f"处理样本帧 {window[0].sample_index} 到 {window[-1].sample_index} ..."
            )
            predictions = run_inference(model, frame_paths, device, dtype, args.preprocess)
            result = save_window_outputs(
                predictions,
                window,
                output_dir,
                window_index,
                args.conf_percentile,
                args.max_points,
                args.save_window_npz,
            )
            result.elapsed_sec = time.time() - window_start
            window_results.append(result)

            for panel in append_preview_frames(predictions, window, window_index, args.show, args.preview_scale):
                if not args.no_preview_video:
                    if preview_writer is None:
                        preview_writer = create_preview_writer(preview_path, panel)
                    preview_writer.write(panel)

            print(
                f"窗口 {window_index:04d} 完成: "
                f"耗时 {result.elapsed_sec:.2f}s, 点云 {result.point_cloud_points} 点, "
                f"深度均值 {result.depth_mean:.4f}"
            )

            del predictions
            if device.type == "cuda":
                torch.cuda.empty_cache()
            gc.collect()
    except KeyboardInterrupt as exc:
        print(f"收到中断: {exc}")
    finally:
        if preview_writer is not None:
            preview_writer.release()
        if args.show:
            cv2.destroyAllWindows()

    total_elapsed = time.time() - total_start
    write_overall_summary(output_dir, frame_meta, window_results, args, total_elapsed)
    print(f"VGGT 视频处理完成，输出目录: {output_dir}")
    if not args.no_preview_video:
        print(f"预览视频: {preview_path}")
    print("说明：如果想实时看到窗口，请加 --show；如果 WSL 没有图形界面，就直接打开 preview.mp4。")

    if args.no_aggregate:
        print("跳过全局对齐聚合（--no-aggregate）。")
        return

    if args.window_stride >= args.window_size:
        print(
            "警告：window-stride >= window-size，相邻窗口没有共享帧，"
            "无法做基于共享帧的全局对齐，跳过聚合。"
        )
        return

    print("开始基于共享帧做全局对齐聚合，这一步在 CPU 上完成。")
    try:
        scripts_dir = str(Path(__file__).resolve().parent)
        if scripts_dir not in sys.path:
            sys.path.insert(0, scripts_dir)
        from aggregate_vggt_aligned import aggregate_aligned_scene

        aggregate_summary = aggregate_aligned_scene(
            result_dir=output_dir,
            output_dir=output_dir / "aligned_full",
            conf_percentile=args.aggregate_conf_percentile,
            edge_margin=args.aggregate_edge_margin,
            depth_max_ratio=args.aggregate_depth_max_ratio,
            voxel_size_ratio=args.aggregate_voxel,
            scale_mode=args.aggregate_scale_mode,
        )
        print(
            f"聚合完成：{aggregate_summary['aggregated_points']} 点 -> "
            f"{aggregate_summary['aligned_ply']}"
        )
    except Exception as exc:  # noqa: BLE001
        # 聚合阶段失败不应该影响已经跑完的窗口结果；只打印错误供后续手动重试。
        print(f"聚合失败，已保留窗口结果，可用 scripts/aggregate_vggt_aligned.py 单独重试: {exc}")


if __name__ == "__main__":
    main()
