#!/usr/bin/env python3
"""
用 VGGT 对一组覆盖整段视频的关键帧做单场景重建。

这个脚本和 run_vggt_video.py 的区别是：
1. 它不按滑动窗口拆分每两个帧一组；
2. 它把整段视频均匀抽成一组关键帧；
3. 一次性送入 VGGT，得到同一个场景坐标系下的相机、深度和点云；
4. 更适合做论文里那种“一个场景、一套相机轨迹、一张点云”的展示。

注意：
如果关键帧太多，6GB 显存可能会不够，所以默认先取 16 帧。
"""

from __future__ import annotations

import argparse
import gc
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch


REPO = Path("/home/ros/ros2_orbslam3")
DEFAULT_VGGT_ROOT = REPO / "Opensource code" / "vggt-main" / "vggt-main"
DEFAULT_VIDEO = REPO / "resources" / "input_video.mp4"
DEFAULT_OUTPUT = REPO / "output" / "vggt_scene_full"
DEFAULT_TORCH_HOME = REPO / ".cache" / "torch"
VGGT_WEIGHTS_URL = "https://huggingface.co/facebook/VGGT-1B/resolve/main/model.pt"


def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(description="Run VGGT on a full-scene keyframe set")
    parser.add_argument("--video", default=str(DEFAULT_VIDEO), help="输入视频路径")
    parser.add_argument("--vggt-root", default=str(DEFAULT_VGGT_ROOT), help="VGGT 官方源码根目录")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT), help="输出目录")
    parser.add_argument("--num-frames", type=int, default=16, help="均匀抽取多少帧做单场景重建")
    parser.add_argument("--start-sec", type=float, default=0.0, help="从视频第几秒开始")
    parser.add_argument("--duration-sec", type=float, default=0.0, help="处理多少秒，0 表示到结尾")
    parser.add_argument("--preprocess", choices=["pad", "crop"], default="pad", help="VGGT 输入预处理方式")
    parser.add_argument("--weights", default="", help="可选：本地 model.pt 路径")
    parser.add_argument("--cpu", action="store_true", help="强制使用 CPU")
    parser.add_argument(
        "--keep-track-head",
        action="store_true",
        help="默认关闭 track head 以节省显存",
    )
    parser.add_argument(
        "--keep-point-head",
        action="store_true",
        help="默认关闭 point head 以节省显存",
    )
    return parser.parse_args()


def prepare_imports(vggt_root: Path) -> None:
    """把 VGGT 源码目录加入 Python 路径。"""
    if not vggt_root.exists():
        raise FileNotFoundError(f"找不到 VGGT 源码目录: {vggt_root}")
    sys.path.insert(0, str(vggt_root))


def choose_device(force_cpu: bool) -> tuple[torch.device, torch.dtype | None]:
    """选择设备和半精度类型。"""
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
    """加载 VGGT 权重。"""
    from vggt.models.vggt import VGGT

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
    incompatible = model.load_state_dict(filtered_state_dict, strict=False, assign=resolved_weights is not None)
    if incompatible.missing_keys:
        print(f"注意：缺失权重键数量 {len(incompatible.missing_keys)}")
    if incompatible.unexpected_keys:
        print(f"注意：忽略未使用权重键数量 {len(incompatible.unexpected_keys)}")

    model.eval()
    if device.type == "cuda" and dtype is not None:
        return model.to(device=device, dtype=dtype)
    return model.to(device)


def read_video_keyframes(
    video_path: Path,
    output_dir: Path,
    num_frames: int,
    start_sec: float,
    duration_sec: float,
) -> tuple[list[Path], dict]:
    """从视频中均匀抽取关键帧。"""
    if not video_path.exists():
        raise FileNotFoundError(f"找不到输入视频: {video_path}")

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

    if num_frames <= 0:
        raise ValueError("--num-frames 必须大于 0")

    cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
    available = max(1, end_frame - start_frame)
    # 均匀抽样，包含首尾附近的帧。
    sample_positions = np.linspace(start_frame, end_frame - 1, num=num_frames, dtype=np.int64)

    image_paths: list[Path] = []
    sample_index = 0
    for target_frame in sample_positions:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(target_frame))
        ok, frame_bgr = cap.read()
        if not ok:
            continue
        image_path = images_dir / f"frame_{sample_index:05d}_src_{int(target_frame):06d}.png"
        cv2.imwrite(str(image_path), frame_bgr)
        image_paths.append(image_path)
        sample_index += 1

    cap.release()

    if not image_paths:
        raise RuntimeError("没有抽到任何关键帧。")

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
        "num_frames": num_frames,
        "num_saved": len(image_paths),
        "frames": [
            {
                "sample_index": i,
                "source_frame": int(sample_positions[i]),
                "source_time_sec": float(sample_positions[i] / fps),
                "image_path": str(image_paths[i]),
            }
            for i in range(len(image_paths))
        ],
    }
    (output_dir / "frame_meta.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
    return image_paths, meta


def run_inference(model, frame_paths: list[Path], device: torch.device, dtype: torch.dtype | None, preprocess: str) -> dict[str, np.ndarray]:
    """调用 VGGT 前向推理。"""
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


def save_ascii_ply(points: np.ndarray, colors: np.ndarray, path: Path) -> None:
    """保存 ASCII PLY 点云。"""
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


def save_point_cloud(predictions: dict[str, np.ndarray], output_path: Path, conf_percentile: float, max_points: int) -> int:
    """导出点云。"""
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


def main() -> None:
    """脚本入口。"""
    args = parse_args()
    prepare_imports(Path(args.vggt_root))

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    device, dtype = choose_device(args.cpu)
    print(f"使用设备: {device}")
    if dtype is not None:
        print(f"使用半精度类型: {dtype}")

    model = load_vggt_model(device, dtype, args.weights, args.keep_track_head, args.keep_point_head)
    frame_paths, frame_meta = read_video_keyframes(
        Path(args.video),
        output_dir,
        args.num_frames,
        args.start_sec,
        args.duration_sec,
    )

    start = time.time()
    predictions = run_inference(model, frame_paths, device, dtype, args.preprocess)
    elapsed = time.time() - start

    predictions_npz = output_dir / "predictions.npz"
    np.savez_compressed(predictions_npz, **predictions)

    point_count = save_point_cloud(predictions, output_dir / "vggt_points.ply", 85.0, 120000)

    summary = {
        "mode": "full_scene_keyframes",
        "video": args.video,
        "video_fps": frame_meta["fps"],
        "video_total_frames": frame_meta["total_frames"],
        "video_duration_sec": frame_meta["duration_sec"],
        "num_frames": frame_meta["num_saved"],
        "start_sec": args.start_sec,
        "duration_limit_sec": args.duration_sec,
        "preprocess": args.preprocess,
        "point_cloud_points": point_count,
        "elapsed_sec": elapsed,
        "output_dir": str(output_dir),
        "frame_meta": str(output_dir / "frame_meta.json"),
        "predictions_npz": str(predictions_npz),
        "vggt_points_ply": str(output_dir / "vggt_points.ply"),
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    (output_dir / "summary.txt").write_text(
        "VGGT 单场景关键帧重建摘要\n"
        f"输入视频: {args.video}\n"
        f"关键帧数: {frame_meta['num_saved']}\n"
        f"点云点数: {point_count}\n"
        f"耗时: {elapsed:.2f} 秒\n"
        f"结果目录: {output_dir}\n",
        encoding="utf-8",
    )

    print(f"VGGT 单场景重建完成，输出目录: {output_dir}")
    print(f"关键帧数: {frame_meta['num_saved']}")
    print(f"点云点数: {point_count}")
    print(f"耗时: {elapsed:.2f} 秒")


if __name__ == "__main__":
    main()
