#!/usr/bin/env python3
"""DPVO 轨迹增强分析：关键帧记忆、质量评估和几何重定位验证。

这个脚本不修改 DPVO 官方算法内核。它读取 DPVO 已经保存的 TUM 轨迹，
再结合输入视频生成一份工程层面的“跟踪记忆”：

1. 对每个 DPVO 输出位姿计算运动、模糊、遮挡比例和估计质量分数；
2. 按位姿变化和时间间隔挑选关键帧；
3. 保存关键帧图像、ORB 特征和关键帧 JSON；
4. 对低质量片段给出可用于下一步重定位的关键帧候选；
5. 使用 ORB 匹配 + Essential Matrix + RANSAC 对候选关键帧做几何验证；
6. 输出 summary.json、tracking_quality.csv、tracking_quality.png 和 relocalization_results.json。

注意：第二版增强模块增加了 2D-2D 几何验证，但还不是完整的主动 PnP
重定位闭环。真正 PnP 需要把关键帧特征和 DPVO 三维地图点绑定起来，
形成 2D-3D 对应关系；当前版本先判断“这个弱跟踪片段能否和历史关键帧
形成稳定几何约束”。
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
import numpy as np


REPO = Path("/home/ros/ros2_orbslam3")
DEFAULT_VIDEO = REPO / "resources" / "input_video.mp4_bev.mp4"
DEFAULT_TRAJECTORY = REPO / "Opensource code" / "DPVO-main" / "saved_trajectories" / "mall_dpvo.txt"
DEFAULT_OUTPUT_DIR = REPO / "output" / "dpvo_enhanced" / "mall_dpvo"
DEFAULT_CALIB = REPO / "Opensource code" / "DPVO-main" / "calib" / "custom_mall.txt"


@dataclass
class PoseSample:
    """一条 DPVO TUM 轨迹样本。"""

    sample_index: int
    timestamp: float
    position: np.ndarray
    quaternion_xyzw: np.ndarray
    source_frame: int
    source_time_sec: float
    translation_step: float = 0.0
    rotation_step_deg: float = 0.0
    blur_laplacian: float = 0.0
    mask_ratio: float = 0.0
    stale_count: int = 0
    quality_score: float = 1.0
    quality_state: str = "good"


@dataclass
class CameraCalib:
    """相机内参和畸变参数。"""

    fx: float
    fy: float
    cx: float
    cy: float
    distortion: np.ndarray

    @property
    def matrix(self) -> np.ndarray:
        """返回原始分辨率下的 3x3 内参矩阵。"""
        return np.asarray(
            [
                [self.fx, 0.0, self.cx],
                [0.0, self.fy, self.cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )


def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(description="Enhance DPVO trajectory with keyframe memory")
    parser.add_argument("--trajectory", default=str(DEFAULT_TRAJECTORY), help="DPVO 保存的 TUM 轨迹文件")
    parser.add_argument("--video", default=str(DEFAULT_VIDEO), help="DPVO 使用的输入视频")
    parser.add_argument("--calib", default=str(DEFAULT_CALIB), help="DPVO 使用的相机标定文件")
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT_DIR), help="增强结果输出目录")
    parser.add_argument("--stride", type=int, default=2, help="DPVO video_stream 的 stride")
    parser.add_argument("--skip", type=int, default=0, help="DPVO video_stream 的 skip")
    parser.add_argument("--name", default="mall_dpvo", help="本次运行名称")
    parser.add_argument("--keyframe-max-gap", type=int, default=30, help="最多隔多少个 DPVO 样本强制插入关键帧")
    parser.add_argument("--keyframe-min-quality", type=float, default=0.35, help="低于该质量分数的帧不进入关键帧记忆")
    parser.add_argument("--translation-threshold", type=float, default=0.0, help="关键帧平移阈值；0 表示自动估计")
    parser.add_argument("--rotation-threshold-deg", type=float, default=0.0, help="关键帧旋转阈值；0 表示自动估计")
    parser.add_argument("--orb-features", type=int, default=1200, help="每个关键帧最多提取多少个 ORB 特征")
    parser.add_argument("--relocalization-topk", type=int, default=5, help="每个低质量片段保留几个重定位候选")
    parser.add_argument("--preview-width", type=int, default=960, help="保存关键帧图片时的最大宽度")
    parser.add_argument("--essential-threshold-px", type=float, default=1.5, help="Essential Matrix RANSAC 像素阈值")
    parser.add_argument("--min-geometric-inliers", type=int, default=35, help="通过几何验证所需的最少内点数量")
    parser.add_argument("--min-geometric-inlier-ratio", type=float, default=0.22, help="通过几何验证所需的最小内点比例")
    parser.add_argument("--save-visualization-video", dest="save_visualization_video", action="store_true",
                        default=True, help="保存增强可视化 MP4 视频")
    parser.add_argument("--no-visualization-video", dest="save_visualization_video", action="store_false",
                        help="不保存增强可视化 MP4 视频")
    parser.add_argument("--show", action="store_true", help="实时显示增强可视化窗口，按 q 或 Esc 退出")
    parser.add_argument("--visualization-fps", type=float, default=0.0,
                        help="增强可视化视频帧率；0 表示按输入视频 FPS/stride 自动估计")
    parser.add_argument("--visualization-every", type=int, default=1,
                        help="每隔多少个 DPVO 样本写一帧可视化视频")
    parser.add_argument("--visualization-width", type=int, default=1440,
                        help="增强可视化结果视频的最大宽度")
    parser.add_argument("--visualization-snapshot-count", type=int, default=12,
                        help="额外保存多少张可视化截图，便于不用打开视频也能快速查看")
    return parser.parse_args()


def normalize_quaternion(quaternion_xyzw: np.ndarray) -> np.ndarray:
    """归一化四元数，避免数值误差影响旋转角计算。"""
    norm = float(np.linalg.norm(quaternion_xyzw))
    if norm <= 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return quaternion_xyzw.astype(np.float64) / norm


def quaternion_angle_deg(q1_xyzw: np.ndarray, q2_xyzw: np.ndarray) -> float:
    """计算两个单位四元数之间的最小旋转角，单位为度。"""
    q1 = normalize_quaternion(q1_xyzw)
    q2 = normalize_quaternion(q2_xyzw)
    dot = abs(float(np.dot(q1, q2)))
    dot = min(1.0, max(-1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def clamp01(value: float) -> float:
    """把分数限制在 0 到 1。"""
    return min(1.0, max(0.0, float(value)))


def robust_median(values: Iterable[float], default: float = 0.0) -> float:
    """计算非零有限值中位数；没有可用数据时返回默认值。"""
    array = np.asarray([v for v in values if np.isfinite(v) and v > 1e-12], dtype=np.float64)
    if array.size == 0:
        return default
    return float(np.median(array))


def robust_mad(values: Iterable[float], default: float = 0.0) -> float:
    """计算 median absolute deviation，用于识别运动跳变。"""
    array = np.asarray([v for v in values if np.isfinite(v)], dtype=np.float64)
    if array.size == 0:
        return default
    median = float(np.median(array))
    mad = float(np.median(np.abs(array - median)))
    return mad if mad > 1e-12 else default


def read_camera_calib(path: Path) -> CameraCalib | None:
    """读取 DPVO 标定文件；文件不存在时返回 None。"""
    if not path.exists():
        print(f"未找到相机标定文件，几何验证会退化为估计焦距: {path}")
        return None
    values = np.loadtxt(path, delimiter=" ").astype(np.float64)
    if values.size < 4:
        raise ValueError(f"标定文件至少需要 fx fy cx cy 四个数: {path}")
    distortion = values[4:] if values.size > 4 else np.empty((0,), dtype=np.float64)
    return CameraCalib(
        fx=float(values[0]),
        fy=float(values[1]),
        cx=float(values[2]),
        cy=float(values[3]),
        distortion=distortion,
    )


def prepared_camera_matrix(
    calib: CameraCalib | None,
    original_shape: tuple[int, int],
    prepared_shape: tuple[int, int],
) -> np.ndarray:
    """估算预处理后图像对应的内参矩阵。"""
    prepared_height, prepared_width = prepared_shape
    original_height, original_width = original_shape

    if calib is None:
        focal = 0.75 * max(prepared_width, prepared_height)
        return np.asarray(
            [
                [focal, 0.0, prepared_width / 2.0],
                [0.0, focal, prepared_height / 2.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    half_width = int(round(original_width * 0.5))
    preview_scale = prepared_width / max(1.0, float(half_width))
    total_scale = 0.5 * preview_scale
    matrix = calib.matrix.copy()
    matrix[0, 0] *= total_scale
    matrix[1, 1] *= total_scale
    matrix[0, 2] *= total_scale
    matrix[1, 2] *= total_scale
    return matrix


def quaternion_to_rotation_matrix(quaternion_xyzw: np.ndarray) -> np.ndarray:
    """把 xyzw 四元数转成 3x3 旋转矩阵。"""
    x, y, z, w = normalize_quaternion(quaternion_xyzw)
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def rotation_matrix_angle_deg(rotation: np.ndarray) -> float:
    """计算旋转矩阵对应的旋转角，单位为度。"""
    trace_value = float(np.trace(rotation))
    cos_angle = (trace_value - 1.0) / 2.0
    cos_angle = min(1.0, max(-1.0, cos_angle))
    return math.degrees(math.acos(cos_angle))


def read_tum_trajectory(path: Path, stride: int, skip: int, fps: float) -> list[PoseSample]:
    """读取 TUM 轨迹文件，并估算每个 DPVO 样本对应的原始视频帧号。"""
    if not path.exists():
        raise FileNotFoundError(f"找不到 DPVO 轨迹文件: {path}")

    samples: list[PoseSample] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        values = [float(item) for item in line.split()]
        if len(values) != 8:
            raise ValueError(f"TUM 轨迹行应有 8 个数，实际为 {len(values)}: {line}")

        timestamp, x, y, z, qx, qy, qz, qw = values
        sample_index = int(round(timestamp))
        source_frame = max(0, skip + (sample_index + 1) * stride - 1)
        source_time_sec = source_frame / fps if fps > 1e-9 else float(sample_index)
        samples.append(
            PoseSample(
                sample_index=sample_index,
                timestamp=timestamp,
                position=np.asarray([x, y, z], dtype=np.float64),
                quaternion_xyzw=normalize_quaternion(np.asarray([qx, qy, qz, qw], dtype=np.float64)),
                source_frame=source_frame,
                source_time_sec=source_time_sec,
            )
        )

    if not samples:
        raise RuntimeError(f"轨迹文件为空: {path}")
    return samples


def extract_overlay_mask(frame_bgr: np.ndarray) -> np.ndarray:
    """识别遮挡视频中的黄/绿区域，用于估计遮挡比例。"""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(
        hsv,
        np.array([15, 70, 70], dtype=np.uint8),
        np.array([42, 255, 255], dtype=np.uint8),
    )
    green = cv2.inRange(
        hsv,
        np.array([40, 60, 60], dtype=np.uint8),
        np.array([95, 255, 255], dtype=np.uint8),
    )
    return cv2.bitwise_or(yellow, green)


def prepare_frame_like_dpvo(
    frame_bgr: np.ndarray,
    preview_width: int,
    calib: CameraCalib | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """按 DPVO stream.py 的思路缩放、裁剪，并把遮挡区域填成中性灰。"""
    if calib is not None and calib.distortion.size > 0:
        frame_bgr = cv2.undistort(frame_bgr, calib.matrix, calib.distortion)

    resized = cv2.resize(frame_bgr, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    height, width = resized.shape[:2]
    resized = resized[: height - height % 16, : width - width % 16]

    mask = extract_overlay_mask(resized)
    if np.any(mask):
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.dilate(mask, kernel)
        resized = resized.copy()
        resized[mask > 0] = 127

    if preview_width > 0 and resized.shape[1] > preview_width:
        scale = preview_width / float(resized.shape[1])
        resized = cv2.resize(
            resized,
            (preview_width, max(1, int(round(resized.shape[0] * scale)))),
            interpolation=cv2.INTER_AREA,
        )
        mask = cv2.resize(mask, (resized.shape[1], resized.shape[0]), interpolation=cv2.INTER_NEAREST)

    return resized, mask


def open_video(video_path: Path) -> tuple[cv2.VideoCapture, dict]:
    """打开输入视频并返回基础元数据。"""
    if not video_path.exists():
        raise FileNotFoundError(f"找不到输入视频: {video_path}")
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"OpenCV 无法打开视频: {video_path}")
    fps = float(cap.get(cv2.CAP_PROP_FPS) or 30.0)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
    return cap, {"fps": fps, "total_frames": total_frames, "width": width, "height": height}


def fill_motion_metrics(samples: list[PoseSample]) -> None:
    """根据相邻位姿计算平移、旋转和重复位姿计数。"""
    stale_count = 0
    for index, sample in enumerate(samples):
        if index == 0:
            continue
        prev = samples[index - 1]
        sample.translation_step = float(np.linalg.norm(sample.position - prev.position))
        sample.rotation_step_deg = quaternion_angle_deg(sample.quaternion_xyzw, prev.quaternion_xyzw)
        if sample.translation_step < 1e-8 and sample.rotation_step_deg < 1e-5:
            stale_count += 1
        else:
            stale_count = 0
        sample.stale_count = stale_count


def collect_image_metrics(
    samples: list[PoseSample],
    video_path: Path,
    preview_width: int,
    calib: CameraCalib | None,
) -> dict:
    """顺序读取视频，给每个 DPVO 样本补充模糊度和遮挡比例。"""
    cap, meta = open_video(video_path)
    by_frame = {sample.source_frame: sample for sample in samples}
    needed_frames = set(by_frame)
    current_frame = 0

    while current_frame <= max(needed_frames) if needed_frames else False:
        ok, frame = cap.read()
        if not ok:
            break
        if current_frame in needed_frames:
            prepared, mask = prepare_frame_like_dpvo(frame, preview_width=preview_width, calib=calib)
            gray = cv2.cvtColor(prepared, cv2.COLOR_BGR2GRAY)
            sample = by_frame[current_frame]
            sample.blur_laplacian = float(cv2.Laplacian(gray, cv2.CV_64F).var())
            sample.mask_ratio = float(np.count_nonzero(mask) / mask.size) if mask.size else 0.0
        current_frame += 1

    cap.release()
    return meta


def score_quality(samples: list[PoseSample]) -> dict:
    """把运动跳变、重复位姿、模糊和遮挡合成为工程质量分数。"""
    translations = [sample.translation_step for sample in samples[1:]]
    rotations = [sample.rotation_step_deg for sample in samples[1:]]
    blur_values = [sample.blur_laplacian for sample in samples if sample.blur_laplacian > 0]

    median_translation = robust_median(translations, default=1e-6)
    mad_translation = robust_mad(translations, default=median_translation)
    median_rotation = robust_median(rotations, default=1e-3)
    mad_rotation = robust_mad(rotations, default=median_rotation)
    blur_good = float(np.percentile(blur_values, 70)) if blur_values else 120.0
    blur_bad = max(1.0, float(np.percentile(blur_values, 20)) if blur_values else 30.0)

    translation_jump_limit = median_translation + 8.0 * mad_translation
    rotation_jump_limit = median_rotation + 8.0 * mad_rotation
    translation_jump_limit = max(translation_jump_limit, median_translation * 6.0)
    rotation_jump_limit = max(rotation_jump_limit, median_rotation * 6.0, 1.0)

    for sample in samples:
        if sample.blur_laplacian <= blur_bad:
            blur_score = 0.0
        else:
            blur_score = clamp01((sample.blur_laplacian - blur_bad) / max(1e-6, blur_good - blur_bad))

        translation_score = 1.0
        if sample.translation_step > translation_jump_limit:
            translation_score = clamp01(translation_jump_limit / max(sample.translation_step, 1e-9))

        rotation_score = 1.0
        if sample.rotation_step_deg > rotation_jump_limit:
            rotation_score = clamp01(rotation_jump_limit / max(sample.rotation_step_deg, 1e-9))

        motion_score = min(translation_score, rotation_score)
        mask_score = clamp01(1.0 - sample.mask_ratio / 0.55)
        stale_score = clamp01(1.0 - max(0, sample.stale_count - 2) / 8.0)

        sample.quality_score = clamp01(
            0.35 * blur_score
            + 0.25 * motion_score
            + 0.20 * mask_score
            + 0.20 * stale_score
        )
        if sample.quality_score >= 0.65:
            sample.quality_state = "good"
        elif sample.quality_score >= 0.35:
            sample.quality_state = "weak"
        else:
            sample.quality_state = "lost"

    return {
        "median_translation_step": median_translation,
        "translation_jump_limit": translation_jump_limit,
        "median_rotation_step_deg": median_rotation,
        "rotation_jump_limit_deg": rotation_jump_limit,
        "blur_bad": blur_bad,
        "blur_good": blur_good,
    }


def auto_keyframe_thresholds(
    samples: list[PoseSample],
    translation_threshold: float,
    rotation_threshold_deg: float,
) -> tuple[float, float]:
    """根据轨迹自动估计关键帧插入阈值。"""
    translations = [sample.translation_step for sample in samples[1:]]
    rotations = [sample.rotation_step_deg for sample in samples[1:]]
    median_translation = robust_median(translations, default=1e-5)
    median_rotation = robust_median(rotations, default=0.2)
    total_path = float(sum(translations))

    if translation_threshold <= 0:
        translation_threshold = max(median_translation * 12.0, total_path / 80.0, 1e-5)
    if rotation_threshold_deg <= 0:
        rotation_threshold_deg = max(median_rotation * 8.0, 3.0)
    return translation_threshold, rotation_threshold_deg


def select_keyframes(
    samples: list[PoseSample],
    keyframe_max_gap: int,
    keyframe_min_quality: float,
    translation_threshold: float,
    rotation_threshold_deg: float,
) -> list[dict]:
    """按质量、运动变化和最大间隔挑选关键帧。"""
    keyframes: list[dict] = []
    last_keyframe_sample: PoseSample | None = None

    for sample in samples:
        if not keyframes:
            reason = "first_frame"
            should_insert = True
        elif sample.quality_score < keyframe_min_quality:
            should_insert = False
            reason = "low_quality"
        else:
            assert last_keyframe_sample is not None
            delta_translation = float(np.linalg.norm(sample.position - last_keyframe_sample.position))
            delta_rotation = quaternion_angle_deg(sample.quaternion_xyzw, last_keyframe_sample.quaternion_xyzw)
            gap = sample.sample_index - last_keyframe_sample.sample_index
            reasons = []
            if delta_translation >= translation_threshold:
                reasons.append("translation")
            if delta_rotation >= rotation_threshold_deg:
                reasons.append("rotation")
            if gap >= keyframe_max_gap:
                reasons.append("max_gap")
            should_insert = bool(reasons)
            reason = "+".join(reasons) if reasons else "skip"

        if should_insert:
            keyframe = {
                "keyframe_id": len(keyframes),
                "sample_index": sample.sample_index,
                "source_frame": sample.source_frame,
                "source_time_sec": sample.source_time_sec,
                "position_xyz": sample.position.tolist(),
                "quaternion_xyzw": sample.quaternion_xyzw.tolist(),
                "quality_score": sample.quality_score,
                "quality_state": sample.quality_state,
                "mask_ratio": sample.mask_ratio,
                "blur_laplacian": sample.blur_laplacian,
                "reason": reason,
            }
            keyframes.append(keyframe)
            last_keyframe_sample = sample

    return keyframes


def read_video_frame(video_path: Path, source_frame: int) -> np.ndarray | None:
    """随机读取指定视频帧。"""
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(source_frame))
    ok, frame = cap.read()
    cap.release()
    return frame if ok else None


def save_keyframe_memory(
    keyframes: list[dict],
    video_path: Path,
    output_dir: Path,
    orb_features: int,
    preview_width: int,
    calib: CameraCalib | None,
) -> dict[int, dict]:
    """保存关键帧图片、ORB 特征和描述子。"""
    image_dir = output_dir / "keyframes" / "images"
    feature_dir = output_dir / "keyframes" / "features"
    image_dir.mkdir(parents=True, exist_ok=True)
    feature_dir.mkdir(parents=True, exist_ok=True)

    orb = cv2.ORB_create(nfeatures=orb_features)
    feature_memory: dict[int, dict] = {}

    for keyframe in keyframes:
        frame = read_video_frame(video_path, keyframe["source_frame"])
        if frame is None:
            keyframe["image_path"] = ""
            keyframe["feature_path"] = ""
            keyframe["num_features"] = 0
            continue

        prepared, _mask = prepare_frame_like_dpvo(frame, preview_width=preview_width, calib=calib)
        gray = cv2.cvtColor(prepared, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = orb.detectAndCompute(gray, None)
        keypoints_xy = np.asarray([kp.pt for kp in keypoints], dtype=np.float32) if keypoints else np.empty((0, 2), dtype=np.float32)
        descriptors_array = descriptors if descriptors is not None else np.empty((0, 32), dtype=np.uint8)

        stem = f"keyframe_{keyframe['keyframe_id']:04d}_sample_{keyframe['sample_index']:06d}"
        image_path = image_dir / f"{stem}.png"
        feature_path = feature_dir / f"{stem}.npz"
        cv2.imwrite(str(image_path), prepared)
        np.savez_compressed(
            feature_path,
            keypoints_xy=keypoints_xy,
            descriptors=descriptors_array,
            sample_index=np.asarray([keyframe["sample_index"]], dtype=np.int32),
            source_frame=np.asarray([keyframe["source_frame"]], dtype=np.int32),
        )

        keyframe["image_path"] = str(image_path)
        keyframe["feature_path"] = str(feature_path)
        keyframe["num_features"] = int(len(keypoints_xy))
        feature_memory[keyframe["keyframe_id"]] = {
            "keypoints_xy": keypoints_xy,
            "descriptors": descriptors_array,
            "feature_path": str(feature_path),
            "num_features": int(len(keypoints_xy)),
        }

    (output_dir / "keyframes" / "keyframes.json").write_text(
        json.dumps(keyframes, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return feature_memory


def group_quality_events(samples: list[PoseSample]) -> list[dict]:
    """把 weak/lost 连续片段整理成事件。"""
    events: list[dict] = []
    current: dict | None = None
    for sample in samples:
        is_event = sample.quality_state in {"weak", "lost"}
        if is_event and current is None:
            current = {
                "event_id": len(events),
                "state": sample.quality_state,
                "start_sample": sample.sample_index,
                "start_source_frame": sample.source_frame,
                "start_time_sec": sample.source_time_sec,
                "end_sample": sample.sample_index,
                "end_source_frame": sample.source_frame,
                "end_time_sec": sample.source_time_sec,
                "min_quality_score": sample.quality_score,
                "max_stale_count": sample.stale_count,
            }
        elif is_event and current is not None:
            current["state"] = "lost" if sample.quality_state == "lost" or current["state"] == "lost" else "weak"
            current["end_sample"] = sample.sample_index
            current["end_source_frame"] = sample.source_frame
            current["end_time_sec"] = sample.source_time_sec
            current["min_quality_score"] = min(current["min_quality_score"], sample.quality_score)
            current["max_stale_count"] = max(current["max_stale_count"], sample.stale_count)
        elif not is_event and current is not None:
            events.append(current)
            current = None
    if current is not None:
        events.append(current)
    return events


def extract_orb_features(
    frame_bgr: np.ndarray,
    orb_features: int,
    preview_width: int,
    calib: CameraCalib | None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """为当前帧计算 ORB 关键点、描述子和预处理后的图像。"""
    prepared, _mask = prepare_frame_like_dpvo(frame_bgr, preview_width=preview_width, calib=calib)
    gray = cv2.cvtColor(prepared, cv2.COLOR_BGR2GRAY)
    orb = cv2.ORB_create(nfeatures=orb_features)
    keypoints, descriptors = orb.detectAndCompute(gray, None)
    keypoints_xy = np.asarray([kp.pt for kp in keypoints], dtype=np.float32) if keypoints else np.empty((0, 2), dtype=np.float32)
    descriptors_array = descriptors if descriptors is not None else np.empty((0, 32), dtype=np.uint8)
    return keypoints_xy, descriptors_array, prepared


def extract_orb_descriptors(
    frame_bgr: np.ndarray,
    orb_features: int,
    preview_width: int,
    calib: CameraCalib | None,
) -> np.ndarray:
    """为重定位候选计算当前帧 ORB 描述子。"""
    _keypoints_xy, descriptors, _prepared = extract_orb_features(
        frame_bgr,
        orb_features=orb_features,
        preview_width=preview_width,
        calib=calib,
    )
    return descriptors


def compute_relocalization_candidates(
    events: list[dict],
    keyframes: list[dict],
    feature_memory: dict[int, dict],
    video_path: Path,
    topk: int,
    orb_features: int,
    preview_width: int,
    calib: CameraCalib | None,
) -> list[dict]:
    """用 ORB 匹配数为低质量片段推荐候选关键帧。"""
    if not events or not keyframes:
        return []

    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    candidates: list[dict] = []

    for event in events:
        frame = read_video_frame(video_path, event["start_source_frame"])
        if frame is None:
            event_candidates = []
        else:
            descriptors = extract_orb_descriptors(
                frame,
                orb_features=orb_features,
                preview_width=preview_width,
                calib=calib,
            )
            event_candidates = []
            for keyframe in keyframes:
                memory = feature_memory.get(keyframe["keyframe_id"], {})
                key_desc = memory.get("descriptors")
                if descriptors.size == 0 or key_desc is None or key_desc.size == 0:
                    good_matches = 0
                else:
                    raw_matches = matcher.knnMatch(descriptors, key_desc, k=2)
                    good_matches = 0
                    for pair in raw_matches:
                        if len(pair) != 2:
                            continue
                        first, second = pair
                        if first.distance < 0.75 * second.distance:
                            good_matches += 1
                pose_distance = float(
                    np.linalg.norm(
                        np.asarray(keyframe["position_xyz"], dtype=np.float64)
                        - np.asarray(keyframes[0]["position_xyz"], dtype=np.float64)
                    )
                )
                event_candidates.append(
                    {
                        "keyframe_id": keyframe["keyframe_id"],
                        "sample_index": keyframe["sample_index"],
                        "source_frame": keyframe["source_frame"],
                        "good_orb_matches": int(good_matches),
                        "keyframe_quality_score": keyframe["quality_score"],
                        "rough_pose_distance_from_first": pose_distance,
                        "image_path": keyframe.get("image_path", ""),
                    }
                )
            event_candidates.sort(
                key=lambda item: (item["good_orb_matches"], item["keyframe_quality_score"]),
                reverse=True,
            )
            event_candidates = event_candidates[:topk]

        candidates.append(
            {
                "event_id": event["event_id"],
                "event_state": event["state"],
                "start_sample": event["start_sample"],
                "start_source_frame": event["start_source_frame"],
                "candidates": event_candidates,
                "note": "这是基于 ORB 描述子匹配的重定位候选，不等同于已经完成 PnP 重定位。",
            }
        )

    return candidates


def match_orb_descriptors(
    query_descriptors: np.ndarray,
    key_descriptors: np.ndarray,
    ratio: float = 0.75,
) -> list[cv2.DMatch]:
    """使用 KNN + Lowe ratio test 匹配 ORB 描述子。"""
    if query_descriptors.size == 0 or key_descriptors.size == 0:
        return []
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    raw_matches = matcher.knnMatch(query_descriptors, key_descriptors, k=2)
    good_matches = []
    for pair in raw_matches:
        if len(pair) != 2:
            continue
        first, second = pair
        if first.distance < ratio * second.distance:
            good_matches.append(first)
    return good_matches


def load_keyframe_feature(feature_path: str) -> tuple[np.ndarray, np.ndarray]:
    """读取关键帧保存的二维关键点和 ORB 描述子。"""
    if not feature_path:
        return np.empty((0, 2), dtype=np.float32), np.empty((0, 32), dtype=np.uint8)
    path = Path(feature_path)
    if not path.exists():
        return np.empty((0, 2), dtype=np.float32), np.empty((0, 32), dtype=np.uint8)
    with np.load(path, allow_pickle=False) as data:
        keypoints_xy = data["keypoints_xy"].astype(np.float32)
        descriptors = data["descriptors"].astype(np.uint8)
    return keypoints_xy, descriptors


def verify_candidate_geometry(
    event: dict,
    candidate: dict,
    keyframe: dict,
    sample_by_index: dict[int, PoseSample],
    query_keypoints_xy: np.ndarray,
    query_descriptors: np.ndarray,
    camera_matrix: np.ndarray,
    essential_threshold_px: float,
    min_inliers: int,
    min_inlier_ratio: float,
) -> dict:
    """对一个候选关键帧执行 ORB 匹配和 Essential Matrix 几何验证。"""
    keypoints_xy, key_descriptors = load_keyframe_feature(keyframe.get("feature_path", ""))
    matches = match_orb_descriptors(query_descriptors, key_descriptors)
    result = {
        "keyframe_id": candidate.get("keyframe_id"),
        "keyframe_sample_index": candidate.get("sample_index"),
        "keyframe_source_frame": candidate.get("source_frame"),
        "candidate_good_orb_matches": int(candidate.get("good_orb_matches", 0)),
        "matched_features": int(len(matches)),
        "essential_inliers": 0,
        "essential_inlier_ratio": 0.0,
        "recover_pose_inliers": 0,
        "verified": False,
        "status": "not_enough_matches",
        "recovered_rotation_deg": None,
        "dpvo_relative_rotation_deg": None,
        "rotation_magnitude_error_deg": None,
        "dpvo_relative_translation_norm": None,
        "keyframe_quality_score": candidate.get("keyframe_quality_score"),
        "image_path": candidate.get("image_path", ""),
    }

    if len(matches) < 8:
        return result

    key_points = np.asarray([keypoints_xy[match.trainIdx] for match in matches], dtype=np.float32)
    query_points = np.asarray([query_keypoints_xy[match.queryIdx] for match in matches], dtype=np.float32)

    essential, inlier_mask = cv2.findEssentialMat(
        key_points,
        query_points,
        camera_matrix,
        method=cv2.RANSAC,
        prob=0.999,
        threshold=essential_threshold_px,
    )
    if essential is None or inlier_mask is None:
        result["status"] = "essential_failed"
        return result

    if essential.shape[0] > 3:
        essential = essential[:3, :]

    essential_inliers = int(np.count_nonzero(inlier_mask))
    inlier_ratio = essential_inliers / max(1, len(matches))
    result["essential_inliers"] = essential_inliers
    result["essential_inlier_ratio"] = float(inlier_ratio)

    if essential_inliers < 5:
        result["status"] = "too_few_essential_inliers"
        return result

    try:
        recover_inliers, recovered_rotation, recovered_translation, _pose_mask = cv2.recoverPose(
            essential,
            key_points,
            query_points,
            camera_matrix,
            mask=inlier_mask,
        )
    except cv2.error as exc:
        result["status"] = f"recover_pose_failed: {exc}"
        return result

    result["recover_pose_inliers"] = int(recover_inliers)
    recovered_rotation_deg = rotation_matrix_angle_deg(recovered_rotation)
    result["recovered_rotation_deg"] = float(recovered_rotation_deg)

    query_sample = sample_by_index.get(int(event["start_sample"]))
    if query_sample is not None:
        key_quaternion = np.asarray(keyframe["quaternion_xyzw"], dtype=np.float64)
        dpvo_rotation_deg = quaternion_angle_deg(key_quaternion, query_sample.quaternion_xyzw)
        dpvo_translation_norm = float(
            np.linalg.norm(np.asarray(keyframe["position_xyz"], dtype=np.float64) - query_sample.position)
        )
        result["dpvo_relative_rotation_deg"] = float(dpvo_rotation_deg)
        result["rotation_magnitude_error_deg"] = float(abs(recovered_rotation_deg - dpvo_rotation_deg))
        result["dpvo_relative_translation_norm"] = dpvo_translation_norm

    verified = (
        essential_inliers >= min_inliers
        and inlier_ratio >= min_inlier_ratio
        and int(recover_inliers) >= min_inliers
    )
    result["verified"] = bool(verified)
    if verified:
        result["status"] = "verified"
    elif essential_inliers >= min_inliers and inlier_ratio >= min_inlier_ratio:
        result["status"] = "pose_degenerate_or_low_parallax"
    else:
        result["status"] = "geometry_weak"
    return result


def compute_geometric_relocalization_results(
    relocalization_candidates: list[dict],
    keyframes: list[dict],
    samples: list[PoseSample],
    video_path: Path,
    calib: CameraCalib | None,
    orb_features: int,
    preview_width: int,
    essential_threshold_px: float,
    min_inliers: int,
    min_inlier_ratio: float,
) -> list[dict]:
    """对候选关键帧做 2D-2D 几何验证，输出第二版重定位结果。"""
    keyframe_by_id = {int(keyframe["keyframe_id"]): keyframe for keyframe in keyframes}
    sample_by_index = {int(sample.sample_index): sample for sample in samples}
    results: list[dict] = []

    for item in relocalization_candidates:
        event_id = int(item["event_id"])
        source_frame = int(item["start_source_frame"])
        frame = read_video_frame(video_path, source_frame)
        if frame is None:
            results.append(
                {
                    "event_id": event_id,
                    "event_state": item.get("event_state"),
                    "start_sample": item.get("start_sample"),
                    "start_source_frame": source_frame,
                    "best_status": "frame_read_failed",
                    "verified": False,
                    "best_candidate": None,
                    "candidates": [],
                }
            )
            continue

        query_keypoints_xy, query_descriptors, prepared = extract_orb_features(
            frame,
            orb_features=orb_features,
            preview_width=preview_width,
            calib=calib,
        )
        camera_matrix = prepared_camera_matrix(
            calib,
            original_shape=frame.shape[:2],
            prepared_shape=prepared.shape[:2],
        )

        candidate_results = []
        for candidate in item.get("candidates", []):
            keyframe = keyframe_by_id.get(int(candidate["keyframe_id"]))
            if keyframe is None:
                continue
            candidate_results.append(
                verify_candidate_geometry(
                    item,
                    candidate,
                    keyframe,
                    sample_by_index,
                    query_keypoints_xy,
                    query_descriptors,
                    camera_matrix,
                    essential_threshold_px=essential_threshold_px,
                    min_inliers=min_inliers,
                    min_inlier_ratio=min_inlier_ratio,
                )
            )

        candidate_results.sort(
            key=lambda result: (
                result["verified"],
                result["essential_inliers"],
                result["essential_inlier_ratio"],
                result["matched_features"],
            ),
            reverse=True,
        )
        best_candidate = candidate_results[0] if candidate_results else None
        results.append(
            {
                "event_id": event_id,
                "event_state": item.get("event_state"),
                "start_sample": item.get("start_sample"),
                "start_source_frame": source_frame,
                "query_num_features": int(len(query_keypoints_xy)),
                "camera_matrix": camera_matrix.tolist(),
                "verified": bool(best_candidate and best_candidate["verified"]),
                "best_status": best_candidate["status"] if best_candidate else "no_candidate",
                "best_candidate": best_candidate,
                "candidates": candidate_results,
                "note": (
                    "第二版使用 ORB + Essential Matrix + RANSAC 做 2D-2D 几何验证。"
                    "这不是完整 PnP；完整 PnP 需要关键帧特征与三维地图点的关联。"
                ),
            }
        )

    return results


def write_relocalization_outputs(results: list[dict], output_dir: Path) -> dict:
    """保存第二版几何重定位验证结果。"""
    json_path = output_dir / "relocalization_results.json"
    csv_path = output_dir / "relocalization_results.csv"
    text_path = output_dir / "relocalization_report.txt"
    json_path.write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding="utf-8")

    rows = []
    for item in results:
        best = item.get("best_candidate") or {}
        rows.append(
            {
                "event_id": item.get("event_id"),
                "event_state": item.get("event_state"),
                "start_sample": item.get("start_sample"),
                "start_source_frame": item.get("start_source_frame"),
                "verified": item.get("verified"),
                "best_status": item.get("best_status"),
                "best_keyframe_id": best.get("keyframe_id"),
                "best_keyframe_sample_index": best.get("keyframe_sample_index"),
                "matched_features": best.get("matched_features"),
                "essential_inliers": best.get("essential_inliers"),
                "essential_inlier_ratio": best.get("essential_inlier_ratio"),
                "recover_pose_inliers": best.get("recover_pose_inliers"),
                "recovered_rotation_deg": best.get("recovered_rotation_deg"),
                "dpvo_relative_rotation_deg": best.get("dpvo_relative_rotation_deg"),
                "rotation_magnitude_error_deg": best.get("rotation_magnitude_error_deg"),
            }
        )

    if rows:
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

    verified_count = sum(1 for item in results if item.get("verified"))
    lines = [
        "DPVO 第二版增强：几何重定位验证报告",
        "======================================",
        f"事件总数: {len(results)}",
        f"几何验证通过事件数: {verified_count}",
        f"几何验证通过比例: {verified_count / max(1, len(results)):.3f}",
        "",
        "验证含义：通过表示该 weak/lost 片段和某个关键帧之间存在稳定 2D-2D 几何约束。",
        "注意：这还不是完整 PnP 位姿修正；完整 PnP 需要关键帧特征关联三维地图点。",
        "",
        "前 10 个事件的最佳候选：",
    ]
    for item in results[:10]:
        best = item.get("best_candidate") or {}
        lines.append(
            "event={event_id}, verified={verified}, status={status}, keyframe={keyframe}, "
            "matches={matches}, inliers={inliers}, ratio={ratio}".format(
                event_id=item.get("event_id"),
                verified=item.get("verified"),
                status=item.get("best_status"),
                keyframe=best.get("keyframe_id"),
                matches=best.get("matched_features"),
                inliers=best.get("essential_inliers"),
                ratio=best.get("essential_inlier_ratio"),
            )
        )
    text_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return {
        "relocalization_results_json": str(json_path),
        "relocalization_results_csv": str(csv_path),
        "relocalization_report_txt": str(text_path),
        "geometric_verified_events": verified_count,
        "geometric_total_events": len(results),
        "geometric_verified_ratio": verified_count / max(1, len(results)),
    }


def color_for_state(state: str) -> tuple[int, int, int]:
    """返回 OpenCV BGR 颜色，用于在视频里标出跟踪状态。"""
    if state == "good":
        return (80, 210, 100)
    if state == "weak":
        return (0, 180, 255)
    if state == "lost":
        return (80, 80, 230)
    return (220, 220, 220)


def draw_text_box(
    image: np.ndarray,
    text: str,
    origin: tuple[int, int],
    color: tuple[int, int, int] = (235, 235, 235),
    scale: float = 0.55,
    thickness: int = 1,
) -> None:
    """画带半透明背景的英文说明文字。"""
    x, y = origin
    font = cv2.FONT_HERSHEY_SIMPLEX
    (width, height), baseline = cv2.getTextSize(text, font, scale, thickness)
    x0 = max(0, x - 5)
    y0 = max(0, y - height - 6)
    x1 = min(image.shape[1], x + width + 5)
    y1 = min(image.shape[0], y + baseline + 5)
    overlay = image.copy()
    cv2.rectangle(overlay, (x0, y0), (x1, y1), (22, 22, 22), -1)
    cv2.addWeighted(overlay, 0.62, image, 0.38, 0.0, image)
    cv2.putText(image, text, (x, y), font, scale, color, thickness, cv2.LINE_AA)


def draw_panel_line(
    panel: np.ndarray,
    text: str,
    y: int,
    color: tuple[int, int, int] = (230, 230, 230),
    scale: float = 0.52,
) -> int:
    """在右侧信息面板画一行文字，并返回下一行 y 坐标。"""
    cv2.putText(panel, text, (16, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)
    return y + int(24 * scale / 0.52)


def format_float(value: object, digits: int = 3) -> str:
    """把数字安全地格式化成短字符串。"""
    if value is None:
        return "n/a"
    try:
        value_float = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not np.isfinite(value_float):
        return "n/a"
    return f"{value_float:.{digits}f}"


def build_event_lookup(events: list[dict]) -> dict[int, dict]:
    """把 weak/lost 事件展开成 sample_index -> event 的查询表。"""
    lookup: dict[int, dict] = {}
    for event in events:
        start = int(event.get("start_sample", 0))
        end = int(event.get("end_sample", start))
        for sample_index in range(start, end + 1):
            lookup[sample_index] = event
    return lookup


def trajectory_points_xz(samples: list[PoseSample]) -> np.ndarray:
    """取轨迹的 x/z 平面坐标，用于画俯视小地图。"""
    return np.asarray([[sample.position[0], sample.position[2]] for sample in samples], dtype=np.float64)


def project_trajectory_points(
    points: np.ndarray,
    rect: tuple[int, int, int, int],
) -> np.ndarray:
    """把 x/z 轨迹点缩放到面板中的矩形区域。"""
    x0, y0, width, height = rect
    pad = 16
    if points.size == 0:
        return np.empty((0, 2), dtype=np.int32)

    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    span = maxs - mins
    span[span < 1e-9] = 1.0

    xs = x0 + pad + (points[:, 0] - mins[0]) / span[0] * max(1, width - 2 * pad)
    ys = y0 + height - pad - (points[:, 1] - mins[1]) / span[1] * max(1, height - 2 * pad)
    return np.stack([xs, ys], axis=1).astype(np.int32)


def draw_trajectory_map(
    panel: np.ndarray,
    samples: list[PoseSample],
    current_index: int,
    keyframes: list[dict],
    best_candidate: dict | None,
    rect: tuple[int, int, int, int],
) -> None:
    """在右侧面板画 DPVO 轨迹、关键帧和当前帧位置。"""
    x0, y0, width, height = rect
    cv2.rectangle(panel, (x0, y0), (x0 + width, y0 + height), (70, 70, 70), 1)
    cv2.putText(panel, "trajectory x-z", (x0 + 10, y0 + 20), cv2.FONT_HERSHEY_SIMPLEX,
                0.48, (210, 210, 210), 1, cv2.LINE_AA)

    points = trajectory_points_xz(samples)
    projected = project_trajectory_points(points, rect)
    if len(projected) >= 2:
        cv2.polylines(panel, [projected], False, (80, 80, 80), 1, cv2.LINE_AA)
        cv2.polylines(panel, [projected[: current_index + 1]], False, (230, 170, 70), 2, cv2.LINE_AA)

    sample_to_projected = {
        int(sample.sample_index): projected[index]
        for index, sample in enumerate(samples)
        if index < len(projected)
    }
    for keyframe in keyframes:
        point = sample_to_projected.get(int(keyframe["sample_index"]))
        if point is not None:
            cv2.circle(panel, tuple(point), 2, (80, 120, 240), -1, cv2.LINE_AA)

    if best_candidate:
        point = sample_to_projected.get(int(best_candidate.get("keyframe_sample_index", -1)))
        if point is not None:
            cv2.circle(panel, tuple(point), 7, (220, 80, 220), 2, cv2.LINE_AA)

    if 0 <= current_index < len(projected):
        cv2.circle(panel, tuple(projected[current_index]), 7, (80, 240, 240), -1, cv2.LINE_AA)


def draw_quality_bar(
    panel: np.ndarray,
    sample: PoseSample,
    rect: tuple[int, int, int, int],
) -> None:
    """画当前帧质量分数条。"""
    x0, y0, width, height = rect
    cv2.rectangle(panel, (x0, y0), (x0 + width, y0 + height), (70, 70, 70), 1)
    cv2.putText(panel, "quality", (x0, y0 - 8), cv2.FONT_HERSHEY_SIMPLEX,
                0.48, (210, 210, 210), 1, cv2.LINE_AA)
    fill_width = int(width * clamp01(sample.quality_score))
    cv2.rectangle(panel, (x0 + 2, y0 + 2), (x0 + fill_width - 2, y0 + height - 2),
                  color_for_state(sample.quality_state), -1)
    cv2.line(panel, (x0 + int(width * 0.35), y0), (x0 + int(width * 0.35), y0 + height),
             (70, 70, 230), 1)
    cv2.line(panel, (x0 + int(width * 0.65), y0), (x0 + int(width * 0.65), y0 + height),
             (70, 220, 90), 1)
    cv2.putText(panel, f"{sample.quality_score:.3f}", (x0 + 8, y0 + height - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (20, 20, 20), 1, cv2.LINE_AA)


def resize_to_fit_width(image: np.ndarray, max_width: int) -> np.ndarray:
    """把可视化画布缩放到指定最大宽度，并保证尺寸适合视频编码。"""
    if max_width > 0 and image.shape[1] > max_width:
        scale = max_width / float(image.shape[1])
        image = cv2.resize(
            image,
            (max_width, max(1, int(round(image.shape[0] * scale)))),
            interpolation=cv2.INTER_AREA,
        )
    height, width = image.shape[:2]
    if width % 2:
        image = image[:, :-1]
    if height % 2:
        image = image[:-1, :]
    return image


def load_thumbnail(
    image_path: str,
    cache: dict[str, np.ndarray],
    width: int,
    height: int,
) -> np.ndarray | None:
    """读取并缓存候选关键帧缩略图。"""
    if not image_path:
        return None
    if image_path not in cache:
        image = cv2.imread(image_path)
        if image is None:
            cache[image_path] = np.empty((0, 0, 3), dtype=np.uint8)
        else:
            cache[image_path] = image
    image = cache[image_path]
    if image.size == 0:
        return None
    scale = min(width / image.shape[1], height / image.shape[0])
    resized = cv2.resize(
        image,
        (max(1, int(image.shape[1] * scale)), max(1, int(image.shape[0] * scale))),
        interpolation=cv2.INTER_AREA,
    )
    canvas = np.full((height, width, 3), (25, 25, 25), dtype=np.uint8)
    y0 = (height - resized.shape[0]) // 2
    x0 = (width - resized.shape[1]) // 2
    canvas[y0:y0 + resized.shape[0], x0:x0 + resized.shape[1]] = resized
    return canvas


def render_visualization_canvas(
    frame_bgr: np.ndarray,
    sample: PoseSample,
    sample_index_in_list: int,
    samples: list[PoseSample],
    keyframes: list[dict],
    event: dict | None,
    relocalization_result: dict | None,
    preview_width: int,
    visualization_width: int,
    calib: CameraCalib | None,
    thumbnail_cache: dict[str, np.ndarray],
) -> np.ndarray:
    """把当前图像、跟踪状态、轨迹和重定位结果合成到一张可视化画布。"""
    prepared, mask = prepare_frame_like_dpvo(frame_bgr, preview_width=preview_width, calib=calib)
    state_color = color_for_state(sample.quality_state)
    frame_with_overlay = prepared.copy()
    cv2.rectangle(
        frame_with_overlay,
        (0, 0),
        (frame_with_overlay.shape[1] - 1, frame_with_overlay.shape[0] - 1),
        state_color,
        6,
    )
    draw_text_box(
        frame_with_overlay,
        f"sample {sample.sample_index} | frame {sample.source_frame} | state {sample.quality_state}",
        (16, 32),
        state_color,
        scale=0.62,
        thickness=2,
    )
    draw_text_box(
        frame_with_overlay,
        f"quality {sample.quality_score:.3f} | blur {sample.blur_laplacian:.1f} | mask {sample.mask_ratio:.3f}",
        (16, 62),
        (235, 235, 235),
        scale=0.54,
    )
    if event:
        draw_text_box(
            frame_with_overlay,
            f"event {event['event_id']} | {event['state']} | {event['start_sample']}->{event['end_sample']}",
            (16, 92),
            (0, 210, 255),
            scale=0.54,
        )

    if np.any(mask):
        mask_color = np.zeros_like(frame_with_overlay)
        mask_color[:, :, 1] = 180
        frame_with_overlay = np.where(mask[:, :, None] > 0,
                                      cv2.addWeighted(frame_with_overlay, 0.65, mask_color, 0.35, 0.0),
                                      frame_with_overlay)

    side_width = 460
    canvas_height = max(frame_with_overlay.shape[0], 700)
    canvas_width = frame_with_overlay.shape[1] + side_width
    canvas = np.full((canvas_height, canvas_width, 3), (18, 18, 18), dtype=np.uint8)
    canvas[: frame_with_overlay.shape[0], : frame_with_overlay.shape[1]] = frame_with_overlay
    panel = canvas[:, frame_with_overlay.shape[1]:]

    y = 34
    y = draw_panel_line(panel, "DPVO enhanced viewer", y, (245, 245, 245), scale=0.62)
    y = draw_panel_line(panel, f"time: {sample.source_time_sec:.2f}s", y)
    y = draw_panel_line(panel, f"state: {sample.quality_state}", y, state_color)
    y = draw_panel_line(panel, f"quality: {sample.quality_score:.3f}", y)
    y = draw_panel_line(panel, f"step trans: {sample.translation_step:.5f}", y)
    y = draw_panel_line(panel, f"step rot: {sample.rotation_step_deg:.2f} deg", y)
    y = draw_panel_line(panel, f"mask ratio: {sample.mask_ratio:.3f}", y)
    y = draw_panel_line(panel, f"blur lap: {sample.blur_laplacian:.1f}", y)
    y += 10

    draw_quality_bar(panel, sample, (16, y, side_width - 32, 30))
    y += 58

    best_candidate = None
    if relocalization_result:
        best_candidate = relocalization_result.get("best_candidate")
        verified = "yes" if relocalization_result.get("verified") else "no"
        y = draw_panel_line(panel, f"event id: {relocalization_result.get('event_id')}", y,
                            (0, 220, 255))
        y = draw_panel_line(panel, f"verified: {verified}", y,
                            (80, 230, 110) if relocalization_result.get("verified") else (90, 90, 230))
        y = draw_panel_line(panel, f"status: {relocalization_result.get('best_status')}", y)
        if best_candidate:
            y = draw_panel_line(panel, f"best kf: {best_candidate.get('keyframe_id')}", y)
            y = draw_panel_line(panel, f"matches: {best_candidate.get('matched_features')}", y)
            y = draw_panel_line(panel, f"inliers: {best_candidate.get('essential_inliers')}", y)
            y = draw_panel_line(panel, f"inlier ratio: {format_float(best_candidate.get('essential_inlier_ratio'))}", y)
    else:
        y = draw_panel_line(panel, "relocalization: stable/no event", y, (150, 220, 150))

    map_y = max(y + 10, 350)
    draw_trajectory_map(
        panel,
        samples,
        sample_index_in_list,
        keyframes,
        best_candidate,
        (16, map_y, side_width - 32, 210),
    )

    thumb_y = map_y + 230
    if best_candidate:
        thumbnail = load_thumbnail(
            str(best_candidate.get("image_path", "")),
            thumbnail_cache,
            width=side_width - 32,
            height=max(80, canvas_height - thumb_y - 20),
        )
        if thumbnail is not None and thumb_y + thumbnail.shape[0] <= panel.shape[0]:
            panel[thumb_y:thumb_y + thumbnail.shape[0], 16:16 + thumbnail.shape[1]] = thumbnail
            cv2.rectangle(panel, (16, thumb_y), (16 + thumbnail.shape[1], thumb_y + thumbnail.shape[0]),
                          (220, 80, 220), 1)
            cv2.putText(panel, "best keyframe", (24, thumb_y + 22), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (245, 245, 245), 1, cv2.LINE_AA)

    return resize_to_fit_width(canvas, visualization_width)


def select_snapshot_samples(samples: list[PoseSample], events: list[dict], count: int) -> set[int]:
    """选择少量可视化截图对应的 sample_index。"""
    if count <= 0 or not samples:
        return set()
    selected = {samples[0].sample_index, samples[-1].sample_index}
    for event in events[: max(0, count // 2)]:
        selected.add(int(event.get("start_sample", samples[0].sample_index)))
    if len(samples) > 2:
        for index in np.linspace(0, len(samples) - 1, num=count, dtype=np.int32):
            selected.add(samples[int(index)].sample_index)
    return set(list(selected)[: max(1, count)])


def write_visualization_outputs(
    samples: list[PoseSample],
    keyframes: list[dict],
    events: list[dict],
    relocalization_results: list[dict],
    video_path: Path,
    output_dir: Path,
    video_meta: dict,
    stride: int,
    preview_width: int,
    visualization_width: int,
    visualization_every: int,
    visualization_fps: float,
    snapshot_count: int,
    save_video: bool,
    show: bool,
    calib: CameraCalib | None,
) -> dict:
    """生成增强可视化 MP4、截图，并可选择实时显示窗口。"""
    visual_dir = output_dir / "visualization"
    snapshot_dir = visual_dir / "snapshots"
    visual_dir.mkdir(parents=True, exist_ok=True)
    snapshot_dir.mkdir(parents=True, exist_ok=True)

    event_lookup = build_event_lookup(events)
    relocalization_by_event = {int(item["event_id"]): item for item in relocalization_results}
    snapshot_samples = select_snapshot_samples(samples, events, snapshot_count)
    every = max(1, int(visualization_every))
    out_fps = float(visualization_fps)
    if out_fps <= 0:
        out_fps = max(1.0, float(video_meta.get("fps", 30.0)) / max(1, stride) / every)

    show_enabled = bool(show)
    if show_enabled and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        print("未检测到 DISPLAY/WAYLAND_DISPLAY，实时窗口已自动关闭；仍会保存视频文件。")
        show_enabled = False

    video_output_path = visual_dir / "dpvo_enhanced_visualization.mp4"
    writer: cv2.VideoWriter | None = None
    written_frames = 0
    written_snapshots = 0
    thumbnail_cache: dict[str, np.ndarray] = {}

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"OpenCV 无法打开视频用于可视化: {video_path}")

    current_frame = -1
    should_stop = False
    for sample_list_index, sample in enumerate(samples):
        need_frame = (
            sample_list_index % every == 0
            or sample.sample_index in snapshot_samples
            or sample.sample_index in event_lookup
        )
        if not need_frame:
            continue

        target_frame = int(sample.source_frame)
        if target_frame <= current_frame:
            cap.set(cv2.CAP_PROP_POS_FRAMES, target_frame)
            current_frame = target_frame - 1

        frame = None
        while current_frame < target_frame:
            ok, frame = cap.read()
            current_frame += 1
            if not ok:
                frame = None
                break
        if frame is None:
            break

        event = event_lookup.get(sample.sample_index)
        relocalization_result = None
        if event is not None:
            relocalization_result = relocalization_by_event.get(int(event["event_id"]))

        canvas = render_visualization_canvas(
            frame,
            sample,
            sample_list_index,
            samples,
            keyframes,
            event,
            relocalization_result,
            preview_width=preview_width,
            visualization_width=visualization_width,
            calib=calib,
            thumbnail_cache=thumbnail_cache,
        )

        if save_video:
            if writer is None:
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                writer = cv2.VideoWriter(str(video_output_path), fourcc, out_fps,
                                         (canvas.shape[1], canvas.shape[0]))
                if not writer.isOpened():
                    raise RuntimeError(f"无法创建增强可视化视频: {video_output_path}")
            writer.write(canvas)
            written_frames += 1

        if sample.sample_index in snapshot_samples:
            snapshot_path = snapshot_dir / f"sample_{sample.sample_index:06d}_frame_{sample.source_frame:06d}.jpg"
            if cv2.imwrite(str(snapshot_path), canvas):
                written_snapshots += 1
            snapshot_samples.discard(sample.sample_index)

        if show_enabled:
            try:
                cv2.imshow("DPVO Enhanced Viewer", canvas)
                key = cv2.waitKey(max(1, int(1000 / out_fps))) & 0xFF
                if key in (27, ord("q")):
                    should_stop = True
            except cv2.error as exc:
                print(f"实时窗口显示失败，已继续保存视频: {exc}")
                show_enabled = False
            if should_stop:
                break

    cap.release()
    if writer is not None:
        writer.release()
    if show_enabled:
        cv2.destroyWindow("DPVO Enhanced Viewer")

    stats = {
        "visualization_dir": str(visual_dir),
        "visualization_video": str(video_output_path) if save_video else "",
        "visualization_snapshots_dir": str(snapshot_dir),
        "visualization_frames": written_frames,
        "visualization_snapshots": written_snapshots,
        "visualization_fps": out_fps,
        "show_requested": bool(show),
        "show_enabled": bool(show_enabled),
        "note": "该可视化是 DPVO 增强后处理回放，不是对 DPVO 网络内部状态的实时侵入式修改。",
    }
    (visual_dir / "visualization_summary.json").write_text(
        json.dumps(stats, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    return stats


def write_quality_outputs(samples: list[PoseSample], output_dir: Path) -> None:
    """写出每帧质量 CSV 和 JSON。"""
    rows = []
    for sample in samples:
        rows.append(
            {
                "sample_index": sample.sample_index,
                "timestamp": sample.timestamp,
                "source_frame": sample.source_frame,
                "source_time_sec": sample.source_time_sec,
                "x": float(sample.position[0]),
                "y": float(sample.position[1]),
                "z": float(sample.position[2]),
                "translation_step": sample.translation_step,
                "rotation_step_deg": sample.rotation_step_deg,
                "blur_laplacian": sample.blur_laplacian,
                "mask_ratio": sample.mask_ratio,
                "stale_count": sample.stale_count,
                "quality_score": sample.quality_score,
                "quality_state": sample.quality_state,
            }
        )

    csv_path = output_dir / "tracking_quality.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    (output_dir / "tracking_quality.json").write_text(
        json.dumps(rows, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def plot_quality(samples: list[PoseSample], output_dir: Path) -> None:
    """生成质量曲线图；如果 matplotlib 不可用则跳过。"""
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:  # pragma: no cover - 运行环境缺图形库时允许跳过
        print(f"跳过 tracking_quality.png：matplotlib 不可用: {exc}")
        return

    xs = [sample.sample_index for sample in samples]
    quality = [sample.quality_score for sample in samples]
    mask = [sample.mask_ratio for sample in samples]
    blur = [sample.blur_laplacian for sample in samples]
    blur_norm = np.asarray(blur, dtype=np.float64)
    if np.max(blur_norm) > np.min(blur_norm):
        blur_norm = (blur_norm - np.min(blur_norm)) / (np.max(blur_norm) - np.min(blur_norm))
    else:
        blur_norm = np.zeros_like(blur_norm)

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(xs, quality, label="quality_score", color="#2b6cb0", linewidth=1.5)
    ax.plot(xs, mask, label="mask_ratio", color="#c05621", linewidth=1.0, alpha=0.8)
    ax.plot(xs, blur_norm, label="blur_laplacian_norm", color="#2f855a", linewidth=1.0, alpha=0.7)
    ax.axhline(0.65, color="#2f855a", linestyle="--", linewidth=0.8)
    ax.axhline(0.35, color="#c53030", linestyle="--", linewidth=0.8)
    ax.set_xlabel("DPVO sample index")
    ax.set_ylabel("score")
    ax.set_title("DPVO Enhanced Tracking Quality")
    ax.grid(True, alpha=0.2)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_dir / "tracking_quality.png", dpi=160)
    plt.close(fig)


def write_summary(
    output_dir: Path,
    args: argparse.Namespace,
    samples: list[PoseSample],
    video_meta: dict,
    thresholds: dict,
    keyframes: list[dict],
    events: list[dict],
    relocalization_candidates: list[dict],
    relocalization_stats: dict,
    visualization_stats: dict,
) -> None:
    """写出增强分析总摘要。"""
    state_counts = {
        "good": sum(1 for sample in samples if sample.quality_state == "good"),
        "weak": sum(1 for sample in samples if sample.quality_state == "weak"),
        "lost": sum(1 for sample in samples if sample.quality_state == "lost"),
    }
    path_length = float(sum(sample.translation_step for sample in samples))
    summary = {
        "name": args.name,
        "video": str(Path(args.video)),
        "trajectory": str(Path(args.trajectory)),
        "output_dir": str(output_dir),
        "mode": "dpvo_enhanced_keyframe_memory",
        "num_pose_samples": len(samples),
        "video_meta": video_meta,
        "stride": args.stride,
        "skip": args.skip,
        "path_length_dpvo_scale": path_length,
        "quality_state_counts": state_counts,
        "mean_quality_score": float(np.mean([sample.quality_score for sample in samples])),
        "median_quality_score": float(np.median([sample.quality_score for sample in samples])),
        "num_keyframes": len(keyframes),
        "num_quality_events": len(events),
        "num_relocalization_candidate_sets": len(relocalization_candidates),
        "num_geometric_verified_events": relocalization_stats.get("geometric_verified_events", 0),
        "geometric_verified_ratio": relocalization_stats.get("geometric_verified_ratio", 0.0),
        "thresholds": thresholds,
        "outputs": {
            "tracking_quality_csv": str(output_dir / "tracking_quality.csv"),
            "tracking_quality_json": str(output_dir / "tracking_quality.json"),
            "tracking_quality_plot": str(output_dir / "tracking_quality.png"),
            "keyframes_json": str(output_dir / "keyframes" / "keyframes.json"),
            "events_json": str(output_dir / "tracking_events.json"),
            "relocalization_candidates_json": str(output_dir / "relocalization_candidates.json"),
            "relocalization_results_json": relocalization_stats.get("relocalization_results_json", ""),
            "relocalization_results_csv": relocalization_stats.get("relocalization_results_csv", ""),
            "relocalization_report_txt": relocalization_stats.get("relocalization_report_txt", ""),
            "visualization_dir": visualization_stats.get("visualization_dir", ""),
            "visualization_video": visualization_stats.get("visualization_video", ""),
            "visualization_snapshots_dir": visualization_stats.get("visualization_snapshots_dir", ""),
            "visualization_summary_json": str(output_dir / "visualization" / "visualization_summary.json"),
        },
        "note": (
            "第二版增强模块提供关键帧记忆、质量评估、重定位候选和 2D-2D 几何验证；"
            "第三版增加了实时窗口回放和结果视频输出。它不修改 DPVO 官方模型，"
            "也还没有执行完整 2D-3D PnP 重定位闭环。"
        ),
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    summary_text = [
        "DPVO 增强分析完成",
        "==================",
        f"运行名称: {args.name}",
        f"输入视频: {args.video}",
        f"轨迹文件: {args.trajectory}",
        f"输出目录: {output_dir}",
        f"位姿样本数: {len(samples)}",
        f"关键帧数量: {len(keyframes)}",
        f"质量事件数量: {len(events)}",
        f"几何验证通过事件数: {relocalization_stats.get('geometric_verified_events', 0)}",
        f"几何验证通过比例: {relocalization_stats.get('geometric_verified_ratio', 0.0):.3f}",
        f"good/weak/lost: {state_counts}",
        f"平均质量分数: {summary['mean_quality_score']:.3f}",
        f"DPVO 尺度下路径长度: {path_length:.6f}",
        f"增强可视化视频: {visualization_stats.get('visualization_video', '')}",
        f"增强可视化截图目录: {visualization_stats.get('visualization_snapshots_dir', '')}",
        f"增强可视化帧数: {visualization_stats.get('visualization_frames', 0)}",
        "",
        "重要说明：本模块是 DPVO 后处理增强，不是重新训练 DPVO。",
        "当前第二版已经做 ORB + Essential Matrix + RANSAC 几何验证。",
        "当前第三版增加了可视化回放窗口和 MP4 结果视频。",
        "下一步可以把关键帧特征和 DPVO 三维点绑定起来，再接完整 PnP 和局部 BA。",
    ]
    (output_dir / "summary.txt").write_text("\n".join(summary_text) + "\n", encoding="utf-8")


def main() -> None:
    """脚本入口。"""
    args = parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(output_dir / ".matplotlib"))
    Path(os.environ["MPLCONFIGDIR"]).mkdir(parents=True, exist_ok=True)

    video_path = Path(args.video)
    trajectory_path = Path(args.trajectory)
    calib = read_camera_calib(Path(args.calib))
    cap, video_meta = open_video(video_path)
    cap.release()

    samples = read_tum_trajectory(
        trajectory_path,
        stride=max(1, args.stride),
        skip=max(0, args.skip),
        fps=float(video_meta["fps"]),
    )
    fill_motion_metrics(samples)
    collect_image_metrics(samples, video_path, preview_width=args.preview_width, calib=calib)
    quality_thresholds = score_quality(samples)
    translation_threshold, rotation_threshold_deg = auto_keyframe_thresholds(
        samples,
        translation_threshold=args.translation_threshold,
        rotation_threshold_deg=args.rotation_threshold_deg,
    )
    keyframes = select_keyframes(
        samples,
        keyframe_max_gap=max(1, args.keyframe_max_gap),
        keyframe_min_quality=args.keyframe_min_quality,
        translation_threshold=translation_threshold,
        rotation_threshold_deg=rotation_threshold_deg,
    )
    feature_memory = save_keyframe_memory(
        keyframes,
        video_path,
        output_dir,
        orb_features=args.orb_features,
        preview_width=args.preview_width,
        calib=calib,
    )
    events = group_quality_events(samples)
    relocalization_candidates = compute_relocalization_candidates(
        events,
        keyframes,
        feature_memory,
        video_path,
        topk=max(1, args.relocalization_topk),
        orb_features=args.orb_features,
        preview_width=args.preview_width,
        calib=calib,
    )
    relocalization_results = compute_geometric_relocalization_results(
        relocalization_candidates,
        keyframes,
        samples,
        video_path,
        calib=calib,
        orb_features=args.orb_features,
        preview_width=args.preview_width,
        essential_threshold_px=args.essential_threshold_px,
        min_inliers=args.min_geometric_inliers,
        min_inlier_ratio=args.min_geometric_inlier_ratio,
    )
    relocalization_stats = write_relocalization_outputs(relocalization_results, output_dir)

    write_quality_outputs(samples, output_dir)
    (output_dir / "tracking_events.json").write_text(
        json.dumps(events, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    (output_dir / "relocalization_candidates.json").write_text(
        json.dumps(relocalization_candidates, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    plot_quality(samples, output_dir)
    visualization_stats = write_visualization_outputs(
        samples,
        keyframes,
        events,
        relocalization_results,
        video_path,
        output_dir,
        video_meta,
        stride=max(1, args.stride),
        preview_width=args.preview_width,
        visualization_width=args.visualization_width,
        visualization_every=max(1, args.visualization_every),
        visualization_fps=args.visualization_fps,
        snapshot_count=args.visualization_snapshot_count,
        save_video=args.save_visualization_video,
        show=args.show,
        calib=calib,
    )

    thresholds = {
        **quality_thresholds,
        "keyframe_translation_threshold": translation_threshold,
        "keyframe_rotation_threshold_deg": rotation_threshold_deg,
        "keyframe_max_gap": args.keyframe_max_gap,
        "keyframe_min_quality": args.keyframe_min_quality,
    }
    write_summary(
        output_dir,
        args,
        samples,
        video_meta,
        thresholds,
        keyframes,
        events,
        relocalization_candidates,
        relocalization_stats,
        visualization_stats,
    )
    print(f"DPVO 增强分析完成，输出目录: {output_dir}")


if __name__ == "__main__":
    main()
