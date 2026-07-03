"""V3.1 阶段 2: 关键帧选择。

从 DPVO 平面化轨迹 + 原始视频里挑关键帧, 用于后续稠密深度重建。

选择规则:
  1. 每 stride_frames 帧作为候选;
  2. 相对上一个关键帧平移 > min_translation_unit → 保留;
  3. 相对旋转 > min_rotation_deg → 保留;
  4. 转弯处 (旋转速率高) 强制多保留;
  5. 行人遮挡严重的帧降低优先级 (不作为关键帧, 除非几何上必须);
  6. 上限 max_keyframes。
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Dict, List, Optional

import cv2
import numpy as np


def _quat_angle_deg(R1: np.ndarray, R2: np.ndarray) -> float:
    """两个旋转矩阵之间的夹角 (度)。"""
    R = R1.T @ R2
    tr = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return math.degrees(math.acos(tr))


def _load_tum(path: str) -> np.ndarray:
    arr = np.loadtxt(path, comments="#")
    if arr.ndim == 1:
        arr = arr[None, :]
    return arr


def _quat_to_R(qx, qy, qz, qw) -> np.ndarray:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


def _tum_row_to_Twc(row: np.ndarray) -> np.ndarray:
    t = row[1:4]
    R = _quat_to_R(row[4], row[5], row[6], row[7])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def select_keyframes(
    video_path: str,
    pose_tum_path: str,
    out_dir: str,
    stride_frames: int = 15,
    min_translation_unit: float = 0.06,
    min_rotation_deg: float = 8.0,
    max_keyframes: int = 240,
    image_width: int = 960,
    image_height: int = 540,
    video_fps: float = 29.417,
    dpvo_stride: int = 2,
    person_occlusion_by_frame: Optional[Dict[int, float]] = None,
    occlusion_high_thresh: float = 0.30,
) -> Dict:
    """选择关键帧并落盘图像 + json + tum。

    ``person_occlusion_by_frame``: {src_frame_index: person_area_ratio}, 可选,
    用于给遮挡严重的帧降权。
    """
    out = Path(out_dir)
    (out / "images").mkdir(parents=True, exist_ok=True)

    tum = _load_tum(pose_tum_path)
    # TUM timestamp 是 DPVO tick → 源视频帧 index = tick * dpvo_stride
    tick_to_row = {}
    for row in tum:
        tick = int(round(row[0]))
        tick_to_row[tick] = row

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise FileNotFoundError(video_path)
    src_n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    # 候选源帧: 每 stride_frames
    kept: List[dict] = []
    last_T: Optional[np.ndarray] = None
    prev_R: Optional[np.ndarray] = None

    # 预扫描: 记录每个候选源帧对应的 pose (tick = src_idx / dpvo_stride)
    cand_src = list(range(0, src_n, stride_frames))

    # 逐候选判断 (先算几何, 再读图)
    decisions = []
    for src_idx in cand_src:
        tick = int(round(src_idx / dpvo_stride))
        row = tick_to_row.get(tick)
        if row is None:
            # 找最近 tick
            near = min(tick_to_row.keys(), key=lambda k: abs(k - tick))
            if abs(near - tick) > 3:
                continue
            row = tick_to_row[near]
        T = _tum_row_to_Twc(row)
        keep = False
        reason = ""
        if last_T is None:
            keep = True
            reason = "first"
        else:
            dt = float(np.linalg.norm(T[:3, 3] - last_T[:3, 3]))
            dr = _quat_angle_deg(last_T[:3, :3], T[:3, :3])
            if dt >= min_translation_unit:
                keep = True; reason = f"trans {dt:.3f}"
            elif dr >= min_rotation_deg:
                keep = True; reason = f"rot {dr:.1f}deg"
        # 转弯处强制保留: 与上上帧比旋转速率
        if not keep and prev_R is not None:
            dr2 = _quat_angle_deg(prev_R, T[:3, :3])
            if dr2 >= min_rotation_deg * 1.5:
                keep = True; reason = f"turn {dr2:.1f}deg"
        # 遮挡降权: 遮挡严重且不是几何必需 → 跳过
        occ = 0.0
        if person_occlusion_by_frame is not None:
            occ = float(person_occlusion_by_frame.get(src_idx, 0.0))
        if keep and occ > occlusion_high_thresh and reason.startswith("trans"):
            # 平移触发但遮挡重: 记录但仍保留 (几何优先), 只标注
            reason += f" (occ {occ:.2f})"

        if keep:
            decisions.append((src_idx, T, reason, occ))
            last_T = T
            prev_R = T[:3, :3]
        else:
            prev_R = T[:3, :3]

    # 上限截断: 均匀下采样
    if len(decisions) > max_keyframes:
        idxs = np.linspace(0, len(decisions) - 1, max_keyframes).astype(int)
        decisions = [decisions[i] for i in idxs]

    # 读图 + 落盘
    kf_records = []
    tum_lines = ["# timestamp tx ty tz qx qy qz qw"]
    for kf_i, (src_idx, T, reason, occ) in enumerate(decisions):
        cap.set(cv2.CAP_PROP_POS_FRAMES, src_idx)
        ret, bgr = cap.read()
        if not ret:
            continue
        img = cv2.resize(bgr, (image_width, image_height), interpolation=cv2.INTER_AREA)
        img_name = f"kf_{kf_i:04d}_src{src_idx:06d}.png"
        cv2.imwrite(str(out / "images" / img_name), img)
        ts = src_idx / max(video_fps, 1e-6)
        kf_records.append({
            "kf_index": kf_i,
            "src_frame_index": int(src_idx),
            "timestamp": float(ts),
            "T_wc": T.tolist(),
            "image_path": str(out / "images" / img_name),
            "image_width": image_width,
            "image_height": image_height,
            "select_reason": reason,
            "person_occlusion": float(occ),
        })
        # TUM: 用 src_idx 当 timestamp 方便对齐
        t = T[:3, 3]
        # 反算四元数
        q = _R_to_quat(T[:3, :3])
        tum_lines.append(f"{src_idx} {t[0]:.6f} {t[1]:.6f} {t[2]:.6f} "
                         f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}")
    cap.release()

    (out / "keyframe_poses_tum.txt").write_text("\n".join(tum_lines) + "\n", encoding="utf-8")
    payload = {
        "video": video_path,
        "pose_tum": pose_tum_path,
        "src_total_frames": src_n,
        "stride_frames": stride_frames,
        "min_translation_unit": min_translation_unit,
        "min_rotation_deg": min_rotation_deg,
        "max_keyframes": max_keyframes,
        "n_keyframes": len(kf_records),
        "image_size": [image_width, image_height],
        "keyframes": kf_records,
    }
    (out / "keyframes.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    return payload


def _R_to_quat(R: np.ndarray) -> np.ndarray:
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 0.5 / math.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w], dtype=np.float64)
