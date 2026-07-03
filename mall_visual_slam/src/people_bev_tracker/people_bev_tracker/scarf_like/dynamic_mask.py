"""V3.1 阶段 3: 动态行人 mask。

从 people_tracks json 读取每帧行人 bbox (V2 json 里没存 per-pixel mask, 只有
bbox_xyxy + foot_pixel), 为每个关键帧生成一张 "行人无效区" mask, 供稠密建图时
把行人区域剔除, 避免行人被融合成静态假墙。

规则:
  1. bbox 膨胀 dilate_px (5-15) 后置为 invalid;
  2. bbox 下半部分额外扩一点 (脚下/影子);
  3. 多个行人取并集;
  4. 若某关键帧没有行人 → 全 valid;
  5. mask=255 表示行人 (invalid), 0 表示可用于静态建图。
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np


def load_people_bboxes_by_frame(people_json_path: str) -> Dict[int, List[dict]]:
    """{src_frame_index: [ {bbox_xyxy, score, track_id}, ... ]}."""
    with open(people_json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    out: Dict[int, List[dict]] = {}
    for fr in data.get("frames", []):
        fi = int(fr["frame_index"])
        people = []
        for p in fr.get("people", []):
            bb = p.get("bbox_xyxy")
            if bb is None:
                continue
            people.append({
                "bbox_xyxy": [float(x) for x in bb],
                "score": float(p.get("score", 0.0)),
                "track_id": int(p.get("track_id", -1)),
            })
        out[fi] = people
    return out


def person_occlusion_ratio_by_frame(
    people_by_frame: Dict[int, List[dict]],
    src_w: int,
    src_h: int,
) -> Dict[int, float]:
    """每帧行人 bbox 面积占图像比例 (给关键帧选择降权用)。"""
    area = float(max(1, src_w * src_h))
    out: Dict[int, float] = {}
    for fi, people in people_by_frame.items():
        s = 0.0
        for p in people:
            x1, y1, x2, y2 = p["bbox_xyxy"]
            s += max(0.0, (x2 - x1)) * max(0.0, (y2 - y1))
        out[fi] = min(1.0, s / area)
    return out


def build_person_mask(
    people: List[dict],
    src_w: int,
    src_h: int,
    kf_w: int,
    kf_h: int,
    dilate_px: int = 10,
    foot_extra_frac: float = 0.15,
) -> np.ndarray:
    """在关键帧尺寸 (kf_w, kf_h) 上生成行人 invalid mask (uint8 0/255)。

    people 的 bbox 是**原始视频尺寸** (src_w, src_h), 需缩放到关键帧尺寸。
    """
    mask = np.zeros((kf_h, kf_w), dtype=np.uint8)
    sx = kf_w / float(src_w)
    sy = kf_h / float(src_h)
    for p in people:
        x1, y1, x2, y2 = p["bbox_xyxy"]
        X1 = int(round(x1 * sx)); Y1 = int(round(y1 * sy))
        X2 = int(round(x2 * sx)); Y2 = int(round(y2 * sy))
        # 脚下多扩一点
        bh = Y2 - Y1
        Y2e = int(round(Y2 + bh * foot_extra_frac))
        cv2.rectangle(mask, (X1, Y1), (X2, Y2e), 255, -1)
    if dilate_px > 0 and mask.any():
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dilate_px, dilate_px))
        mask = cv2.dilate(mask, k, iterations=1)
    return mask


def build_all_masks(
    keyframes: List[dict],
    people_by_frame: Dict[int, List[dict]],
    src_w: int,
    src_h: int,
    out_dir: str,
    dilate_px: int = 10,
    save_debug: bool = True,
) -> Dict:
    """为所有关键帧生成 person mask, 落盘 npy + debug png。返回统计。"""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    if save_debug:
        (out / "debug").mkdir(parents=True, exist_ok=True)

    n_with_person = 0
    total_ratio = 0.0
    records = []
    for kf in keyframes:
        fi = int(kf["src_frame_index"])
        kf_w = int(kf["image_width"]); kf_h = int(kf["image_height"])
        people = people_by_frame.get(fi, [])
        mask = build_person_mask(people, src_w, src_h, kf_w, kf_h, dilate_px=dilate_px)
        ratio = float((mask > 0).sum()) / float(kf_w * kf_h)
        if people:
            n_with_person += 1
        total_ratio += ratio
        npy_path = out / f"mask_kf_{int(kf['kf_index']):04d}.npy"
        np.save(str(npy_path), (mask > 0).astype(np.uint8))
        if save_debug and (kf["kf_index"] % 20 == 0):
            cv2.imwrite(str(out / "debug" / f"mask_kf_{int(kf['kf_index']):04d}.png"), mask)
        records.append({
            "kf_index": int(kf["kf_index"]),
            "src_frame_index": fi,
            "n_person": len(people),
            "invalid_ratio": ratio,
            "mask_npy": str(npy_path),
        })

    n = max(1, len(keyframes))
    return {
        "n_keyframes": len(keyframes),
        "n_keyframes_with_person": n_with_person,
        "mean_invalid_ratio": total_ratio / n,
        "dilate_px": dilate_px,
        "records": records,
    }
