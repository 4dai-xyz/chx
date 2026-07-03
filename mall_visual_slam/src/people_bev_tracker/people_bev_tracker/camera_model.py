"""相机模型 — 读取标定文件，提供像素到相机射线的转换。

第一版只读取 Pinhole 部分 (fx, fy, cx, cy)。若标定分辨率与视频不一致，
会自动按比例缩放 K。
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional, Tuple

import numpy as np


def _read_yaml_text(path: str) -> str:
    txt = Path(path).read_text(encoding="utf-8", errors="ignore")
    # ORB-SLAM3 风格的 YAML 头部 "%YAML:1.0" 会让 PyYAML 解析失败，去掉它。
    lines = []
    for line in txt.splitlines():
        s = line.strip()
        if s.startswith("%YAML"):
            continue
        lines.append(line)
    return "\n".join(lines)


def _parse_calib_yaml(path: str) -> dict:
    text = _read_yaml_text(path)
    out: dict = {}
    for raw in text.splitlines():
        if not raw or raw.lstrip().startswith("#"):
            continue
        if ":" not in raw:
            continue
        key, _, value = raw.partition(":")
        key = key.strip()
        value = value.strip()
        if not value:
            continue
        if value.startswith('"') and value.endswith('"'):
            value = value[1:-1]
            out[key] = value
            continue
        try:
            if "." in value or "e" in value.lower():
                out[key] = float(value)
            else:
                out[key] = int(value)
        except ValueError:
            out[key] = value
    return out


def load_intrinsics(
    calib_path: str,
    video_size: Optional[Tuple[int, int]] = None,
) -> Tuple[np.ndarray, Tuple[int, int]]:
    """读取相机内参矩阵 K。

    支持的字段:
        Camera.fx / Camera.fy / Camera.cx / Camera.cy / Camera.width / Camera.height
        或裸字段 fx / fy / cx / cy / width / height

    若提供 ``video_size = (W, H)`` 且与标定分辨率不一致，会自动等比缩放 K。
    """
    cfg = _parse_calib_yaml(calib_path)

    def _get(keys, required=True):
        for k in keys:
            if k in cfg and cfg[k] != "":
                return float(cfg[k])
        if required:
            raise KeyError(f"calib file missing one of: {keys}")
        return None

    fx = _get(["Camera.fx", "fx"])
    fy = _get(["Camera.fy", "fy"])
    cx = _get(["Camera.cx", "cx"])
    cy = _get(["Camera.cy", "cy"])
    calib_w = _get(["Camera.width", "width"], required=False)
    calib_h = _get(["Camera.height", "height"], required=False)

    K = np.array(
        [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )

    if video_size is not None and calib_w is not None and calib_h is not None:
        vw, vh = video_size
        if abs(vw - calib_w) > 1 or abs(vh - calib_h) > 1:
            sx = vw / float(calib_w)
            sy = vh / float(calib_h)
            K[0, 0] *= sx
            K[1, 1] *= sy
            K[0, 2] *= sx
            K[1, 2] *= sy
            calib_w, calib_h = float(vw), float(vh)

    size = (
        int(calib_w) if calib_w else (video_size[0] if video_size else 0),
        int(calib_h) if calib_h else (video_size[1] if video_size else 0),
    )
    return K, size


def pixel_to_ray(pixel_uv: np.ndarray, K: np.ndarray) -> np.ndarray:
    """像素 (u, v) -> 相机系归一化射线 [x, y, z]."""
    uv = np.asarray(pixel_uv, dtype=np.float64).reshape(2)
    p = np.array([uv[0], uv[1], 1.0])
    ray = np.linalg.inv(K) @ p
    n = np.linalg.norm(ray)
    if n < 1e-12:
        return ray
    return ray / n
