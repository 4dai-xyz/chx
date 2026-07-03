"""脚底像素点提取。

优先使用分割 mask 求底部像素中位 x；mask 不可用时退化为 bbox 底中点。
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np


def compute_footpoint(
    mask: Optional[np.ndarray],
    bbox_xyxy: np.ndarray,
    image_shape: Tuple[int, int],
    bottom_percent: float = 0.05,
    min_area: int = 80,
) -> np.ndarray:
    """返回 (u, v) 像素脚底点 (float)."""
    h, w = int(image_shape[0]), int(image_shape[1])
    x1, y1, x2, y2 = [float(v) for v in bbox_xyxy.reshape(4)]

    use_mask = False
    if mask is not None:
        m = mask
        if m.dtype != bool:
            m = m > 0
        if m.shape[0] != h or m.shape[1] != w:
            m = None
        elif int(m.sum()) >= min_area:
            use_mask = True

    if use_mask:
        ys, xs = np.where(m)
        if ys.size > 0:
            v_max = int(ys.max())
            v_min_band = int(v_max - max(1, bottom_percent * (v_max - ys.min() + 1)))
            band = ys >= v_min_band
            x_band = xs[band]
            if x_band.size > 0:
                u = float(np.median(x_band))
                v = float(v_max)
                u = max(0.0, min(u, w - 1.0))
                v = max(0.0, min(v, h - 1.0))
                return np.array([u, v], dtype=np.float64)

    # bbox fallback
    u = 0.5 * (x1 + x2)
    v = y2
    u = max(0.0, min(u, w - 1.0))
    v = max(0.0, min(v, h - 1.0))
    return np.array([u, v], dtype=np.float64)
