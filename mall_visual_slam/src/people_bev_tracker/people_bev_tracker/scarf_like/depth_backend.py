"""V3.1 阶段 4: 单目深度后端 (Depth Anything V2 Metric Indoor)。

统一接口:

    backend = DepthBackend(cfg)
    result = backend.infer(image_bgr)   # -> {"depth": HxW float32 (米), "backend": name}

优先 Depth Anything V2 Metric Indoor (输出米制深度), 通过 HuggingFace transformers
加载 (权重已缓存在 ~/.cache/huggingface)。不 clone 官方 GitHub, 不改官方源码。
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Dict, Optional

import numpy as np


class DepthBackend:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.device = cfg.get("device", "cuda")
        self.model_metric = cfg.get(
            "hf_model_metric",
            "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf")
        self.min_depth_m = float(cfg.get("min_depth_m", 0.3))
        self.max_depth_m = float(cfg.get("max_depth_m", 15.0))
        self.name = "depth_anything_v2_metric_indoor"

        # 离线加载 (权重已下载)
        os.environ.setdefault("HF_HUB_OFFLINE", "1")
        import torch
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation
        self._torch = torch
        self.use_cuda = bool(torch.cuda.is_available()) and self.device.startswith("cuda")
        self.processor = AutoImageProcessor.from_pretrained(self.model_metric)
        self.model = AutoModelForDepthEstimation.from_pretrained(self.model_metric)
        self.model = self.model.to("cuda" if self.use_cuda else "cpu").eval()

    def infer(self, image_bgr: np.ndarray) -> Dict:
        """输入 BGR (H, W, 3), 输出米制深度 (H, W) float32。"""
        import torch
        from PIL import Image
        rgb = image_bgr[:, :, ::-1]
        pil = Image.fromarray(np.ascontiguousarray(rgb))
        inp = self.processor(images=pil, return_tensors="pt")
        if self.use_cuda:
            inp = {k: v.to("cuda") for k, v in inp.items()}
        with torch.no_grad():
            out = self.model(**inp)
        post = self.processor.post_process_depth_estimation(
            out, target_sizes=[(pil.height, pil.width)])
        depth = post[0]["predicted_depth"].detach().cpu().numpy().astype(np.float32)
        # clamp
        depth = np.clip(depth, self.min_depth_m, self.max_depth_m)
        return {"depth": depth, "backend": self.name}


def depth_to_vis(depth: np.ndarray) -> np.ndarray:
    """米制深度 → 彩色可视化 (近红远蓝)。"""
    import cv2
    d = depth.copy()
    lo, hi = np.percentile(d, [2, 98])
    if hi <= lo:
        hi = lo + 1e-3
    dn = np.clip((d - lo) / (hi - lo), 0, 1)
    vis = (dn * 255).astype(np.uint8)
    return cv2.applyColorMap(255 - vis, cv2.COLORMAP_INFERNO)
