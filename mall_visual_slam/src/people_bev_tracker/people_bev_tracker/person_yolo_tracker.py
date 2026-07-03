"""Ultralytics YOLO-seg + 内置 BoT-SORT/ByteTrack 行人跟踪。"""

from __future__ import annotations

from typing import List, Optional

import numpy as np

from .types import TrackedPerson


class YoloPersonTracker:
    def __init__(
        self,
        model_name: str = "yolo11n-seg.pt",
        fallback_model: str = "yolov8n-seg.pt",
        tracker_name: str = "botsort.yaml",
        fallback_tracker: str = "bytetrack.yaml",
        person_class_id: int = 0,
        conf: float = 0.35,
        iou: float = 0.50,
        imgsz: int = 960,
        device: str = "auto",
    ):
        from ultralytics import YOLO  # 延迟导入，避免模块加载即触发权重下载

        self.person_class_id = int(person_class_id)
        self.conf = float(conf)
        self.iou = float(iou)
        self.imgsz = int(imgsz)
        self.device = device if device != "auto" else None

        try:
            self.model = YOLO(model_name)
            self.model_name = model_name
        except Exception as exc:  # noqa: BLE001
            print(f"[YoloPersonTracker] 主模型 {model_name} 加载失败: {exc}")
            print(f"[YoloPersonTracker] fallback -> {fallback_model}")
            self.model = YOLO(fallback_model)
            self.model_name = fallback_model

        self.tracker_name = tracker_name
        self.fallback_tracker = fallback_tracker
        self._tracker_ok = True

    def step(self, frame_bgr: np.ndarray) -> List[TrackedPerson]:
        kwargs = dict(
            source=frame_bgr,
            persist=True,
            tracker=self.tracker_name,
            classes=[self.person_class_id],
            conf=self.conf,
            iou=self.iou,
            imgsz=self.imgsz,
            verbose=False,
        )
        if self.device:
            kwargs["device"] = self.device

        try:
            results = self.model.track(**kwargs)
        except Exception as exc:  # noqa: BLE001
            if self._tracker_ok and self.fallback_tracker:
                print(
                    f"[YoloPersonTracker] tracker {self.tracker_name} 失败: {exc}"
                    f" -> fallback {self.fallback_tracker}"
                )
                self.tracker_name = self.fallback_tracker
                self._tracker_ok = False
                kwargs["tracker"] = self.tracker_name
                results = self.model.track(**kwargs)
            else:
                raise

        if not results:
            return []
        res = results[0]
        if res.boxes is None or res.boxes.id is None:
            return []
        ids = res.boxes.id.detach().cpu().numpy().astype(int).reshape(-1)
        xyxy = res.boxes.xyxy.detach().cpu().numpy().astype(np.float64)
        conf = (
            res.boxes.conf.detach().cpu().numpy().astype(np.float64)
            if res.boxes.conf is not None
            else np.ones(len(ids), dtype=np.float64)
        )
        cls = (
            res.boxes.cls.detach().cpu().numpy().astype(int)
            if res.boxes.cls is not None
            else np.full(len(ids), self.person_class_id, dtype=int)
        )

        masks: Optional[np.ndarray] = None
        if getattr(res, "masks", None) is not None and res.masks.data is not None:
            md = res.masks.data.detach().cpu().numpy()  # (N, mh, mw) float
            # 把 mask 放大到原图大小，再二值化
            import cv2

            h, w = frame_bgr.shape[:2]
            stacked = np.zeros((md.shape[0], h, w), dtype=bool)
            for i in range(md.shape[0]):
                m_i = md[i]
                if m_i.shape[0] != h or m_i.shape[1] != w:
                    m_i = cv2.resize(
                        m_i, (w, h), interpolation=cv2.INTER_NEAREST
                    )
                stacked[i] = m_i > 0.5
            masks = stacked

        out: List[TrackedPerson] = []
        for i, tid in enumerate(ids):
            if cls[i] != self.person_class_id:
                continue
            mask_i = masks[i] if masks is not None else None
            out.append(
                TrackedPerson(
                    track_id=int(tid),
                    bbox_xyxy=xyxy[i].copy(),
                    score=float(conf[i]),
                    mask=mask_i,
                )
            )
        return out
