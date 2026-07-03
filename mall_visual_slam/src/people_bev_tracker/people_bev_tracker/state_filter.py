"""按 track_id 平滑 BEV 位置 (EMA)，并维护历史轨迹。"""

from __future__ import annotations

from collections import deque
from typing import Deque, Dict, Optional, Tuple

import numpy as np


class PeopleStateFilter:
    def __init__(
        self,
        ema_alpha: float = 0.35,
        max_human_speed_mps: float = 3.0,
        reject_large_jumps: bool = True,
        max_lost_frames: int = 15,
        history_length: int = 80,
    ):
        self.alpha = float(ema_alpha)
        self.max_speed = float(max_human_speed_mps)
        self.reject_jumps = bool(reject_large_jumps)
        self.max_lost_frames = int(max_lost_frames)
        self.history_length = int(history_length)

        # track_id -> dict(filtered_bev, last_ts, last_frame, history, lost_count)
        self._tracks: Dict[int, dict] = {}

    def update(
        self,
        track_id: int,
        timestamp: float,
        frame_index: int,
        bev_xy: np.ndarray,
        score: float,
    ) -> Tuple[np.ndarray, bool]:
        """更新一个 track。返回 (filtered_bev_xy, accepted)。

        accepted=False 表示此次 BEV 跳变过大，被丢弃。
        """
        bev_xy = np.asarray(bev_xy, dtype=np.float64).reshape(2)
        tr = self._tracks.get(track_id)
        if tr is None:
            filtered = bev_xy.copy()
            hist: Deque[Tuple[int, np.ndarray]] = deque(maxlen=self.history_length)
            hist.append((frame_index, filtered.copy()))
            self._tracks[track_id] = {
                "filtered": filtered,
                "last_ts": timestamp,
                "last_frame": frame_index,
                "history": hist,
                "lost_count": 0,
                "score": float(score),
            }
            return filtered, True

        dt = max(timestamp - tr["last_ts"], 1e-3)
        prev = tr["filtered"]
        delta = bev_xy - prev
        speed = float(np.linalg.norm(delta)) / dt
        if self.reject_jumps and speed > self.max_speed and tr["lost_count"] == 0:
            tr["lost_count"] += 1
            return prev.copy(), False

        filtered = self.alpha * bev_xy + (1.0 - self.alpha) * prev
        tr["filtered"] = filtered
        tr["last_ts"] = timestamp
        tr["last_frame"] = frame_index
        tr["lost_count"] = 0
        tr["score"] = float(score)
        tr["history"].append((frame_index, filtered.copy()))
        return filtered, True

    def mark_unseen(self, current_frame: int) -> None:
        """对本帧未出现的 track 增加 lost 计数。"""
        for tr in self._tracks.values():
            if tr["last_frame"] != current_frame:
                tr["lost_count"] += 1

    def active_tracks(self) -> Dict[int, dict]:
        return {
            tid: tr
            for tid, tr in self._tracks.items()
            if tr["lost_count"] <= self.max_lost_frames
        }

    def all_tracks(self) -> Dict[int, dict]:
        return self._tracks

    def latest_position(self, track_id: int) -> Optional[np.ndarray]:
        tr = self._tracks.get(track_id)
        if tr is None:
            return None
        return tr["filtered"].copy()
