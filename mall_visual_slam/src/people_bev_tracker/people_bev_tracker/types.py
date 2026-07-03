"""数据结构定义。"""

from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass
class CameraPose:
    timestamp: float
    frame_index: int
    T_wc: np.ndarray  # (4, 4), camera-to-world


@dataclass
class TrackedPerson:
    track_id: int
    bbox_xyxy: np.ndarray  # [x1, y1, x2, y2]
    score: float
    mask: Optional[np.ndarray] = None
    foot_pixel: Optional[np.ndarray] = None


@dataclass
class PersonWorldState:
    track_id: int
    timestamp: float
    frame_index: int
    world_xyz: np.ndarray
    bev_xy: np.ndarray
    filtered_bev_xy: np.ndarray
    score: float


@dataclass
class FrameRecord:
    frame_index: int
    timestamp: float
    camera_pose: Optional[CameraPose]
    people: list = field(default_factory=list)
