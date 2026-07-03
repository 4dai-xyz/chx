"""二维 BEV 平面图绘制。"""

from __future__ import annotations

from typing import Dict, Iterable, List, Optional, Tuple

import cv2
import numpy as np


def _color_for_id(track_id: int) -> Tuple[int, int, int]:
    rng = np.random.default_rng(int(track_id) * 9973 + 17)
    h = float(rng.uniform(0, 179))
    hsv = np.array([[[h, 200, 255]]], dtype=np.uint8)
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]
    return (int(bgr[0]), int(bgr[1]), int(bgr[2]))


class BEVCanvas:
    """以世界 BEV 坐标为基础的二维栅格画布。"""

    def __init__(
        self,
        width_px: int = 1200,
        height_px: int = 1200,
        resolution_m_per_px: float = 0.05,
        origin_world: Tuple[float, float] = (0.0, 0.0),
        grid_step_m: float = 1.0,
        trail_length: int = 80,
        background_bgr: Tuple[int, int, int] = (32, 32, 32),
        grid_bgr: Tuple[int, int, int] = (64, 64, 64),
        axis_bgr: Tuple[int, int, int] = (96, 96, 160),
        camera_bgr: Tuple[int, int, int] = (255, 255, 255),
        camera_trail_bgr: Tuple[int, int, int] = (180, 180, 180),
        static_layer: Optional[np.ndarray] = None,
    ):
        self.W = int(width_px)
        self.H = int(height_px)
        self.res = float(resolution_m_per_px)
        self.origin = np.asarray(origin_world, dtype=np.float64).reshape(2)
        self.grid_step = float(grid_step_m)
        self.trail_length = int(trail_length)
        self.bg = background_bgr
        self.grid_color = grid_bgr
        self.axis_color = axis_bgr
        self.cam_color = camera_bgr
        self.cam_trail_color = camera_trail_bgr
        self.static_layer = static_layer

        self._base = self._make_base()

    def _make_base(self) -> np.ndarray:
        # 优先用 static_layer 当底图 (占据栅格); 否则纯色。
        if self.static_layer is not None:
            sl = self.static_layer
            if sl.ndim == 2:
                img = cv2.cvtColor(sl.astype(np.uint8), cv2.COLOR_GRAY2BGR)
            else:
                img = sl.astype(np.uint8).copy()
            if img.shape[0] != self.H or img.shape[1] != self.W:
                img = cv2.resize(img, (self.W, self.H), interpolation=cv2.INTER_NEAREST)
        else:
            img = np.full((self.H, self.W, 3), self.bg, dtype=np.uint8)
        cx = self.W // 2
        cy = self.H // 2
        # 米制网格
        step_px = int(round(self.grid_step / self.res))
        if step_px > 4:
            for x in range(cx % step_px, self.W, step_px):
                cv2.line(img, (x, 0), (x, self.H - 1), self.grid_color, 1)
            for y in range(cy % step_px, self.H, step_px):
                cv2.line(img, (0, y), (self.W - 1, y), self.grid_color, 1)
        # 主轴
        cv2.line(img, (cx, 0), (cx, self.H - 1), self.axis_color, 1)
        cv2.line(img, (0, cy), (self.W - 1, cy), self.axis_color, 1)
        # 米尺标注
        for i in range(-int(self.W / step_px / 2), int(self.W / step_px / 2) + 1, 2):
            if i == 0:
                continue
            px = cx + i * step_px
            cv2.putText(
                img,
                f"{int(i * self.grid_step)}m",
                (px + 2, cy - 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                self.axis_color,
                1,
                cv2.LINE_AA,
            )
        return img

    def world_to_canvas(self, bev_xy: np.ndarray) -> Tuple[int, int]:
        x = float(bev_xy[0]) - self.origin[0]
        y = float(bev_xy[1]) - self.origin[1]
        px = int(round(self.W / 2 + x / self.res))
        py = int(round(self.H / 2 - y / self.res))
        return px, py

    def draw(
        self,
        camera_trail_world: List[np.ndarray],
        current_camera_world: Optional[np.ndarray],
        camera_heading_world: Optional[np.ndarray],
        people_history: Dict[int, List[Tuple[int, np.ndarray]]],
        people_active_ids: Iterable[int],
        frame_index: int,
        timestamp: float,
        draw_camera_trail: bool = True,
        draw_people_trails: bool = True,
        draw_inactive_people: bool = True,
    ) -> np.ndarray:
        img = self._base.copy()

        # 相机历史轨迹
        if draw_camera_trail and len(camera_trail_world) >= 2:
            pts = np.array(
                [self.world_to_canvas(p) for p in camera_trail_world], dtype=np.int32
            )
            cv2.polylines(img, [pts], False, self.cam_trail_color, 2, cv2.LINE_AA)

        # 当前相机位置 + 朝向
        if current_camera_world is not None:
            cu, cv_ = self.world_to_canvas(current_camera_world)
            cv2.circle(img, (cu, cv_), 6, self.cam_color, -1, cv2.LINE_AA)
            if camera_heading_world is not None:
                # 朝向是 BEV 平面上的单位向量
                dx = float(camera_heading_world[0])
                dy = float(camera_heading_world[1])
                n = np.hypot(dx, dy)
                if n > 1e-9:
                    dx /= n
                    dy /= n
                    arrow_len_px = 24
                    eu = cu + int(round(dx * arrow_len_px / 1.0))
                    ev = cv_ - int(round(dy * arrow_len_px / 1.0))
                    cv2.arrowedLine(
                        img,
                        (cu, cv_),
                        (eu, ev),
                        self.cam_color,
                        2,
                        cv2.LINE_AA,
                        tipLength=0.35,
                    )
            cv2.putText(
                img,
                "CAM",
                (cu + 8, cv_ - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                self.cam_color,
                1,
                cv2.LINE_AA,
            )

        active_ids = set(int(t) for t in people_active_ids)

        # 每个 track 的历史 + 当前位置
        for tid, hist in people_history.items():
            color = _color_for_id(tid)
            is_active = tid in active_ids
            if draw_people_trails and len(hist) >= 2:
                pts = np.array(
                    [self.world_to_canvas(p) for _, p in hist[-self.trail_length :]],
                    dtype=np.int32,
                )
                cv2.polylines(img, [pts], False, color, 2, cv2.LINE_AA)
            if is_active and len(hist) > 0:
                last_pt = hist[-1][1]
                pu, pv = self.world_to_canvas(last_pt)
                cv2.circle(img, (pu, pv), 7, color, -1, cv2.LINE_AA)
                cv2.circle(img, (pu, pv), 8, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.putText(
                    img,
                    f"ID {tid}",
                    (pu + 9, pv - 9),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    color,
                    1,
                    cv2.LINE_AA,
                )

        # 顶部 HUD
        cv2.rectangle(img, (0, 0), (self.W, 28), (16, 16, 16), -1)
        cv2.putText(
            img,
            f"frame {frame_index}  t={timestamp:.2f}s  active={len(active_ids)}  total_ids={len(people_history)}",
            (8, 19),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (240, 240, 240),
            1,
            cv2.LINE_AA,
        )
        return img
