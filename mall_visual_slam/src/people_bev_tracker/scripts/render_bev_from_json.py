#!/usr/bin/env python3
"""仅基于 people_tracks.json + camera_trajectory.json 重新渲染 BEV 视频。

不重跑 YOLO / 不依赖原视频。适合：
* 想换 BEV 配色 / 分辨率 / 范围
* 只想看「相机轨迹 + 行人当前定位点」(关闭 people 轨迹线)

用法:
    python src/people_bev_tracker/scripts/render_bev_from_json.py \
      --people-json output/people_bev/people_tracks.json \
      --camera-json output/people_bev/camera_trajectory.json \
      --output       output/people_bev/bev_tracking_clean.mp4 \
      --no-people-trails
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict, deque
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from people_bev_tracker.bev_canvas import BEVCanvas
from people_bev_tracker.io_utils import open_video_writer


def _camera_heading_bev(T_wc: np.ndarray, bev_axes: List[str]) -> np.ndarray:
    R = np.asarray(T_wc, dtype=np.float64)[:3, :3]
    fwd_w = R @ np.array([0.0, 0.0, 1.0])
    idx = {"x": 0, "y": 1, "z": 2}
    return np.array(
        [fwd_w[idx[bev_axes[0].lower()]], fwd_w[idx[bev_axes[1].lower()]]],
        dtype=np.float64,
    )


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--people-json", required=True)
    p.add_argument("--camera-json", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--fps", type=float, default=30.0)
    p.add_argument("--width", type=int, default=1200)
    p.add_argument("--height", type=int, default=1200)
    p.add_argument("--resolution", type=float, default=0.004,
                   help="BEV 像素分辨率 (每像素多少 DPVO 单位)")
    p.add_argument("--origin", type=float, nargs=2, default=(0.0, 1.5),
                   help="BEV 画布中心对应的世界 (bev_x, bev_y)")
    p.add_argument("--grid-step", type=float, default=0.5)
    p.add_argument("--trail-length", type=int, default=80,
                   help="行人轨迹保留的最大点数 (即使关闭显示也用于平滑)")
    p.add_argument("--camera-trail-length", type=int, default=0,
                   help="0 表示绘制全部相机轨迹；>0 表示只画最近 N 个相机位置")
    p.add_argument("--no-people-trails", action="store_true",
                   help="只画行人当前定位点，不画轨迹线")
    p.add_argument("--no-camera-trail", action="store_true",
                   help="不画相机历史轨迹")
    p.add_argument("--use-filtered", action="store_true",
                   help="行人定位点使用 filtered_bev_xy (默认用 bev_xy)")
    p.add_argument("--max-lost-frames", type=int, default=15,
                   help="超过这么多帧没出现的 track_id 视为非 active")
    args = p.parse_args()

    people_data = json.load(open(args.people_json, "r", encoding="utf-8"))
    cam_data = json.load(open(args.camera_json, "r", encoding="utf-8"))

    frames = people_data["frames"]
    bev_axes = people_data.get("bev_axes", ["x", "z"])

    # frame_index -> camera bev_xy + T_wc
    cam_lookup: Dict[int, dict] = {}
    for c in cam_data["poses"]:
        cam_lookup[int(c["frame_index"])] = c

    bev = BEVCanvas(
        width_px=args.width,
        height_px=args.height,
        resolution_m_per_px=args.resolution,
        origin_world=tuple(args.origin),
        grid_step_m=args.grid_step,
        trail_length=args.trail_length,
    )

    writer = open_video_writer(args.output, args.fps, (bev.W, bev.H))

    # 累积：相机轨迹 + 行人最新位置 (历史用于 trail 的可选绘制)
    cam_trail: List[np.ndarray] = []
    people_history: Dict[int, deque] = defaultdict(
        lambda: deque(maxlen=args.trail_length)
    )
    last_seen: Dict[int, int] = {}

    for frame in frames:
        fi = int(frame["frame_index"])
        ts = float(frame["timestamp"])

        # 相机
        cam_entry = cam_lookup.get(fi)
        cam_bev = None
        heading = None
        if cam_entry is not None:
            cam_bev = np.asarray(cam_entry["bev_xy"], dtype=np.float64)
            cam_trail.append(cam_bev)
            T_wc = np.asarray(cam_entry["T_wc"], dtype=np.float64)
            heading = _camera_heading_bev(T_wc, bev_axes)
        elif cam_trail:
            cam_bev = cam_trail[-1]

        # 行人
        for person in frame["people"]:
            if not person.get("projected"):
                continue
            key = "filtered_bev_xy" if args.use_filtered else "bev_xy"
            xy = person.get(key)
            if xy is None:
                continue
            tid = int(person["track_id"])
            people_history[tid].append((fi, np.asarray(xy, dtype=np.float64)))
            last_seen[tid] = fi

        active_ids = [
            tid for tid, lf in last_seen.items() if fi - lf <= args.max_lost_frames
        ]

        cam_trail_display = cam_trail
        if args.camera_trail_length > 0:
            cam_trail_display = cam_trail[-args.camera_trail_length :]

        # convert deque -> list of (idx, xy) for canvas
        history_for_canvas: Dict[int, List[Tuple[int, np.ndarray]]] = {
            tid: list(h) for tid, h in people_history.items()
        }

        img = bev.draw(
            camera_trail_world=cam_trail_display,
            current_camera_world=cam_bev,
            camera_heading_world=heading,
            people_history=history_for_canvas,
            people_active_ids=active_ids,
            frame_index=fi,
            timestamp=ts,
            draw_camera_trail=not args.no_camera_trail,
            draw_people_trails=not args.no_people_trails,
        )
        writer.write(img)

        if (fi + 1) % 300 == 0:
            print(
                f"[render] frame {fi+1}/{frames[-1]['frame_index']+1}"
                f"  active={len(active_ids)}  total_ids={len(last_seen)}"
            )

    writer.release()
    print(f"[render] done -> {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
