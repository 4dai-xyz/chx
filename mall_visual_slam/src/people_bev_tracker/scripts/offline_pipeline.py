#!/usr/bin/env python3
"""离线行人 BEV 跟踪流水线。

将原始视频、DPVO 轨迹、YOLO-seg + BoT-SORT 跟踪串起来：
    原始视频帧 -> YOLO-seg / BoT-SORT -> 行人 mask + bbox + track_id
        -> 脚底像素 -> 像素射线
        -> 与世界地面 y=0 相交 -> 行人世界坐标 -> BEV 坐标 -> EMA 平滑
        -> 在 BEV 画布上叠加相机轨迹与行人位置

输出:
    output/people_bev/bev_tracking.mp4
    output/people_bev/debug_overlay.mp4
    output/people_bev/people_tracks.json
    output/people_bev/camera_trajectory.json
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

import cv2
import numpy as np

# Allow running this script without installing the package.
PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from people_bev_tracker.bev_canvas import BEVCanvas, _color_for_id
from people_bev_tracker.camera_model import load_intrinsics
from people_bev_tracker.footpoint import compute_footpoint
from people_bev_tracker.ground_projection import (
    intersect_footpoint_with_camera_ground,
    intersect_footpoint_with_ground,
    select_bev_axes,
)
from people_bev_tracker.io_utils import (
    iterate_frames,
    load_config,
    open_video_reader,
    open_video_writer,
    to_jsonable,
    write_json,
)
from people_bev_tracker.person_yolo_tracker import YoloPersonTracker
from people_bev_tracker.pose_io import (
    load_tum_trajectory,
    make_mock_trajectory,
    nearest_pose,
)
from people_bev_tracker.state_filter import PeopleStateFilter


def _resolve_path(p: str, repo_root: Path) -> str:
    pp = Path(p)
    if pp.is_absolute():
        return str(pp)
    return str((repo_root / p).resolve())


def _compose_side_by_side(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lh, lw = left.shape[:2]
    rh, rw = right.shape[:2]
    target_h = max(lh, rh)
    scale_l = target_h / lh
    scale_r = target_h / rh
    new_lw = max(1, int(round(lw * scale_l)))
    new_rw = max(1, int(round(rw * scale_r)))
    left_r = cv2.resize(left, (new_lw, target_h))
    right_r = cv2.resize(right, (new_rw, target_h))
    return np.hstack([left_r, right_r])


def _camera_heading_bev(T_wc: np.ndarray, bev_axes: List[str]) -> np.ndarray:
    """相机 +z 方向投到 BEV 平面，返回 2D 单位向量 (不归一化时为零向量)。"""
    R_wc = T_wc[:3, :3]
    forward_w = R_wc @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
    return select_bev_axes(forward_w, bev_axes)


def run(args: argparse.Namespace) -> int:
    repo_root = Path(__file__).resolve().parents[3]
    cfg = load_config(args.config)

    video_cfg = cfg.get("video", {})
    cam_cfg = cfg.get("camera", {})
    pose_cfg = cfg.get("pose", {})
    det_cfg = cfg.get("detector", {})
    trk_cfg = cfg.get("tracker", {})
    fp_cfg = cfg.get("footpoint", {})
    ground_cfg = cfg.get("ground", {})
    world_cfg = cfg.get("world", {})
    filt_cfg = cfg.get("filter", {})
    bev_cfg = cfg.get("bev", {})
    out_cfg = cfg.get("output", {})

    # CLI overrides
    video_path = args.video or video_cfg.get("path", "resources/input_video.mp4")
    calib_path = args.calib or cam_cfg.get(
        "calib_path", "config/KannalaBrandt8_1280x720.yaml"
    )
    pose_path = args.pose or pose_cfg.get(
        "tum_path", "output/dpvo/trajectory_tum.txt"
    )
    output_dir = args.output_dir or out_cfg.get("dir", "output/people_bev")
    tracker_name = args.tracker or trk_cfg.get("name", "botsort.yaml")
    model_name = args.model or det_cfg.get("model", "yolo11n-seg.pt")
    max_frames = args.max_frames if args.max_frames is not None else int(
        video_cfg.get("max_frames", 0) or 0
    )

    video_path = _resolve_path(video_path, repo_root)
    calib_path = _resolve_path(calib_path, repo_root)
    pose_path = _resolve_path(pose_path, repo_root)
    output_dir = _resolve_path(output_dir, repo_root)
    Path(output_dir).mkdir(parents=True, exist_ok=True)

    if not Path(video_path).exists():
        raise SystemExit(f"input video not found: {video_path}")
    if not Path(calib_path).exists():
        raise SystemExit(f"calibration not found: {calib_path}")

    # video meta
    cap = open_video_reader(video_path)
    src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    src_fps = float(cap.get(cv2.CAP_PROP_FPS)) or 30.0
    src_n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    out_fps = float(video_cfg.get("output_fps", src_fps) or src_fps)
    frame_stride = int(video_cfg.get("frame_stride", 1) or 1)

    print(f"[pipeline] video={video_path}  {src_w}x{src_h}@{src_fps:.2f}fps  n={src_n}")
    print(f"[pipeline] calib={calib_path}")
    print(f"[pipeline] pose ={pose_path}")
    print(f"[pipeline] out  ={output_dir}")
    print(f"[pipeline] tracker={tracker_name}  model={model_name}  max_frames={max_frames}")

    K, calib_size = load_intrinsics(calib_path, video_size=(src_w, src_h))
    print(f"[pipeline] K =\n{K}\n  calib_size_used={calib_size}")

    # poses
    if not Path(pose_path).exists():
        if args.allow_mock_pose:
            print(f"[pipeline] WARNING using mock identity trajectory")
            poses = make_mock_trajectory(src_n, src_fps)
        else:
            raise SystemExit(
                "找不到 DPVO 轨迹文件。请先运行 DPVO 并导出 TUM 格式轨迹，"
                "或用 --pose 指定轨迹路径，或加 --allow-mock-pose 仅做可视化测试。\n"
                f"missing: {pose_path}"
            )
    else:
        poses = load_tum_trajectory(
            pose_path,
            pose_is_twc=bool(pose_cfg.get("pose_is_twc", True)),
            scale=float(pose_cfg.get("scale", 1.0)),
            timestamp_unit=str(pose_cfg.get("timestamp_unit", "dpvo_tick")),
            dpvo_stride=int(pose_cfg.get("dpvo_stride", 2)),
            video_fps=src_fps,
        )
    pose_tol = float(pose_cfg.get("timestamp_tolerance_sec", 0.2))
    print(f"[pipeline] loaded {len(poses)} poses (tolerance={pose_tol:.3f}s)")

    # detector + tracker
    yolo = YoloPersonTracker(
        model_name=model_name,
        fallback_model=det_cfg.get("fallback_model", "yolov8n-seg.pt"),
        tracker_name=tracker_name,
        fallback_tracker=trk_cfg.get("fallback_name", "bytetrack.yaml"),
        person_class_id=int(det_cfg.get("person_class_id", 0)),
        conf=float(det_cfg.get("conf_thres", 0.35)),
        iou=float(det_cfg.get("iou_thres", 0.5)),
        imgsz=int(det_cfg.get("imgsz", 960)),
        device=det_cfg.get("device", "auto") or "auto",
    )

    # filter
    pf = PeopleStateFilter(
        ema_alpha=float(filt_cfg.get("ema_alpha", 0.35)),
        max_human_speed_mps=float(filt_cfg.get("max_human_speed_mps", 3.0)),
        reject_large_jumps=bool(filt_cfg.get("reject_large_jumps", True)),
        max_lost_frames=int(trk_cfg.get("max_lost_frames", 15)),
        history_length=int(bev_cfg.get("trail_length", 80)),
    )

    # BEV canvas
    bev_axes_cfg = world_cfg.get("bev_axes", ["x", "z"])
    bev = BEVCanvas(
        width_px=int(bev_cfg.get("width_px", 1200)),
        height_px=int(bev_cfg.get("height_px", 1200)),
        resolution_m_per_px=float(bev_cfg.get("resolution_m_per_px", 0.05)),
        origin_world=tuple(bev_cfg.get("origin_world", [0.0, 0.0])),
        grid_step_m=float(bev_cfg.get("grid_step_m", 1.0)),
        trail_length=int(bev_cfg.get("trail_length", 80)),
    )

    ground_normal = np.array(ground_cfg.get("normal", [0.0, 1.0, 0.0]), dtype=np.float64)
    ground_d = float(ground_cfg.get("d", 0.0))
    ground_mode = str(ground_cfg.get("mode", "world")).lower()
    camera_height = float(ground_cfg.get("camera_height", 0.1))
    camera_pitch_deg = float(ground_cfg.get("camera_pitch_deg", 0.0))

    # output writers
    bev_path = str(Path(output_dir) / "bev_tracking.mp4")
    dbg_path = str(Path(output_dir) / "debug_overlay.mp4")
    people_json_path = str(Path(output_dir) / "people_tracks.json")
    camera_json_path = str(Path(output_dir) / "camera_trajectory.json")

    save_bev = bool(out_cfg.get("save_bev_video", True))
    save_dbg = bool(out_cfg.get("save_debug_overlay", True))
    save_json = bool(out_cfg.get("save_json", True))

    bev_writer = (
        open_video_writer(bev_path, out_fps, (bev.W, bev.H))
        if save_bev
        else None
    )
    dbg_writer = (
        open_video_writer(dbg_path, out_fps, (src_w, src_h)) if save_dbg else None
    )

    # accumulators
    people_frames: List[dict] = []
    camera_poses_record: List[dict] = []
    camera_trail_bev: List[np.ndarray] = []
    pose_miss = 0
    pose_hit = 0
    proj_ok = 0
    proj_fail = 0

    t0 = time.time()
    for src_idx, frame_bgr in iterate_frames(cap, max_frames=max_frames, frame_stride=frame_stride):
        timestamp = src_idx / src_fps if src_fps > 0 else float(src_idx)

        pose = nearest_pose(poses, timestamp, pose_tol)
        if pose is None:
            pose_miss += 1
        else:
            pose_hit += 1

        tracked = yolo.step(frame_bgr)

        debug_frame = frame_bgr.copy()
        frame_people: List[dict] = []

        for person in tracked:
            foot = compute_footpoint(
                person.mask,
                person.bbox_xyxy,
                image_shape=(src_h, src_w),
                bottom_percent=float(fp_cfg.get("mask_bottom_percent", 0.05)),
                min_area=int(fp_cfg.get("min_mask_area_px", 80)),
            )
            person.foot_pixel = foot
            color = _color_for_id(person.track_id)

            x1, y1, x2, y2 = [int(round(v)) for v in person.bbox_xyxy]
            cv2.rectangle(debug_frame, (x1, y1), (x2, y2), color, 2)
            label = f"ID {person.track_id}  {person.score:.2f}"
            cv2.putText(
                debug_frame,
                label,
                (x1, max(0, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA,
            )
            if person.mask is not None:
                overlay = debug_frame.copy()
                overlay[person.mask] = (
                    np.array(color, dtype=np.uint8) * 0.5
                    + overlay[person.mask] * 0.5
                ).astype(np.uint8)
                debug_frame = overlay
            cv2.circle(
                debug_frame,
                (int(round(foot[0])), int(round(foot[1]))),
                4,
                color,
                -1,
                cv2.LINE_AA,
            )

            entry = {
                "track_id": int(person.track_id),
                "bbox_xyxy": person.bbox_xyxy.tolist(),
                "score": float(person.score),
                "foot_pixel": foot.tolist(),
                "world_xyz": None,
                "bev_xy": None,
                "filtered_bev_xy": None,
                "projected": False,
            }

            if pose is not None:
                if ground_mode == "camera":
                    Xw = intersect_footpoint_with_camera_ground(
                        foot,
                        K,
                        pose.T_wc,
                        camera_height=camera_height,
                        camera_pitch_deg=camera_pitch_deg,
                    )
                else:
                    Xw = intersect_footpoint_with_ground(
                        foot,
                        K,
                        pose.T_wc,
                        ground_normal,
                        ground_d,
                    )
                if Xw is not None:
                    bev_xy = select_bev_axes(Xw, bev_axes_cfg)
                    filtered, accepted = pf.update(
                        person.track_id,
                        timestamp,
                        src_idx,
                        bev_xy,
                        person.score,
                    )
                    entry["world_xyz"] = Xw.tolist()
                    entry["bev_xy"] = bev_xy.tolist()
                    entry["filtered_bev_xy"] = filtered.tolist()
                    entry["projected"] = True
                    entry["accepted"] = bool(accepted)
                    proj_ok += 1
                    cv2.putText(
                        debug_frame,
                        f"({Xw[0]:.2f},{Xw[2]:.2f})m",
                        (x1, min(src_h - 4, y2 + 16)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.42,
                        color,
                        1,
                        cv2.LINE_AA,
                    )
                else:
                    proj_fail += 1
                    cv2.putText(
                        debug_frame,
                        "no-ground",
                        (x1, min(src_h - 4, y2 + 16)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.42,
                        (0, 0, 255),
                        1,
                        cv2.LINE_AA,
                    )
            else:
                cv2.putText(
                    debug_frame,
                    "no-pose",
                    (x1, min(src_h - 4, y2 + 16)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.42,
                    (0, 0, 255),
                    1,
                    cv2.LINE_AA,
                )

            frame_people.append(entry)

        # mark unseen tracks
        pf.mark_unseen(src_idx)

        # 相机轨迹
        if pose is not None:
            cam_xyz = pose.T_wc[:3, 3]
            cam_bev = select_bev_axes(cam_xyz, bev_axes_cfg)
            camera_trail_bev.append(cam_bev)
            heading = _camera_heading_bev(pose.T_wc, bev_axes_cfg)
            camera_poses_record.append(
                {
                    "frame_index": int(src_idx),
                    "timestamp": float(timestamp),
                    "T_wc": pose.T_wc.tolist(),
                    "bev_xy": cam_bev.tolist(),
                }
            )
        else:
            cam_bev = camera_trail_bev[-1] if camera_trail_bev else None
            heading = None

        # 收集 active 行人历史
        people_history: Dict[int, List[Tuple[int, np.ndarray]]] = {}
        for tid, tr in pf.all_tracks().items():
            people_history[tid] = list(tr["history"])
        active_ids = [tid for tid in pf.active_tracks().keys()]

        bev_frame = bev.draw(
            camera_trail_world=camera_trail_bev,
            current_camera_world=cam_bev,
            camera_heading_world=heading,
            people_history=people_history,
            people_active_ids=active_ids,
            frame_index=src_idx,
            timestamp=timestamp,
            draw_camera_trail=bool(bev_cfg.get("draw_camera_trail", True)),
            draw_people_trails=bool(bev_cfg.get("draw_people_trails", True)),
        )

        if bev_writer is not None:
            bev_writer.write(bev_frame)
        if dbg_writer is not None:
            dbg_writer.write(debug_frame)

        people_frames.append(
            {
                "frame_index": int(src_idx),
                "timestamp": float(timestamp),
                "pose_available": pose is not None,
                "people": frame_people,
            }
        )

        if (src_idx + 1) % 50 == 0:
            elapsed = time.time() - t0
            rate = (src_idx + 1) / elapsed if elapsed > 0 else 0.0
            print(
                f"[pipeline] frame {src_idx+1}  poseHit={pose_hit} miss={pose_miss}"
                f"  proj_ok={proj_ok} fail={proj_fail}  {rate:.1f} fps"
            )

    cap.release()
    if bev_writer is not None:
        bev_writer.release()
    if dbg_writer is not None:
        dbg_writer.release()

    # write json
    if save_json:
        write_json(
            people_json_path,
            {
                "video": video_path,
                "calib": calib_path,
                "pose": pose_path,
                "video_size": [src_w, src_h],
                "video_fps": src_fps,
                "bev_axes": bev_axes_cfg,
                "ground": {"normal": ground_normal.tolist(), "d": ground_d},
                "pose_hit": int(pose_hit),
                "pose_miss": int(pose_miss),
                "projection_ok": int(proj_ok),
                "projection_fail": int(proj_fail),
                "frames": people_frames,
            },
        )
        write_json(
            camera_json_path,
            {
                "video": video_path,
                "pose": pose_path,
                "poses": camera_poses_record,
            },
        )

    elapsed = time.time() - t0
    print("=" * 70)
    print(f"[pipeline] done in {elapsed:.1f}s")
    print(
        f"[pipeline] pose hit/miss = {pose_hit}/{pose_miss},"
        f"  projection ok/fail = {proj_ok}/{proj_fail}"
    )
    if save_bev:
        print(f"[pipeline] bev_tracking.mp4  -> {bev_path}")
    if save_dbg:
        print(f"[pipeline] debug_overlay.mp4 -> {dbg_path}")
    if save_json:
        print(f"[pipeline] people_tracks.json -> {people_json_path}")
        print(f"[pipeline] camera_trajectory.json -> {camera_json_path}")
    return 0


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--video", default=None)
    p.add_argument("--calib", default=None)
    p.add_argument("--pose", default=None)
    p.add_argument("--output-dir", default=None)
    p.add_argument("--tracker", default=None)
    p.add_argument("--model", default=None)
    p.add_argument("--max-frames", type=int, default=None)
    p.add_argument(
        "--config",
        default=str(Path(__file__).resolve().parents[1] / "config" / "people_bev_tracker.yaml"),
    )
    p.add_argument("--allow-mock-pose", action="store_true")
    args = p.parse_args()
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
