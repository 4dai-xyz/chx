#!/usr/bin/env python3
"""route_A 主流水线: 用平面化 DPVO 轨迹 + 静态地图 + YOLO 行人 → BEV MP4/JSON。

相对 ``offline_pipeline.py`` 的差异:
1. 从 static_map.npy + meta 加载 BEV 底图作 canvas.static_layer
2. --pose 用 route_A/trajectory_flat.txt
3. --ground-plane 用 route_A/ground_plane_final.json (作用于 people 投影)
4. 输出文件名带 _route_A 后缀, 落到 output/route_A/
5. --live 开 cv2.imshow 实时窗口
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple

import cv2
import numpy as np

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
    write_json,
)
from people_bev_tracker.person_yolo_tracker import YoloPersonTracker
from people_bev_tracker.pose_io import (
    load_tum_trajectory,
    make_mock_trajectory,
    nearest_pose,
)
from people_bev_tracker.state_filter import PeopleStateFilter
from people_bev_tracker.bev_alignment import (
    alignment_cfg_from_meta,
    apply_bev_alignment_xy,
    apply_bev_alignment_heading,
)


def _resolve_path(p: str, repo_root: Path) -> str:
    pp = Path(p)
    return str(pp if pp.is_absolute() else (repo_root / p).resolve())


def _camera_heading_bev(T_wc: np.ndarray, bev_axes: List[str]) -> np.ndarray:
    R_wc = T_wc[:3, :3]
    forward_w = R_wc @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
    return select_bev_axes(forward_w, bev_axes)


def run(args: argparse.Namespace) -> int:
    repo_root = Path(__file__).resolve().parents[3]
    cfg = load_config(args.config)

    # -------- resolve inputs --------
    video_path = _resolve_path(args.video or cfg["input"]["video"], repo_root)
    calib_path = _resolve_path(args.calib or cfg["input"]["calib"], repo_root)
    pose_path = _resolve_path(args.pose or "output/route_A/trajectory_flat.txt", repo_root)
    output_dir = Path(_resolve_path(args.output_dir or cfg["output"]["dir"], repo_root))
    output_dir.mkdir(parents=True, exist_ok=True)

    static_map_path = _resolve_path(
        args.static_map or str(output_dir / "static_map.npy"), repo_root)
    static_map_meta_path = _resolve_path(
        args.static_map_meta or str(output_dir / "static_map_meta.json"), repo_root)
    ground_plane_path = _resolve_path(
        args.ground_plane or str(output_dir / "ground_plane_final.json"), repo_root)

    for pth, label in [(video_path, "video"), (calib_path, "calib"),
                       (pose_path, "pose"), (static_map_path, "static_map"),
                       (static_map_meta_path, "static_map_meta"),
                       (ground_plane_path, "ground_plane")]:
        if not Path(pth).exists():
            raise SystemExit(f"missing {label}: {pth}")

    # -------- load video meta --------
    cap = open_video_reader(video_path)
    src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    src_fps = float(cap.get(cv2.CAP_PROP_FPS)) or 30.0
    src_n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    out_fps = float(cfg.get("output_fps", src_fps)) or src_fps
    max_frames = int(args.max_frames if args.max_frames is not None else 0)
    frame_stride = int(cfg.get("frame_stride", 1) or 1)

    print(f"[pipeline_A] video={video_path}  {src_w}x{src_h}@{src_fps:.2f}fps  n={src_n}")
    print(f"[pipeline_A] calib={calib_path}")
    print(f"[pipeline_A] pose ={pose_path}")
    print(f"[pipeline_A] static_map = {static_map_path}")
    print(f"[pipeline_A] ground_plane = {ground_plane_path}")
    print(f"[pipeline_A] output_dir = {output_dir}")

    # -------- calib K (auto-scale) --------
    K, _ = load_intrinsics(calib_path, video_size=(src_w, src_h))

    # -------- load poses --------
    pose_cfg = cfg.get("pose", {})
    poses = load_tum_trajectory(
        pose_path,
        pose_is_twc=bool(pose_cfg.get("pose_is_twc", True)),
        scale=float(pose_cfg.get("scale", 1.0)),
        timestamp_unit=str(pose_cfg.get("timestamp_unit", "dpvo_tick")),
        dpvo_stride=int(pose_cfg.get("dpvo_stride", 2)),
        video_fps=src_fps,
    )
    pose_tol = float(pose_cfg.get("timestamp_tolerance_sec", 0.2))
    print(f"[pipeline_A] loaded {len(poses)} poses (tol={pose_tol:.2f}s)")

    # -------- load static map + meta --------
    grid = np.load(static_map_path)
    with open(static_map_meta_path, "r", encoding="utf-8") as f:
        meta = json.load(f)

    # render static image to feed as bg (支持 V2 多模式)
    from people_bev_tracker.static_map import (
        render_static_map,
        render_nav_binary,
        render_tricolor,
        render_paper_style,
    )
    render_mode = str(getattr(args, "map_render_mode", "paper") or "paper").lower()
    # 从 v2 config 里读 render 颜色 (若 config 是 V1 的, 没这段就用默认)
    render_cfg_all = (cfg.get("static_map_v2", {}) or {}).get("render", {}) or {}
    if render_mode == "binary":
        static_bgr = render_nav_binary(grid, render_cfg_all.get("nav_binary"))
    elif render_mode == "tricolor":
        static_bgr = render_tricolor(grid, render_cfg_all.get("tricolor"))
    elif render_mode == "paper":
        static_bgr = render_paper_style(grid, colors=render_cfg_all.get("paper_style"))
    else:
        # 兼容 V1 的三色带 (unknown 深灰 / free 浅灰 / occupied 中灰)
        static_bgr = render_static_map(grid, meta)
    print(f"[pipeline_A] map render mode = {render_mode}")

    # R_align: static_map 是在 (R_align @ world) 坐标里画的。相机/行人
    # 在原始 world 坐标, 渲染时需要同样 rotate。
    R_align = np.asarray(meta.get("R_align", np.eye(3).tolist()), dtype=np.float64)

    # V3: bev_alignment. 如果 static_map 已经烘焙了 transform (mirror_y 等),
    # meta 里会有 bev_alignment 字段, pipeline 也要对相机/行人坐标做同一变换。
    align_cfg = alignment_cfg_from_meta(meta)
    if align_cfg.get("enabled") and align_cfg.get("transform", "identity") != "identity":
        print(f"[pipeline_A] bev_alignment enabled: transform = {align_cfg['transform']}")
    else:
        align_cfg = {"enabled": False, "transform": "identity"}

    # -------- load ground plane (for people projection) --------
    with open(ground_plane_path, "r", encoding="utf-8") as f:
        gp = json.load(f)
    best_gp = gp["best"] if "best" in gp else gp
    ground_normal = np.asarray(best_gp["normal"], dtype=np.float64)
    ground_d = float(best_gp["d"])
    print(f"[pipeline_A] ground plane: n={ground_normal.tolist()} d={ground_d:.4f}")

    # -------- BEV canvas: 底图 = static map --------
    bev_cfg = cfg.get("bev", {})
    bev = BEVCanvas(
        width_px=int(meta["width_px"]),
        height_px=int(meta["height_px"]),
        resolution_m_per_px=float(meta["resolution_unit_per_px"]),
        origin_world=tuple(meta["origin_world"]),
        grid_step_m=float(bev_cfg.get("grid_step_m", 0.5)),
        trail_length=int(bev_cfg.get("trail_length", 80)),
        static_layer=static_bgr,
    )
    bev_axes_cfg = list(meta.get("bev_axes", ["x", "z"]))

    # -------- YOLO + BoT-SORT --------
    p_cfg = cfg.get("people", {})
    yolo = YoloPersonTracker(
        model_name=p_cfg.get("model", "yolo11n-seg.pt"),
        fallback_model=p_cfg.get("fallback_model", "yolov8n-seg.pt"),
        tracker_name=p_cfg.get("tracker", "botsort.yaml"),
        fallback_tracker=p_cfg.get("fallback_tracker", "bytetrack.yaml"),
        person_class_id=int(p_cfg.get("person_class_id", 0)),
        conf=float(p_cfg.get("conf_thres", 0.35)),
        iou=float(p_cfg.get("iou_thres", 0.5)),
        imgsz=int(p_cfg.get("imgsz", 960)),
        device=p_cfg.get("device", "auto") or "auto",
    )

    # -------- state filter --------
    f_cfg = cfg.get("filter", {})
    pf = PeopleStateFilter(
        ema_alpha=float(f_cfg.get("ema_alpha", 0.35)),
        max_human_speed_mps=float(f_cfg.get("max_human_speed_mps", 3.0)),
        reject_large_jumps=bool(f_cfg.get("reject_large_jumps", True)),
        max_lost_frames=int(p_cfg.get("max_lost_frames", 15)),
        history_length=int(bev_cfg.get("trail_length", 80)),
    )

    # -------- output writers --------
    out_suffix = str(getattr(args, "output_suffix", "route_A") or "route_A")
    out_bev = str(output_dir / f"bev_tracking_{out_suffix}.mp4")
    out_bev_clean = str(output_dir / f"bev_tracking_clean_{out_suffix}.mp4")
    out_dbg = str(output_dir / f"debug_overlay_{out_suffix}.mp4")
    out_people_json = str(output_dir / f"people_tracks_{out_suffix}.json")
    out_cam_json = str(output_dir / f"camera_trajectory_{out_suffix}.json")
    out_val_json = str(output_dir / f"validation_summary_{out_suffix}.json")
    out_final_png = str(output_dir / f"final_frame_{out_suffix}.png")

    bev_writer = open_video_writer(out_bev, out_fps, (bev.W, bev.H))
    clean_writer = None
    if bool(cfg.get("output", {}).get("save_clean_video", True)):
        clean_writer = open_video_writer(out_bev_clean, out_fps, (bev.W, bev.H))
    dbg_writer = open_video_writer(out_dbg, out_fps, (src_w, src_h))

    # -------- run loop --------
    people_frames: List[dict] = []
    camera_poses_record: List[dict] = []
    cam_trail_bev: List[np.ndarray] = []
    pose_hit = pose_miss = proj_ok = proj_fail = 0

    live = bool(args.live)
    t0 = time.time()
    for src_idx, frame_bgr in iterate_frames(cap, max_frames=max_frames, frame_stride=frame_stride):
        ts = src_idx / src_fps if src_fps > 0 else float(src_idx)

        pose = nearest_pose(poses, ts, pose_tol)
        if pose is None:
            pose_miss += 1
        else:
            pose_hit += 1

        tracked = yolo.step(frame_bgr)
        debug_frame = frame_bgr.copy()
        frame_people: List[dict] = []

        for person in tracked:
            foot = compute_footpoint(
                person.mask, person.bbox_xyxy,
                image_shape=(src_h, src_w),
                bottom_percent=float(p_cfg.get("mask_bottom_percent", 0.05)),
                min_area=int(p_cfg.get("min_mask_area_px", 80)),
            )
            person.foot_pixel = foot
            color = _color_for_id(person.track_id)

            x1, y1, x2, y2 = [int(round(v)) for v in person.bbox_xyxy]
            cv2.rectangle(debug_frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(debug_frame, f"ID {person.track_id}",
                        (x1, max(0, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
            cv2.circle(debug_frame, (int(round(foot[0])), int(round(foot[1]))),
                       4, color, -1, cv2.LINE_AA)

            entry = {
                "track_id": int(person.track_id),
                "bbox_xyxy": person.bbox_xyxy.tolist(),
                "score": float(person.score),
                "foot_pixel": foot.tolist(),
                "world_xyz": None, "bev_xy": None,
                "filtered_bev_xy": None, "projected": False,
            }

            if pose is not None:
                # 相机系地面 (with pitch, height): 对头戴 + 中景行人稳定
                # (世界系地面对上半身检测会 lam<0 失败)
                gp_cfg = cfg.get("ground", {})
                Xw = intersect_footpoint_with_camera_ground(
                    foot, K, pose.T_wc,
                    camera_height=float(gp_cfg.get("fallback_camera_height", 0.1)),
                    camera_pitch_deg=float(gp_cfg.get("fallback_camera_pitch_deg", 15.0)),
                )
                if Xw is None:
                    # 兜底: 世界系
                    Xw = intersect_footpoint_with_ground(
                        foot, K, pose.T_wc, ground_normal, ground_d)
                if Xw is not None:
                    # 应用 R_align 让世界坐标对齐 static_map 底图
                    Xw_a = R_align @ np.asarray(Xw, dtype=np.float64)
                    bev_xy = select_bev_axes(Xw_a, bev_axes_cfg)
                    # V3: 应用 bev_alignment (mirror_y 等)
                    bev_xy = apply_bev_alignment_xy(bev_xy, align_cfg)
                    filtered, accepted = pf.update(
                        person.track_id, ts, src_idx, bev_xy, person.score)
                    entry["world_xyz"] = Xw.tolist()
                    entry["world_xyz_aligned"] = Xw_a.tolist()
                    entry["bev_xy"] = bev_xy.tolist()
                    entry["filtered_bev_xy"] = filtered.tolist()
                    entry["projected"] = True
                    entry["accepted"] = bool(accepted)
                    proj_ok += 1
                else:
                    proj_fail += 1
            frame_people.append(entry)

        pf.mark_unseen(src_idx)

        if pose is not None:
            cam_xyz = pose.T_wc[:3, 3]
            cam_xyz_a = R_align @ np.asarray(cam_xyz, dtype=np.float64)
            cam_bev = select_bev_axes(cam_xyz_a, bev_axes_cfg)
            # V3: bev_alignment
            cam_bev = apply_bev_alignment_xy(cam_bev, align_cfg)
            cam_trail_bev.append(cam_bev)
            # heading 也要 rotate + 应用同一 transform (translation 无)
            forward_w = pose.T_wc[:3, :3] @ np.array([0.0, 0.0, 1.0])
            forward_w_a = R_align @ forward_w
            heading = select_bev_axes(forward_w_a, bev_axes_cfg)
            heading = apply_bev_alignment_heading(heading, align_cfg)
            camera_poses_record.append({
                "frame_index": int(src_idx),
                "timestamp": float(ts),
                "T_wc": pose.T_wc.tolist(),
                "bev_xy": cam_bev.tolist(),
            })
        else:
            cam_bev = cam_trail_bev[-1] if cam_trail_bev else None
            heading = None

        people_history: Dict[int, List[Tuple[int, np.ndarray]]] = {}
        for tid, tr in pf.all_tracks().items():
            people_history[tid] = list(tr["history"])
        active_ids = list(pf.active_tracks().keys())

        bev_frame = bev.draw(
            camera_trail_world=cam_trail_bev,
            current_camera_world=cam_bev,
            camera_heading_world=heading,
            people_history=people_history,
            people_active_ids=active_ids,
            frame_index=src_idx,
            timestamp=ts,
            draw_camera_trail=bool(bev_cfg.get("draw_camera_trail", True)),
            draw_people_trails=bool(bev_cfg.get("draw_people_trails", False)),
        )
        clean_frame = None
        if clean_writer is not None:
            clean_frame = bev.draw(
                camera_trail_world=cam_trail_bev,
                current_camera_world=cam_bev,
                camera_heading_world=heading,
                people_history=people_history,
                people_active_ids=active_ids,
                frame_index=src_idx,
                timestamp=ts,
                draw_camera_trail=True,
                draw_people_trails=False,
            )

        bev_writer.write(bev_frame)
        if clean_writer is not None and clean_frame is not None:
            clean_writer.write(clean_frame)
        dbg_writer.write(debug_frame)

        people_frames.append({
            "frame_index": int(src_idx),
            "timestamp": float(ts),
            "pose_available": pose is not None,
            "people": frame_people,
        })

        if live:
            cv2.imshow("Route A BEV", bev_frame)
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                print("[pipeline_A] user quit via 'q'")
                break

        if (src_idx + 1) % 50 == 0:
            el = time.time() - t0
            rate = (src_idx + 1) / el if el > 0 else 0.0
            print(f"[pipeline_A] frame {src_idx+1}/{src_n}  "
                  f"pose {pose_hit}/{pose_miss}  proj {proj_ok}/{proj_fail}  {rate:.1f} fps")

    cap.release()
    bev_writer.release()
    if clean_writer is not None:
        clean_writer.release()
    dbg_writer.release()
    if live:
        cv2.destroyAllWindows()

    # 保存最后一帧 BEV 作 final_frame_*.png
    if 'bev_frame' in locals():
        try:
            cv2.imwrite(out_final_png, bev_frame)
        except Exception:
            pass

    write_json(out_people_json, {
        "video": video_path, "calib": calib_path, "pose": pose_path,
        "static_map": static_map_path,
        "ground_plane": best_gp,
        "video_size": [src_w, src_h], "video_fps": src_fps,
        "bev_axes": bev_axes_cfg,
        "pose_hit": pose_hit, "pose_miss": pose_miss,
        "projection_ok": proj_ok, "projection_fail": proj_fail,
        "frames": people_frames,
    })
    write_json(out_cam_json, {
        "video": video_path, "pose": pose_path,
        "poses": camera_poses_record,
    })
    write_json(out_val_json, {
        "pose_hit": pose_hit,
        "pose_miss": pose_miss,
        "projection_ok": proj_ok,
        "projection_fail": proj_fail,
        "occupied_ratio": meta["statistics"]["occupied_ratio"],
        "free_ratio": meta["statistics"]["free_ratio"],
        "unknown_ratio": meta["statistics"]["unknown_ratio"],
        "people_tracks": len({p["track_id"] for f in people_frames for p in f["people"] if p["projected"]}),
        "source_pose": pose_path,
        "source_pointcloud": meta.get("ground_plane", {}).get("source", "unknown"),
        "static_map_meta": static_map_meta_path,
    })

    el = time.time() - t0
    print("=" * 70)
    print(f"[pipeline_A] done in {el:.1f}s")
    print(f"[pipeline_A] pose hit/miss = {pose_hit}/{pose_miss}")
    print(f"[pipeline_A] projection ok/fail = {proj_ok}/{proj_fail}")
    print(f"[pipeline_A] outputs -> {output_dir}")
    return 0


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--config",
                   default=str(Path(__file__).resolve().parents[1] /
                               "config" / "route_A.yaml"))
    p.add_argument("--video", default=None)
    p.add_argument("--calib", default=None)
    p.add_argument("--pose", default=None)
    p.add_argument("--static-map", default=None)
    p.add_argument("--static-map-meta", default=None)
    p.add_argument("--ground-plane", default=None)
    p.add_argument("--output-dir", default=None)
    p.add_argument("--live", action="store_true", help="open cv2.imshow live window")
    p.add_argument("--max-frames", type=int, default=None)
    p.add_argument("--map-render-mode",
                   choices=["paper", "binary", "tricolor", "v1"],
                   default="paper",
                   help="static_layer 渲染模式 (V2 默认 paper)")
    p.add_argument("--output-suffix", default="route_A",
                   help="输出文件后缀, e.g. route_A / route_A_v2")
    args = p.parse_args()
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
