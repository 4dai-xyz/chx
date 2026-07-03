#!/usr/bin/env python3
"""V3.2 行人物理过滤 CLI.

输出:
  people_tracks_raw.json                    (原样拷贝, 便于对比)
  people_tracks_filtered.json               (含所有观测 + status)
  people_filter_metrics.json                (12 项质量指标)
  people_filter_report.md                   (中文报告)
  people_filter_debug_video.mp4             (调试版: accepted 绿 / corrected 蓝虚线 / rejected 红叉+原因)
  accepted_people_overlay.png               (最终帧, 只显示 accepted+corrected)
  rejected_people_overlay.png               (最终帧, 只显示 rejected + 原因标签)
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from people_bev_tracker.person_map_filter import (
    FilterConfig, filter_people_tracks_on_map, world_xy_to_grid_ij,
)
from people_bev_tracker.static_map import render_paper_style


def _load_grid_and_meta(static_map_npy: str, static_map_meta_json: str):
    grid = np.load(static_map_npy)
    meta = json.load(open(static_map_meta_json))
    return grid, meta


def _resolve(p, root):
    pp = Path(p)
    return str(pp if pp.is_absolute() else (root / p).resolve())


STATUS_COLOR = {
    "accepted":                 (60, 220, 100),    # green (BGR)
    "corrected_to_nearest_free":(255, 160, 60),    # cyan/blue
    "rejected_too_far":         (100, 100, 255),
    "rejected_too_close":       (100, 100, 255),
    "rejected_out_of_fov":      (0,   180, 255),
    "rejected_in_occupied":     (0,   40, 220),
    "rejected_in_unknown":      (60,  60, 220),
    "rejected_near_obstacle":   (0,   120, 255),
    "rejected_no_near_free_cell": (0, 0, 200),
    "rejected_outside_canvas":  (128, 128, 128),
    "rejected_track_too_short": (200, 0, 200),
    "rejected_speed_outlier":   (200, 0, 100),
    "rejected_low_confidence":  (200, 200, 60),
    "rejected_no_camera_pose":  (100, 100, 100),
}


def _draw_person(img, ij, status, tid, reason=None, mode="publish", raw_ij=None):
    px, py = int(ij[0]), int(ij[1])
    color = STATUS_COLOR.get(status, (200, 200, 200))
    if status == "accepted":
        cv2.circle(img, (px, py), 6, color, -1, cv2.LINE_AA)
        cv2.circle(img, (px, py), 7, (255, 255, 255), 1, cv2.LINE_AA)
        if tid is not None:
            cv2.putText(img, f"{tid}", (px + 8, py - 7),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
    elif status == "corrected_to_nearest_free":
        cv2.circle(img, (px, py), 6, color, -1, cv2.LINE_AA)
        cv2.circle(img, (px, py), 7, (255, 255, 255), 1, cv2.LINE_AA)
        if raw_ij is not None and mode == "debug":
            # 从 raw 到 corrected 画虚线
            _dashed_line(img, (int(raw_ij[0]), int(raw_ij[1])), (px, py), color, 1)
        if tid is not None:
            cv2.putText(img, f"{tid}*", (px + 8, py - 7),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
    else:
        if mode == "debug":
            # 红叉 + 短原因
            cv2.line(img, (px - 6, py - 6), (px + 6, py + 6), color, 2, cv2.LINE_AA)
            cv2.line(img, (px - 6, py + 6), (px + 6, py - 6), color, 2, cv2.LINE_AA)
            if reason:
                short = reason.split()[0].replace("rejected_", "")
                cv2.putText(img, short[:12], (px + 6, py + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA)


def _dashed_line(img, p1, p2, color, thick, dash=6):
    x1, y1 = p1; x2, y2 = p2
    d = math.hypot(x2 - x1, y2 - y1)
    if d < 1e-6:
        return
    n = int(d // (dash * 2))
    for i in range(n + 1):
        t0 = (i * 2) / max(1, (n + 1) * 2)
        t1 = (i * 2 + 1) / max(1, (n + 1) * 2)
        s = (int(x1 + (x2 - x1) * t0), int(y1 + (y2 - y1) * t0))
        e = (int(x1 + (x2 - x1) * t1), int(y1 + (y2 - y1) * t1))
        cv2.line(img, s, e, color, thick, cv2.LINE_AA)


def _render_final_overlay(bg, records, meta, filter_status_set, mode="publish"):
    img = bg.copy()
    # 找最后一帧的观测
    if not records:
        return img
    last_frame = max(int(r["frame_index"]) for r in records)
    for r in records:
        if int(r["frame_index"]) != last_frame:
            continue
        if r["status"] not in filter_status_set:
            continue
        xy = r.get("filtered_bev_xy") or r.get("raw_bev_xy")
        if xy is None:
            continue
        ij = world_xy_to_grid_ij(np.asarray(xy), meta)[0]
        raw_ij = world_xy_to_grid_ij(np.asarray(r["raw_bev_xy"]), meta)[0] if r.get("corrected_from") else None
        _draw_person(img, ij, r["status"], r["track_id"], r.get("rejected_reason"),
                     mode=mode, raw_ij=raw_ij)
    return img


def _make_debug_video(records, grid, meta, out_mp4, fps=29.417):
    from people_bev_tracker.static_map import render_tricolor
    bg = render_tricolor(grid)
    H, W = bg.shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    vw = cv2.VideoWriter(out_mp4, fourcc, fps, (W, H))
    # 按 frame_index 分组
    by_frame = {}
    for r in records:
        by_frame.setdefault(int(r["frame_index"]), []).append(r)
    frame_ids = sorted(by_frame.keys())
    for fi in frame_ids:
        img = bg.copy()
        # HUD
        cv2.rectangle(img, (0, 0), (W, 28), (16, 16, 16), -1)
        n_ok = sum(1 for r in by_frame[fi] if r["status"] in ("accepted", "corrected_to_nearest_free"))
        n_rej = len(by_frame[fi]) - n_ok
        cv2.putText(img, f"frame {fi}  accepted+corrected={n_ok}  rejected={n_rej}",
                    (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (240, 240, 240), 1, cv2.LINE_AA)
        for r in by_frame[fi]:
            xy = r.get("filtered_bev_xy") or r.get("raw_bev_xy")
            if xy is None:
                continue
            ij = world_xy_to_grid_ij(np.asarray(xy), meta)[0]
            raw_ij = world_xy_to_grid_ij(np.asarray(r["raw_bev_xy"]), meta)[0] if r.get("corrected_from") else None
            _draw_person(img, ij, r["status"], r["track_id"], r.get("rejected_reason"),
                         mode="debug", raw_ij=raw_ij)
        vw.write(img)
    vw.release()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--people-json", required=True,
                    help="V3 pipeline_A 输出的 people_tracks_route_A_v3_dense.json")
    ap.add_argument("--camera-json", required=True,
                    help="camera_trajectory_route_A_v3_dense.json")
    ap.add_argument("--static-map", required=True)
    ap.add_argument("--static-map-meta", required=True)
    ap.add_argument("--output-dir", required=True)
    ap.add_argument("--dpvo-units-per-meter", type=float, default=0.605)
    ap.add_argument("--min-distance-m", type=float, default=0.5)
    ap.add_argument("--max-distance-m", type=float, default=12.0)
    ap.add_argument("--half-fov-deg", type=float, default=70.0)
    ap.add_argument("--min-obstacle-clearance-m", type=float, default=0.30)
    ap.add_argument("--max-correction-m", type=float, default=0.80)
    ap.add_argument("--min-track-length", type=int, default=2)
    ap.add_argument("--max-speed-mps", type=float, default=3.0)
    ap.add_argument("--max-speed-mps-lenient", type=float, default=5.0)
    ap.add_argument("--min-confidence", type=float, default=0.20)
    ap.add_argument("--fps", type=float, default=29.417)
    args = ap.parse_args()

    root = Path("/home/ros/ros2_orbslam3")
    ppl = _resolve(args.people_json, root)
    cam = _resolve(args.camera_json, root)
    smn = _resolve(args.static_map, root)
    smm = _resolve(args.static_map_meta, root)
    out = Path(_resolve(args.output_dir, root)); out.mkdir(parents=True, exist_ok=True)

    grid, meta = _load_grid_and_meta(smn, smm)

    cfg = FilterConfig(
        dpvo_units_per_meter=args.dpvo_units_per_meter,
        min_distance_m=args.min_distance_m, max_distance_m=args.max_distance_m,
        half_fov_deg=args.half_fov_deg,
        min_obstacle_clearance_m=args.min_obstacle_clearance_m,
        max_correction_m=args.max_correction_m,
        min_track_length=args.min_track_length,
        max_speed_mps=args.max_speed_mps,
        max_speed_mps_lenient=args.max_speed_mps_lenient,
        min_confidence=args.min_confidence,
    )
    print(f"[filter] cfg = {cfg}")

    payload = filter_people_tracks_on_map(ppl, cam, grid, meta, cfg)
    records = payload["records"]; metrics = payload["metrics"]

    # 输出文件
    shutil.copyfile(ppl, out / "people_tracks_raw.json")
    (out / "people_tracks_filtered.json").write_text(
        json.dumps({"config": payload["config"], "records": records, "metrics": metrics},
                   ensure_ascii=False, indent=2))
    (out / "people_filter_metrics.json").write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2))

    # overlay 图
    from people_bev_tracker.static_map import render_tricolor
    bg = render_tricolor(grid)
    accepted_set = {"accepted", "corrected_to_nearest_free"}
    rejected_set = set(STATUS_COLOR.keys()) - accepted_set
    cv2.imwrite(str(out / "accepted_people_overlay.png"),
                _render_final_overlay(bg, records, meta, accepted_set, mode="publish"))
    cv2.imwrite(str(out / "rejected_people_overlay.png"),
                _render_final_overlay(bg, records, meta, rejected_set, mode="debug"))

    # 调试视频
    _make_debug_video(records, grid, meta, str(out / "people_filter_debug_video.mp4"),
                      fps=args.fps)

    # 中文报告
    m = metrics
    hard_pass = (m["people_in_occupied_ratio_after"] <= 1e-6
                 and m["people_in_unknown_ratio_after"] <= 0.05
                 and m["people_near_obstacle_ratio_after"] <= 0.05)
    tbl_status = {}
    for r in records:
        tbl_status[r["status"]] = tbl_status.get(r["status"], 0) + 1
    md_status = "\n".join(f"* {k}: {v}" for k, v in sorted(tbl_status.items(), key=lambda x: -x[1]))
    (out / "people_filter_report.md").write_text(f"""# 动态行人物理过滤报告 (V3.2)

## 1. 输入
* people_tracks (V3): `{ppl}`
* camera_trajectory: `{cam}`
* static_map: `{smn}` (mirror_y, obs>=3 + hybrid keep_obs2_near_obs3)

## 2. 过滤配置
* 距离范围: {cfg.min_distance_m} ~ {cfg.max_distance_m} m
* 视野半角: {cfg.half_fov_deg}°
* 最小障碍距离: {cfg.min_obstacle_clearance_m} m
* 最大修正距离: {cfg.max_correction_m} m
* 最小 track 长度: {cfg.min_track_length}
* 速度上限: {cfg.max_speed_mps} m/s (宽松 {cfg.max_speed_mps_lenient})
* 最小置信度: {cfg.min_confidence}
* DPVO 单位/米: {cfg.dpvo_units_per_meter}

## 3. 数量统计
* 原始观测数 (projected): {m['people_raw_count']}
* accepted: {m['people_accepted_count']}
* corrected_to_nearest_free: {m['people_corrected_count']}
* rejected: {m['people_rejected_count']}
* acceptance_ratio: {m['people_acceptance_ratio']*100:.1f}%
* 独立 track_id (输入): {m['unique_tracks_input']}
* 独立 track_id (保留): {m['unique_tracks_accepted']}

## 4. 硬性质量指标
| 指标 | 值 | 目标 | 达标 |
| :--- | :---: | :---: | :---: |
| people_in_occupied_ratio_after | {m['people_in_occupied_ratio_after']*100:.2f}% | 0% | {'✅' if m['people_in_occupied_ratio_after']<=1e-6 else '❌'} |
| people_in_unknown_ratio_after | {m['people_in_unknown_ratio_after']*100:.2f}% | ≤5% | {'✅' if m['people_in_unknown_ratio_after']<=0.05 else '❌'} |
| people_near_obstacle_ratio_after | {m['people_near_obstacle_ratio_after']*100:.2f}% | ≤5% | {'✅' if m['people_near_obstacle_ratio_after']<=0.05 else '❌'} |

## 5. 前后对比 (在地图上的分布)
| 指标 | before (raw) | after (kept only) |
| :--- | :---: | :---: |
| in_occupied_ratio | {m['people_in_occupied_ratio_before']*100:.1f}% | {m['people_in_occupied_ratio_after']*100:.2f}% |
| in_unknown_ratio | {m['people_in_unknown_ratio_before']*100:.1f}% | {m['people_in_unknown_ratio_after']*100:.2f}% |

## 6. 各状态分布
{md_status}

## 7. 其他
* track_id_switch_suspect_count (被短 track 过滤掉的 tid): {m['track_id_switch_suspect_count']}
* people_speed_outlier_count: {m['people_speed_outlier_count']}

## 8. 输出文件
* `people_tracks_raw.json` — 原始 V3 people_tracks 拷贝, 便于对比
* `people_tracks_filtered.json` — 每观测含 status + 原因 + 修正后坐标
* `people_filter_metrics.json`
* `people_filter_debug_video.mp4` — 调试版, accepted 绿 / corrected 蓝虚线 / rejected 红叉+原因
* `accepted_people_overlay.png` — 最终帧仅显示 accepted+corrected
* `rejected_people_overlay.png` — 最终帧 rejected 分布 + 原因

## 9. 硬性目标是否通过
**{'✅ 通过' if hard_pass else '⚠️ 未完全通过'}** (per 07 doc §3.2)
""", encoding="utf-8")

    print(f"[filter] done: raw={m['people_raw_count']} accepted={m['people_accepted_count']} "
          f"corrected={m['people_corrected_count']} rejected={m['people_rejected_count']}")
    print(f"[filter] in_occupied_after={m['people_in_occupied_ratio_after']*100:.2f}%  "
          f"in_unknown_after={m['people_in_unknown_ratio_after']*100:.2f}%  "
          f"near_obst_after={m['people_near_obstacle_ratio_after']*100:.2f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
