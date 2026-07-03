"""V3.2 阶段: 动态行人物理过滤 (基于地图 + 相机视野 + 时序稳定性)。

按顺序执行 8 条过滤规则, 每个观测记录一个 status:
    accepted
    corrected_to_nearest_free
    rejected_too_far
    rejected_too_close
    rejected_out_of_fov
    rejected_in_occupied
    rejected_in_unknown
    rejected_near_obstacle
    rejected_no_near_free_cell
    rejected_track_too_short
    rejected_speed_outlier
    rejected_low_confidence

单位:
* BEV 坐标在 DPVO 单位; 米制换算用 `dpvo_units_per_meter` (V3 metric scale ≈ 0.605)。
* 距离阈值以米输入, 内部转 DPVO 单位。
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np


# ---------------------------------------------------------------------------
#  几何工具
# ---------------------------------------------------------------------------


def world_xy_to_grid_ij(xy: np.ndarray, meta: dict) -> np.ndarray:
    W = int(meta["width_px"]); H = int(meta["height_px"])
    r = float(meta["resolution_unit_per_px"])
    ox, oz = float(meta["origin_world"][0]), float(meta["origin_world"][1])
    xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    px = (W / 2 + (xy[:, 0] - ox) / r).astype(np.int64)
    py = (H / 2 - (xy[:, 1] - oz) / r).astype(np.int64)
    return np.stack([px, py], axis=1)


def compute_obstacle_distance_field(grid: np.ndarray) -> np.ndarray:
    """每个像素到最近 occupied (grid==255) 的欧氏距离 (像素)。"""
    inv = (grid != 255).astype(np.uint8) * 255
    # cv2.distanceTransform: 输入 uint8, 非零像素返回到最近 0 的距离; 我们要到 occupied (=255) 的距离,
    # 所以 mask 反一下: 让 occupied=0, 其它=255, 结果是各像素到最近 occupied 的距离
    return cv2.distanceTransform(inv, cv2.DIST_L2, 3)


def compute_nearest_free_offset(grid: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """对每个像素返回 (最近 free 像素的 dx, dy) (像素单位, dy 向下+, dx 向右+)。

    实现: distanceTransformWithLabels + 反查标签中心。
    简单版: 用 masked distance 找最近 free 索引。
    """
    H, W = grid.shape
    free = (grid == 127).astype(np.uint8) * 255
    # cv2.distanceTransform on 反 mask 得到 "到最近 free 的距离"
    inv_free = (grid != 127).astype(np.uint8) * 255
    dist, labels = cv2.distanceTransformWithLabels(
        inv_free, cv2.DIST_L2, 3, labelType=cv2.DIST_LABEL_PIXEL)
    # labels 是"最近零像素"的 label; 我们需要从 label 反查坐标
    # 找到 free==0 的位置 = 该 label 对应的 free 像素坐标
    # 简化: 直接返回 dist 图 + None; 用 dist 判可行, 修正用最近 free 位置 → 用另一个查询
    return dist, labels


def find_nearest_free_pixel(px: int, py: int, grid: np.ndarray, max_radius_px: int) -> Optional[Tuple[int, int]]:
    """在 max_radius_px 半径内找最近的 free (127) 像素, 用矩形搜索 + 距离排序。"""
    H, W = grid.shape
    x0 = max(0, px - max_radius_px); x1 = min(W, px + max_radius_px + 1)
    y0 = max(0, py - max_radius_px); y1 = min(H, py + max_radius_px + 1)
    patch = grid[y0:y1, x0:x1]
    ys, xs = np.where(patch == 127)
    if xs.size == 0:
        return None
    dx = xs - (px - x0); dy = ys - (py - y0)
    d2 = dx * dx + dy * dy
    k = int(np.argmin(d2))
    if math.sqrt(d2[k]) > max_radius_px:
        return None
    return int(x0 + xs[k]), int(y0 + ys[k])


# ---------------------------------------------------------------------------
#  过滤配置 + 状态
# ---------------------------------------------------------------------------


@dataclass
class FilterConfig:
    dpvo_units_per_meter: float = 0.605
    min_distance_m: float = 0.5
    max_distance_m: float = 12.0
    half_fov_deg: float = 70.0
    min_obstacle_clearance_m: float = 0.30
    max_correction_m: float = 0.80
    min_track_length: int = 2                 # 至少连续 N 帧
    max_speed_mps: float = 3.0
    max_speed_mps_lenient: float = 5.0
    min_confidence: float = 0.20


@dataclass
class PersonObservation:
    frame_index: int
    timestamp: float
    track_id: int
    confidence: float
    raw_bev_xy: List[float]
    filtered_bev_xy: Optional[List[float]]
    camera_distance_m: float
    status: str
    map_cell_type: str
    nearest_obstacle_distance_m: float
    rejected_reason: Optional[str] = None
    corrected_from: Optional[List[float]] = None


# ---------------------------------------------------------------------------
#  主过滤函数
# ---------------------------------------------------------------------------


def _cell_type(grid: np.ndarray, px: int, py: int) -> str:
    H, W = grid.shape
    if not (0 <= px < W and 0 <= py < H):
        return "outside"
    v = int(grid[py, px])
    return {0: "unknown", 127: "free", 255: "occupied"}.get(v, "unknown")


def filter_people_tracks_on_map(
    people_json_path: str,
    camera_json_path: str,
    static_map: np.ndarray,
    static_map_meta: dict,
    cfg: FilterConfig,
) -> Dict:
    """按帧过滤 people_tracks。返回 (records, metrics)。

    坐标系: people_tracks 里的 `filtered_bev_xy` (V2 存的) 是 mirror_y **之前**的
    aligned BEV; 我们要先应用 mirror_y (跟 static_map 一致)。camera_json 同理。
    """
    from .bev_alignment import apply_bev_alignment_xy

    align_cfg = static_map_meta.get("bev_alignment", {"enabled": True, "transform": "mirror_y"})
    res = float(static_map_meta["resolution_unit_per_px"])
    upm = float(cfg.dpvo_units_per_meter)
    min_dist_u = cfg.min_distance_m * upm
    max_dist_u = cfg.max_distance_m * upm
    min_clear_u = cfg.min_obstacle_clearance_m * upm
    max_corr_u = cfg.max_correction_m * upm
    max_corr_px = int(max_corr_u / res)
    min_clear_px = min_clear_u / res

    # 障碍距离场 (像素单位)
    obst_dist_px = compute_obstacle_distance_field(static_map)

    # 相机每帧 BEV 位置 + heading  (raw json 保存的是**未 mirror_y** 的 aligned BEV)
    cam_data = json.load(open(camera_json_path))
    cam_bev_by_frame: Dict[int, Tuple[np.ndarray, np.ndarray]] = {}
    for pose in cam_data.get("poses", []):
        fi = int(pose["frame_index"])
        raw_xy = np.asarray(pose["bev_xy"], dtype=np.float64)
        # 相机 BEV 应 mirror_y
        m_xy = apply_bev_alignment_xy(raw_xy.reshape(1, 2), align_cfg)[0]
        # heading: T_wc[:3,:3] @ (0,0,1) → aligned BEV → mirror
        T = np.asarray(pose["T_wc"], dtype=np.float64)
        R_align = np.asarray(static_map_meta.get("R_align"), dtype=np.float64)
        fwd_w = T[:3, :3] @ np.array([0.0, 0.0, 1.0])
        fwd_a = R_align @ fwd_w
        fwd_bev_raw = np.array([fwd_a[0], fwd_a[2]], dtype=np.float64)
        fwd_bev = apply_bev_alignment_xy(fwd_bev_raw.reshape(1, 2), align_cfg)[0]
        cam_bev_by_frame[fi] = (m_xy, fwd_bev)

    # people
    ppl_data = json.load(open(people_json_path))
    records: List[PersonObservation] = []
    track_history: Dict[int, List[Tuple[int, np.ndarray]]] = {}   # track_id -> [(frame, xy), ...]

    for frame in ppl_data.get("frames", []):
        fi = int(frame["frame_index"])
        ts = float(frame["timestamp"])
        cam = cam_bev_by_frame.get(fi)
        for pr in frame.get("people", []):
            if not pr.get("projected"):
                continue
            tid = int(pr["track_id"])
            conf = float(pr.get("score", 0.0))
            raw_xy_v2 = pr.get("filtered_bev_xy") or pr.get("bev_xy")
            if raw_xy_v2 is None:
                continue
            raw_xy_pre_mirror = np.asarray(raw_xy_v2, dtype=np.float64)
            # 应用 mirror_y (V2 json 是 V2 pipeline 里已经 mirror_y'ed 的 BEV; V2 pipeline
            # 应用 R_align 后 select_bev_axes, 未应用 mirror_y。但 V3 pipeline_A 后来在同一
            # 输入下写 people_tracks_route_A_v3_dense.json 时已经用了 alignment,
            # 所以坐标已经 mirror_y'ed。这里就直接当已 mirror_y'ed 处理, 不再变换。)
            xy = raw_xy_pre_mirror

            # Rule 1: 距离
            if cam is None:
                st = "rejected_no_camera_pose"; d_m = float("nan")
                _push(records, fi, ts, tid, conf, xy.tolist(), None, d_m, st, "n/a", float("nan"), st)
                continue
            cam_xy, cam_fwd = cam
            dv = xy - cam_xy
            d_u = float(np.linalg.norm(dv))
            d_m = d_u / max(upm, 1e-9)
            if d_u < min_dist_u:
                _push(records, fi, ts, tid, conf, xy.tolist(), None, d_m,
                      "rejected_too_close", "n/a", float("nan"), "rejected_too_close"); continue
            if d_u > max_dist_u:
                _push(records, fi, ts, tid, conf, xy.tolist(), None, d_m,
                      "rejected_too_far", "n/a", float("nan"), "rejected_too_far"); continue

            # Rule 2: FOV
            fn = float(np.linalg.norm(cam_fwd))
            if fn < 1e-9:
                cos_h = 1.0
            else:
                cos_h = float(np.dot(dv, cam_fwd) / (d_u * fn))
            angle = math.degrees(math.acos(max(-1.0, min(1.0, cos_h))))
            if angle > cfg.half_fov_deg:
                _push(records, fi, ts, tid, conf, xy.tolist(), None, d_m,
                      "rejected_out_of_fov", "n/a", float("nan"), f"rejected_out_of_fov ({angle:.0f}deg)"); continue

            # Rule 3 & 4 & 5: map cell + obstacle clearance + correction
            ij = world_xy_to_grid_ij(xy, static_map_meta)[0]
            px, py = int(ij[0]), int(ij[1])
            ct = _cell_type(static_map, px, py)
            if ct == "free":
                dist_obst_px = float(obst_dist_px[py, px]) if 0 <= px < static_map.shape[1] and 0 <= py < static_map.shape[0] else 0.0
                dist_obst_m = dist_obst_px * res / max(upm, 1e-9)
                if dist_obst_px < min_clear_px:
                    _push(records, fi, ts, tid, conf, xy.tolist(), None, d_m,
                          "rejected_near_obstacle", "free", dist_obst_m,
                          f"rejected_near_obstacle ({dist_obst_m:.2f}m)"); continue
                # 通过 → accepted
                _push(records, fi, ts, tid, conf, xy.tolist(), xy.tolist(), d_m,
                      "accepted", "free", dist_obst_m, None)
                track_history.setdefault(tid, []).append((fi, xy))
                continue

            # ct in {occupied, unknown, outside}: 尝试修正到最近 free
            nearest = find_nearest_free_pixel(px, py, static_map, max_corr_px)
            if nearest is None:
                st = "rejected_in_occupied" if ct == "occupied" else \
                     "rejected_in_unknown" if ct == "unknown" else "rejected_outside_canvas"
                _push(records, fi, ts, tid, conf, xy.tolist(), None, d_m, st, ct, float("nan"),
                      st); continue
            # 检查修正后的障碍距离
            nx, ny = nearest
            dist_obst_px = float(obst_dist_px[ny, nx])
            dist_obst_m = dist_obst_px * res / max(upm, 1e-9)
            if dist_obst_px < min_clear_px:
                _push(records, fi, ts, tid, conf, xy.tolist(), None, d_m,
                      "rejected_near_obstacle", ct, dist_obst_m,
                      f"rejected_near_obstacle_after_correction ({dist_obst_m:.2f}m)"); continue
            # 反算修正后的 world xy
            W = int(static_map_meta["width_px"]); H = int(static_map_meta["height_px"])
            ox, oz = float(static_map_meta["origin_world"][0]), float(static_map_meta["origin_world"][1])
            corr_x = ox + (nx - W / 2) * res
            corr_y = oz - (ny - H / 2) * res
            corr_xy = np.array([corr_x, corr_y])
            _push(records, fi, ts, tid, conf, xy.tolist(), corr_xy.tolist(), d_m,
                  "corrected_to_nearest_free", "free", dist_obst_m, None, corrected_from=xy.tolist())
            track_history.setdefault(tid, []).append((fi, corr_xy))

    # ------- 后处理: track 长度 + 速度 outlier -------
    #   track_history 里保存的是当前"accepted/corrected" 观测; 若某 track 累计出现次数 <
    #   min_track_length, 把它已经 accept 的降级为 rejected_track_too_short.
    kept_tids = {tid: hist for tid, hist in track_history.items() if len(hist) >= cfg.min_track_length}
    accepted_tids = set(kept_tids.keys())
    for r in records:
        if r.status in ("accepted", "corrected_to_nearest_free") and r.track_id not in accepted_tids:
            r.status = "rejected_track_too_short"
            r.rejected_reason = f"rejected_track_too_short (< {cfg.min_track_length} frames)"

    # 速度 outlier: 依 track_history 计算相邻速度, 标 outlier 帧
    tid_frame_to_speed: Dict[Tuple[int, int], float] = {}
    for tid, hist in kept_tids.items():
        hist = sorted(hist, key=lambda x: x[0])
        for i in range(1, len(hist)):
            f0, p0 = hist[i - 1]; f1, p1 = hist[i]
            dt = max(1e-3, (f1 - f0) / 29.417)   # 假设 29.4 fps
            v_u_per_s = float(np.linalg.norm(p1 - p0)) / dt
            v_mps = v_u_per_s / max(upm, 1e-9)
            if v_mps > cfg.max_speed_mps_lenient:
                tid_frame_to_speed[(tid, f1)] = v_mps
    for r in records:
        if r.status in ("accepted", "corrected_to_nearest_free"):
            v = tid_frame_to_speed.get((r.track_id, r.frame_index))
            if v is not None:
                r.status = "rejected_speed_outlier"
                r.rejected_reason = f"rejected_speed_outlier ({v:.1f}m/s)"

    # 低置信度过滤
    for r in records:
        if r.status in ("accepted", "corrected_to_nearest_free") and r.confidence < cfg.min_confidence:
            r.status = "rejected_low_confidence"
            r.rejected_reason = f"rejected_low_confidence ({r.confidence:.2f})"

    metrics = _compute_metrics(records, cfg)
    return {
        "records": [asdict(r) for r in records],
        "metrics": metrics,
        "config": asdict(cfg),
        "cell_types_before_after": {
            "n_total": len(records),
        },
    }


def _push(records, fi, ts, tid, conf, raw_xy, filt_xy, d_m, st, mc, dob, reason, corrected_from=None):
    records.append(PersonObservation(
        frame_index=int(fi), timestamp=float(ts), track_id=int(tid),
        confidence=float(conf), raw_bev_xy=list(raw_xy),
        filtered_bev_xy=(list(filt_xy) if filt_xy is not None else None),
        camera_distance_m=float(d_m), status=st, map_cell_type=mc,
        nearest_obstacle_distance_m=float(dob), rejected_reason=reason,
        corrected_from=corrected_from,
    ))


def _compute_metrics(records: List[PersonObservation], cfg: FilterConfig) -> dict:
    N = len(records)
    n_accept = sum(1 for r in records if r.status == "accepted")
    n_corr = sum(1 for r in records if r.status == "corrected_to_nearest_free")
    n_rej = N - n_accept - n_corr
    # before: 原始 bev cell 分布
    n_before_occ = sum(1 for r in records if r.map_cell_type == "occupied")
    n_before_unk = sum(1 for r in records if r.map_cell_type == "unknown")
    # after: 只看被保留的 (accepted+corrected)
    n_after_ok = n_accept + n_corr
    n_after_occ = sum(1 for r in records if r.status == "accepted" and r.map_cell_type == "occupied")
    n_after_unk = sum(1 for r in records if r.status == "accepted" and r.map_cell_type == "unknown")
    n_after_near = sum(
        1 for r in records
        if r.status in ("accepted", "corrected_to_nearest_free")
        and math.isfinite(r.nearest_obstacle_distance_m)
        and r.nearest_obstacle_distance_m < cfg.min_obstacle_clearance_m
    )
    tids_kept = {r.track_id for r in records if r.status in ("accepted", "corrected_to_nearest_free")}
    tids_all = {r.track_id for r in records}
    return {
        "people_raw_count": N,
        "people_accepted_count": n_accept,
        "people_corrected_count": n_corr,
        "people_rejected_count": n_rej,
        "people_acceptance_ratio": (n_after_ok / max(1, N)),
        "people_in_occupied_ratio_before": n_before_occ / max(1, N),
        "people_in_occupied_ratio_after": n_after_occ / max(1, n_after_ok),
        "people_in_unknown_ratio_before": n_before_unk / max(1, N),
        "people_in_unknown_ratio_after": n_after_unk / max(1, n_after_ok),
        "people_near_obstacle_ratio_after": n_after_near / max(1, n_after_ok),
        "track_id_switch_suspect_count": len(tids_all - tids_kept),
        "people_speed_outlier_count": sum(1 for r in records if r.status == "rejected_speed_outlier"),
        "unique_tracks_accepted": len(tids_kept),
        "unique_tracks_input": len(tids_all),
    }
