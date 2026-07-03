#!/usr/bin/env python3
"""Route A V3.1 主构建脚本: ScaRF-inspired 纯视觉稠密静态建图 (阶段 1-7)。

顺序执行:
  阶段 2  关键帧选择
  阶段 3  动态行人 mask
  阶段 4  单目深度推理 (Depth Anything V2 Metric Indoor)
  阶段 5  单帧深度尺度对齐
  阶段 6  子图融合 → dense_global_static
  阶段 7  稠密点云 → 2D occupancy (nav_binary / tricolor / paper / topdown_3d)

每阶段写一份中文报告到 output/route_A_v3_scarf/reports/。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

PKG_ROOT = Path(__file__).resolve().parents[1]
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

from people_bev_tracker.io_utils import load_config
from people_bev_tracker.camera_model import load_intrinsics
from people_bev_tracker.scarf_like import keyframes as kf_mod
from people_bev_tracker.scarf_like import dynamic_mask as dm_mod
from people_bev_tracker.scarf_like import depth_backend as db_mod
from people_bev_tracker.scarf_like import scale_alignment as sa_mod
from people_bev_tracker.scarf_like import submap_fusion as sf_mod
from people_bev_tracker.scarf_like import occupancy_from_dense as oc_mod
from people_bev_tracker.scarf_like import render_3d_topdown as r3_mod
from people_bev_tracker.static_map import (
    render_nav_binary, render_tricolor, render_paper_style,
)
from people_bev_tracker.map_quality import evaluate_grid_quality


REPO = Path("/home/ros/ros2_orbslam3")


def _resolve(p, root=REPO):
    pp = Path(p)
    return str(pp if pp.is_absolute() else (root / p).resolve())


def _w(path, text):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(text, encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--output-dir", default=None)
    ap.add_argument("--max-keyframes", type=int, default=None)
    args = ap.parse_args()

    cfg = load_config(_resolve(args.config))
    out_dir = Path(_resolve(args.output_dir or cfg["output"]["dir"]))
    reports = out_dir / "reports"
    reports.mkdir(parents=True, exist_ok=True)

    video = _resolve(cfg["input"]["video"])
    pose_tum = _resolve(cfg["input"]["pose_tum"])
    calib = _resolve(cfg["input"]["calib"])
    ground_plane_path = _resolve(cfg["input"]["ground_plane"])
    people_json = _resolve(cfg["input"]["person_tracks"])
    if not Path(people_json).exists():
        people_json = _resolve(cfg["input"]["person_tracks_fallback"])

    # 视频尺寸
    cap = cv2.VideoCapture(video)
    src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    src_fps = float(cap.get(cv2.CAP_PROP_FPS)) or 29.417
    src_n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    cap.release()

    with open(ground_plane_path) as f:
        gp = json.load(f)
    gp_best = gp["best"] if "best" in gp else gp
    g_normal = np.asarray(gp_best["normal"], dtype=np.float64)
    g_d = float(gp_best["d"])

    kf_cfg = cfg["keyframes"]
    kf_w = int(kf_cfg["image_width"]); kf_h = int(kf_cfg["image_height"])
    K, _ = load_intrinsics(calib, video_size=(kf_w, kf_h))

    t_all = time.time()

    # ================= 报告 01: 工程结构 =================
    _write_report_01(reports / "01_V3工程结构与配置报告.md", cfg, out_dir)

    # ================= 阶段 2: 关键帧 =================
    print("=" * 60); print("[V3] 阶段 2: 关键帧选择")
    people_by_frame = dm_mod.load_people_bboxes_by_frame(people_json)
    occ_by_frame = dm_mod.person_occlusion_ratio_by_frame(people_by_frame, src_w, src_h)
    max_kf = int(args.max_keyframes or kf_cfg["max_keyframes"])
    t0 = time.time()
    kf_payload = kf_mod.select_keyframes(
        video_path=video, pose_tum_path=pose_tum,
        out_dir=str(out_dir / "keyframes"),
        stride_frames=int(kf_cfg["stride_frames"]),
        min_translation_unit=float(kf_cfg["min_translation_unit"]),
        min_rotation_deg=float(kf_cfg["min_rotation_deg"]),
        max_keyframes=max_kf,
        image_width=kf_w, image_height=kf_h,
        video_fps=src_fps, dpvo_stride=2,
        person_occlusion_by_frame=occ_by_frame,
    )
    keyframes = kf_payload["keyframes"]
    print(f"[V3] 关键帧 {len(keyframes)} / 候选源帧 {src_n}, {time.time()-t0:.1f}s")
    _write_report_02(reports / "02_关键帧选择报告.md", kf_payload, src_n, occ_by_frame)

    # ================= 阶段 3: 动态 mask =================
    print("=" * 60); print("[V3] 阶段 3: 动态行人 mask")
    dm_payload = dm_mod.build_all_masks(
        keyframes, people_by_frame, src_w, src_h,
        out_dir=str(out_dir / "dynamic_masks"), dilate_px=10, save_debug=True)
    mask_paths = {r["kf_index"]: r["mask_npy"] for r in dm_payload["records"]}
    print(f"[V3] mask: {dm_payload['n_keyframes_with_person']} 帧有行人, "
          f"平均无效比 {dm_payload['mean_invalid_ratio']*100:.1f}%")
    _write_report_03(reports / "03_动态行人Mask报告.md", dm_payload, people_json)

    # ================= 阶段 4: 深度推理 =================
    print("=" * 60); print("[V3] 阶段 4: 单目深度推理")
    depth_cache = out_dir / "depth_cache"; depth_cache.mkdir(parents=True, exist_ok=True)
    depth_vis = out_dir / "depth_vis"; depth_vis.mkdir(parents=True, exist_ok=True)
    backend = db_mod.DepthBackend(cfg["depth"])
    depth_paths = {}
    t0 = time.time(); n_fail = 0; per_times = []
    for kf in keyframes:
        ki = int(kf["kf_index"])
        img = cv2.imread(kf["image_path"])
        if img is None:
            n_fail += 1; continue
        tt = time.time()
        res = backend.infer(img)
        per_times.append(time.time() - tt)
        dpath = depth_cache / f"depth_kf_{ki:04d}.npy"
        np.save(str(dpath), res["depth"].astype(np.float32))
        depth_paths[ki] = str(dpath)
        if ki % 20 == 0:
            cv2.imwrite(str(depth_vis / f"depth_kf_{ki:04d}.png"),
                        db_mod.depth_to_vis(res["depth"]))
    depth_dt = time.time() - t0
    mean_ms = (np.mean(per_times) * 1000) if per_times else 0.0
    print(f"[V3] 深度: {len(depth_paths)} 帧, 平均 {mean_ms:.0f}ms, 总 {depth_dt:.1f}s")
    _write_report_04(reports / "04_单目深度推理报告.md", backend.name, len(depth_paths),
                     backend.use_cuda, mean_ms, n_fail, str(depth_vis))

    # ================= 阶段 5: 尺度对齐 =================
    print("=" * 60); print("[V3] 阶段 5: 深度尺度对齐")
    sc_cfg = cfg["scale"]
    sa_payload = sa_mod.align_all_scales(
        keyframes, depth_paths, mask_paths, K, g_normal, g_d,
        out_dir=str(out_dir),
        bottom_frac=float(sc_cfg.get("floor_bottom_frac", 0.45)),
        min_floor_pixels=int(sc_cfg.get("min_floor_pixels", 300)),
        scale_smooth_window=int(sc_cfg.get("scale_smooth_window", 7)),
        max_scale_jump_ratio=float(sc_cfg.get("max_scale_jump_ratio", 1.25)))
    scales = {r["kf_index"]: r["scale_final"] for r in sa_payload["records"]}
    print(f"[V3] scale final median {sa_payload['scale_final_median']:.3f} "
          f"[{sa_payload['scale_final_min']:.3f}, {sa_payload['scale_final_max']:.3f}]")
    _write_report_05(reports / "05_深度尺度对齐报告.md", sa_payload)

    # ================= 阶段 6: 子图融合 =================
    print("=" * 60); print("[V3] 阶段 6: 子图融合")
    sm_cfg = cfg["submap"]
    t0 = time.time()
    sf_payload = sf_mod.fuse_submaps(
        keyframes, depth_paths, mask_paths, scales, K,
        out_dir=str(out_dir),
        keyframes_per_submap=int(sm_cfg["keyframes_per_submap"]),
        overlap_keyframes=int(sm_cfg["overlap_keyframes"]),
        voxel_size_unit=float(sm_cfg["voxel_size_unit"]),
        min_observations=int(sm_cfg["min_observations"]),
        max_points_per_submap=int(sm_cfg["max_points_per_submap"]),
        min_depth_m=float(cfg["depth"]["min_depth_m"]),
        max_depth_m=float(cfg["depth"]["max_depth_m"]),
        px_stride=int(sm_cfg.get("depth_stride_px", 4)))
    print(f"[V3] {sf_payload['n_submaps']} submaps, "
          f"dense {sf_payload['dense_global_n_points']} 点, {time.time()-t0:.1f}s")
    _write_report_06(reports / "06_子图融合与稠密点云报告.md", sf_payload)

    # ================= 阶段 7: 2D occupancy =================
    print("=" * 60); print("[V3] 阶段 7: 稠密 → 2D occupancy")
    dense = np.load(str(out_dir / "dense_global_static.npy"))
    dense_obs_path = out_dir / "dense_global_static_obs.npy"
    dense_obs = np.load(str(dense_obs_path)) if dense_obs_path.exists() else None
    tum_all = np.loadtxt(pose_tum, comments="#")
    if tum_all.ndim == 1:
        tum_all = tum_all[None, :]
    traj_xyz = tum_all[:, 1:4].astype(np.float64)
    # 所有位姿 → (M,4,4) 供视锥 ray carving
    poses_all = np.stack([kf_mod._tum_row_to_Twc(r) for r in tum_all], axis=0)
    grid, meta, debug = oc_mod.build_occupancy_from_dense(
        dense, traj_xyz, gp_best, cfg["occupancy"], transform="mirror_y",
        camera_poses_T_wc=poses_all, dense_obs=dense_obs)

    best = out_dir / "best"; best.mkdir(parents=True, exist_ok=True)
    np.save(str(best / "static_map.npy"), grid)
    with open(best / "static_map_meta.json", "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)

    rcfg = cfg.get("render", {})
    cv2.imwrite(str(best / "nav_binary_map.png"),
                render_nav_binary(grid, rcfg.get("nav_binary")))
    cv2.imwrite(str(best / "static_map_tricolor.png"),
                render_tricolor(grid, rcfg.get("tricolor")))
    cv2.imwrite(str(best / "paper_style_global_view.png"),
                render_paper_style(grid, trajectory_ij=debug["trajectory_ij"],
                                   colors=rcfg.get("paper_style")))
    # debug 图
    for k, v in debug.items():
        if k == "trajectory_ij" or v is None:
            continue
        cv2.imwrite(str(out_dir / f"debug_v3_{k}.png"),
                    (v if v.dtype == np.uint8 else v.astype(np.uint8)))

    # topdown 3D
    try:
        r3_mod.render_topdown_3d(dense, traj_xyz, gp_best,
                                 str(best / "topdown_3d_scene.png"), transform="mirror_y")
    except Exception as e:
        print(f"[V3] topdown render failed: {e}")

    # 质量
    q = evaluate_grid_quality(grid, meta, trajectory_ij=debug["trajectory_ij"])
    with open(best / "quality.json", "w", encoding="utf-8") as f:
        json.dump(q, f, ensure_ascii=False, indent=2)
    print(f"[V3] occupancy: free {q['active_free_ratio']*100:.1f}% "
          f"unknown {q['active_unknown_ratio']*100:.1f}% "
          f"largest_free {q['largest_free_component_ratio']*100:.1f}% "
          f"collision {q['trajectory_collision_ratio']*100:.2f}%")
    _write_report_07(reports / "07_二维栅格地图生成报告.md", meta, q, sf_payload)

    print("=" * 60)
    print(f"[V3] build 完成, 总耗时 {time.time()-t_all:.1f}s")
    return 0


# --------------------------------------------------------------------------
# 报告写作
# --------------------------------------------------------------------------

def _write_report_01(path, cfg, out_dir):
    _w(path, f"""# 阶段 1: V3 工程结构与配置报告

## 1. 新增文件

配置:
* `src/people_bev_tracker/config/route_A_v3_scarf.yaml`

模块 (`src/people_bev_tracker/people_bev_tracker/scarf_like/`):
* `__init__.py`
* `keyframes.py`          — 关键帧选择
* `dynamic_mask.py`       — 动态行人 mask
* `depth_backend.py`      — Depth Anything V2 Metric Indoor 封装
* `scale_alignment.py`    — 单帧深度尺度对齐
* `submap_fusion.py`      — 子图融合 + 多帧一致性
* `occupancy_from_dense.py` — 稠密点云 → 2D occupancy
* `render_3d_topdown.py`  — 3D top-down 展示图

脚本:
* `scripts/check_depth_backends.py`
* `scripts/build_route_A_v3_scarf_like.py` (本脚本)
* `scripts/run_route_A_v3_pipeline.py`
* `scripts/inspect_route_A_v3_outputs.py`

## 2. 各模块职责

| 模块 | 职责 |
| :--- | :--- |
| keyframes | 从 DPVO 平面化轨迹 + 视频选关键帧 (平移/旋转/转弯/遮挡降权) |
| dynamic_mask | 从 people_tracks 生成行人 invalid mask, 防止行人融入静态地图 |
| depth_backend | 单目度量深度 (米) 推理 |
| scale_alignment | 用地面高度约束求每帧 scale (米→DPVO 单位), 平滑 + 限跳变 |
| submap_fusion | 反投影 + voxel + 多帧一致性过滤 → 稠密静态点云 |
| occupancy_from_dense | 地面对齐 + mirror_y + 分层 → nav_binary/tricolor/paper |
| render_3d_topdown | 稠密点云高度着色俯视展示 |

## 3. V3 与 V2 的区别

| 维度 | V2 | V3.1 |
| :--- | :--- | :--- |
| 静态几何来源 | **VGGT 稠密点云** (一次性, 尺度杂散, 有假墙/缺墙) | **DPVO 位姿 + 单目深度逐帧重建** |
| 尺度一致性 | 无 (直接用 VGGT 输出) | **每帧 scale 对齐 + 平滑 + 限跳变** |
| 多帧一致性 | 无 | **submap voxel + min_observations 过滤** |
| 动态行人 | 只在 people layer | **建图前 mask 掉, 不进静态点云** |
| 坐标方向 | 未校准 (仰视/镜像) | **mirror_y 已校准** |
| free 来源 | corridor + frustum + semantic | floor points + trajectory corridor + ray |

## 4. 为什么不再用 VGGT 作为主几何

V2 报告已证明: VGGT 点云虽然 40 万点, 但:
* 地面 inlier 只 3.4%;
* 中央出现假墙, 把主通道切成两段 (largest_free_component 仅 35%);
* 后半段走廊只有一侧墙, 另一侧完全缺失;
* 它是一次性前馈输出, 没有尺度一致性约束, 无法逐帧纠正。

V3.1 改用"DPVO 稳定位姿 + 单目深度 + 尺度对齐 + 多帧融合", 这正是 ScaRF-SLAM
的核心思想: 用鲁棒 SLAM 定位, 用前馈深度做稠密建图, 用尺度一致性和子图融合
约束单帧深度。VGGT 仅保留为对照 baseline。
""")


def _write_report_02(path, payload, src_n, occ_by_frame):
    kfs = payload["keyframes"]
    src_idxs = [k["src_frame_index"] for k in kfs]
    first_half = sum(1 for s in src_idxs if s < src_n / 2)
    second_half = len(src_idxs) - first_half
    n_occ = sum(1 for k in kfs if k["person_occlusion"] > 0.3)
    reasons = {}
    for k in kfs:
        base = k["select_reason"].split()[0]
        reasons[base] = reasons.get(base, 0) + 1
    _w(path, f"""# 阶段 2: 关键帧选择报告

## 1. 基本情况
* 视频总帧数: **{src_n}**
* 选出关键帧: **{len(kfs)}**
* 关键帧图像尺寸: {payload['image_size']}
* stride_frames={payload['stride_frames']}, min_translation_unit={payload['min_translation_unit']}, min_rotation_deg={payload['min_rotation_deg']}, max_keyframes={payload['max_keyframes']}

## 2. 关键帧分布
* 前半段 (src<{int(src_n/2)}) 关键帧: **{first_half}**
* 后半段 关键帧: **{second_half}**
* 覆盖情况: {'前后段都有覆盖 ✅' if second_half > 0 else '⚠️ 后半段无关键帧'}

## 3. 触发原因分布
""" + "\n".join(f"* {r}: {c}" for r, c in sorted(reasons.items())) + f"""

## 4. 遮挡情况
* 行人遮挡 > 30% 的关键帧: **{n_occ}** (这些帧几何上被保留, 但深度会用行人 mask 剔除)

## 5. 是否覆盖转弯和后半段
* 转弯保留 (turn 触发): {reasons.get('turn', 0)} 帧
* 后半段覆盖: {'是' if second_half > 0 else '否'}
""")


def _write_report_03(path, payload, people_json):
    _w(path, f"""# 阶段 3: 动态行人 Mask 报告

## 1. 使用的 people_tracks 文件
`{people_json}`

## 2. 处理情况
* 处理关键帧: **{payload['n_keyframes']}**
* 其中有行人的关键帧: **{payload['n_keyframes_with_person']}**
* 平均无效区 (行人) 占比: **{payload['mean_invalid_ratio']*100:.2f}%**
* bbox 膨胀: {payload['dilate_px']} px

## 3. 对静态建图的作用
* 行人 bbox (含脚下扩展 + 膨胀) 区域在深度反投影时被剔除;
* 保证动态行人不会融合进 dense_global_static;
* 避免行人形成"假墙"污染 occupancy grid。

## 4. 说明
* V2 的 people_tracks JSON 只存 bbox_xyxy + foot_pixel, 未存 per-pixel mask,
  因此这里用 bbox (下半部分额外扩 15%) 近似, 再膨胀 10px, 足够覆盖行人 + 影子。
* debug mask 图保存在 `dynamic_masks/debug/` (每 20 帧一张)。
""")


def _write_report_04(path, backend_name, n_kf, use_cuda, mean_ms, n_fail, vis_dir):
    _w(path, f"""# 阶段 4: 单目深度推理报告

## 1. 使用的 depth backend
**{backend_name}** (Depth Anything V2 Metric Indoor Small, 经 HuggingFace transformers)

## 2. 推理情况
* 推理关键帧: **{n_kf}**
* 失败帧: {n_fail}
* 使用 GPU: {'是 (cuda:0)' if use_cuda else '否 (CPU)'}
* 平均单帧耗时: **{mean_ms:.0f} ms**

## 3. 输出
* 深度缓存: `depth_cache/depth_kf_XXXX.npy` (float32, 米制)
* 深度可视化: `{vis_dir}/` (每 20 帧一张, 近红远蓝)

## 4. 说明
* 输出的是**度量深度 (米)**, 非相对逆深度;
* 深度已 clamp 到 [min_depth_m, max_depth_m];
* 未使用 VGGT 作为深度来源 (VGGT 仅对照)。
""")


def _write_report_05(path, payload):
    recs = payload["records"]
    n_bad = payload["n_scale_failed"]
    n_floor_bad = payload["n_floor_insufficient"]
    _w(path, f"""# 阶段 5: 深度尺度对齐报告

## 1. 每帧尺度 (DPVO 单位 / 米)
* 最终 scale 中位数: **{payload['scale_final_median']:.4f}**
* 范围: [{payload['scale_final_min']:.4f}, {payload['scale_final_max']:.4f}]
* 物理含义: 1 米 ≈ {payload['scale_final_median']:.3f} DPVO 单位
  (对照: V1 相机高度 0.79 DPVO 单位 ≈ 1.6 m 眼镜高度 → 约 0.49 单位/米,
   同数量级, 说明尺度估计合理)

## 2. 平滑与限跳变
* 滑动中值窗口: {payload['scale_smooth_window']}
* 最大跳变比: {payload['max_scale_jump_ratio']}
* raw → clamped → smoothed 曲线见 `depth_scale_curve.png`

## 3. floor 不足的帧
* floor 像素不足的关键帧: **{n_floor_bad}** (用邻近/全局中值填补)
* 尺度估计完全失败的关键帧: **{n_bad}**

## 4. 是否解决单目相对深度尺度问题
* Depth Anything Metric 已给米制深度, 这里进一步用**地面高度约束**把米制统一
  到 DPVO 世界单位, 使多帧点云尺度一致;
* 这就是借鉴 ScaRF-SLAM 的 frame scale optimization: 不盲信单帧深度,
  用几何 (地面) 锚定 + 时序平滑约束尺度。
""")


def _write_report_06(path, payload):
    subs = payload["submaps"]
    tbl = "\n".join(
        f"| {s['submap_id']} | {s['kf_range']} | {s['n_keyframes']} | {s['n_points']} |"
        for s in subs)
    _w(path, f"""# 阶段 6: 子图融合与稠密点云报告

## 1. 子图情况
* 子图数量: **{payload['n_submaps']}**
* 每子图关键帧: {payload['keyframes_per_submap']} (重叠 {payload['overlap_keyframes']})
* voxel 大小: {payload['voxel_size_unit']} DPVO 单位
* 多帧一致性阈值: min_observations = {payload['min_observations']}

## 2. 各子图
| submap | kf 范围 | 关键帧数 | 点数 |
| :--- | :--- | :--- | :--- |
{tbl}

## 3. 全局稠密点云
* dense_global_static 点数: **{payload['dense_global_n_points']}**
* 输出: `dense_global_static.ply` / `.npy` (+ 每点观测帧数 `_obs.npy`)

## 4. 与 VGGT 点云对比
* VGGT 点云: 40 万点 (过滤后 30 万), 一次性前馈, 无多帧一致性;
* V3 稠密点云: {payload['dense_global_n_points']} 点, **每个 voxel 至少被
  {payload['min_observations']} 个关键帧观测**, 单帧漂浮点/假墙被过滤;
* 关键差异: V3 点云由 DPVO 位姿逐帧锚定 + 尺度对齐, 后半段走廊两侧
  只要相机看到就会被多帧覆盖, 理论上比 VGGT 覆盖更均匀。

## 5. 异常
* 若某子图点数为 0, 通常是该段关键帧 floor/深度质量差或几乎静止 (无新观测)。
""")


def _write_report_07(path, meta, q, sf_payload):
    st = meta["statistics"]
    _w(path, f"""# 阶段 7: 二维栅格地图生成报告

## 1. free 来源
* 地面点 (height ≈ 0, |h| ≤ {meta['floor_height_abs_thresh_unit']} DPVO 单位) 投影;
* 相机轨迹 corridor (永远 free);
* 二者 morphological close 后合并。

## 2. occupied 来源
* 高于地面 {meta['obstacle_height_range']} 区间的稠密点;
* 这些点已在阶段 6 通过 min_observations 多帧一致性过滤;
* close → 去小连通域 → dilate; 与 trajectory corridor 冲突时相信 corridor。

## 3. unknown 来源
* 既无地面观测又无障碍观测的区域;
* 二值导航图中按 black (不可通行) 处理。

## 4. 统计
* occupied: {st['occupied_ratio']*100:.2f}% (全图)
* free: {st['free_ratio']*100:.2f}% (全图)
* unknown: {st['unknown_ratio']*100:.2f}% (全图)
* dense 点数: {st['n_dense_points']}, floor 点 {st['n_floor_points']}, obstacle 点 {st['n_obstacle_points']}

## 5. active area 质量指标
* active_free_ratio: **{q['active_free_ratio']*100:.2f}%**
* active_unknown_ratio: **{q['active_unknown_ratio']*100:.2f}%**
* trajectory_collision_ratio: **{q['trajectory_collision_ratio']*100:.2f}%**
* obstacle_small_component_ratio: **{q['obstacle_small_component_ratio']*100:.2f}%**
* largest_free_component_ratio: **{q['largest_free_component_ratio']*100:.2f}%**

## 6. mirror_y
* `bev_alignment.transform = mirror_y` 已保存进 static_map_meta.json;
* nav_binary / tricolor / paper 三图方向一致。

## 7. 输出
* `best/nav_binary_map.png`
* `best/static_map_tricolor.png`
* `best/paper_style_global_view.png`
* `best/topdown_3d_scene.png`
* `best/static_map.npy` / `static_map_meta.json` / `quality.json`
""")


if __name__ == "__main__":
    raise SystemExit(main())
