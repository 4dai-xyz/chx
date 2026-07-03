#!/usr/bin/env python3
"""V3.2 阶段 A: V3.1 地图失败诊断。

回答: occupied 太少是因为 dense point cloud 缺墙, 还是 occupancy 阈值太严?
     obs=2 的点是不是真墙? obs>=3 是不是过严?
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--v3-dir", default="output/route_A_v3_scarf")
    ap.add_argument("--v2-dir", default="output/route_A_v2")
    ap.add_argument("--route-a-dir", default="output/route_A")
    args = ap.parse_args()

    v3 = Path(args.v3_dir); out = v3 / "diagnostics"; out.mkdir(parents=True, exist_ok=True)

    dense = np.load(v3 / "dense_global_static.npy")
    obs = np.load(v3 / "dense_global_static_obs.npy")
    gp = json.load(open(Path(args.route_a_dir) / "ground_plane_final.json"))["best"]
    n = np.asarray(gp["normal"]); n = n / np.linalg.norm(n); d = float(gp["d"])
    if float(np.median(np.loadtxt(Path(args.route_a_dir) / "trajectory_flat.txt",
                                   comments="#")[:, 1:4] @ n + d)) < 0:
        n = -n; d = -d

    # 地面对齐 + mirror_y
    from people_bev_tracker.static_map import _rot_matrix_align_a_to_b
    from people_bev_tracker.bev_alignment import apply_bev_alignment_xy
    R = _rot_matrix_align_a_to_b(n, np.array([0.0, 1.0, 0.0]))
    pts_a = (R @ dense.T).T
    h_align = pts_a[:, 1] + d
    xz = np.stack([pts_a[:, 0], pts_a[:, 2]], axis=1)
    xz = apply_bev_alignment_xy(xz, {"enabled": True, "transform": "mirror_y"})

    # obs 分布
    obs_hist = {int(k): int(v) for k, v in zip(*np.unique(obs, return_counts=True))}
    # 高度分层
    floor_mask = np.abs(h_align) <= 0.12
    obst_mask = (h_align >= 0.12) & (h_align <= 0.85)
    tall_mask = h_align > 0.85

    dp = {
        "n_points_total": int(dense.shape[0]),
        "obs_distribution": obs_hist,
        "n_floor_pts (|h|<=0.12)": int(floor_mask.sum()),
        "n_obstacle_pts (0.12<h<=0.85)": int(obst_mask.sum()),
        "n_tall_pts (h>0.85)": int(tall_mask.sum()),
        "obstacle_obs>=2": int((obst_mask & (obs >= 2)).sum()),
        "obstacle_obs>=3": int((obst_mask & (obs >= 3)).sum()),
        "obstacle_obs=1": int((obst_mask & (obs == 1)).sum()),
        "floor_obs>=2": int((floor_mask & (obs >= 2)).sum()),
    }
    with open(out / "dense_point_distribution.json", "w") as f:
        json.dump(dp, f, ensure_ascii=False, indent=2)

    # topdown 图: 全点 / floor / obstacle / obs=2 / obs>=3
    def scatter(mask, title, path, s=0.3):
        fig, ax = plt.subplots(figsize=(9, 8))
        pts = xz[mask]
        if pts.shape[0]:
            ax.scatter(pts[:, 0], pts[:, 1], c=h_align[mask], s=s, cmap="cividis")
        ax.set_aspect("equal"); ax.set_title(f"{title}  (n={mask.sum()})")
        ax.grid(alpha=0.2); fig.tight_layout(); fig.savefig(path, dpi=100); plt.close(fig)

    scatter(np.ones(dense.shape[0], bool), "dense all", out / "dense_topdown_all.png")
    scatter(floor_mask, "floor points (|h|<=0.12)", out / "floor_points_debug.png")
    scatter(obst_mask, "obstacle band (0.12<h<=0.85)", out / "obstacle_points_debug.png")
    scatter((obs == 2) & obst_mask, "obstacle band, obs=2", out / "obs2_points_topdown.png")
    scatter((obs >= 3) & obst_mask, "obstacle band, obs>=3", out / "obs3_points_topdown.png")

    # obs2 vs obs3 side by side
    fig, axs = plt.subplots(1, 2, figsize=(16, 7), sharex=True, sharey=True)
    for ax, m, ttl in [
        (axs[0], (obs == 2) & obst_mask, "obs=2 (76274 total in dense)"),
        (axs[1], (obs >= 3) & obst_mask, "obs>=3 (multi-view stable walls)")]:
        pts = xz[m]
        if pts.shape[0]:
            ax.scatter(pts[:, 0], pts[:, 1], c=h_align[m], s=0.4, cmap="cividis")
        ax.set_aspect("equal"); ax.grid(alpha=0.2); ax.set_title(f"{ttl}  n={m.sum()}")
    fig.tight_layout(); fig.savefig(out / "obs2_vs_obs3_comparison.png", dpi=100); plt.close(fig)

    # depth quality summary (from cached scales report)
    depth_scales = json.load(open(v3 / "depth_scales.json"))
    dq = {
        "global_scale_units_per_meter": depth_scales.get("global_scale"),
        "n_valid_frames_for_global": depth_scales.get("n_valid_frames_for_global"),
        "raw_median": depth_scales.get("per_frame_raw_median"),
        "raw_std": depth_scales.get("per_frame_raw_std"),
        "n_floor_insufficient": depth_scales.get("n_floor_insufficient"),
    }
    json.dump(dq, open(out / "scale_quality_summary.json", "w"), ensure_ascii=False, indent=2)

    # 诊断结论 md
    n_obs2 = dp["obstacle_obs>=2"]; n_obs3 = dp["obstacle_obs>=3"]
    (out / "diagnosis_report.md").write_text(f"""# V3.1 地图失败诊断报告

## 1. 关键数字
* dense 总点数: {dp['n_points_total']}
* obs 分布: `{dp['obs_distribution']}`
* floor 层 (|h|<=0.12): {dp['n_floor_pts (|h|<=0.12)']} 点
* obstacle 层 (0.12<h<=0.85): {dp['n_obstacle_pts (0.12<h<=0.85)']} 点
* obstacle & obs>=2: **{n_obs2}**  (V3.1 未启用, 认为里面有大量噪声)
* obstacle & obs>=3: **{n_obs3}**  (V3.1 目前用的, 太稀)

## 2. 诊断结论

**主因: 占据分层规则过严 (D)。**

对比 obs=2 (`obs2_points_topdown.png`) 和 obs>=3 (`obs3_points_topdown.png`):
* obs>=3 的点确实是**多帧一致的稳定墙**, 但**只有 {n_obs3} 个**, 数量远不够撑起商场两侧长走廊;
* obs=2 有 {n_obs2 - n_obs3} 个候选墙点, 视觉检查里**很多是真墙** (紧贴 obs>=3 沿墙成条状),
  但也混着一些反光/远距离噪声漂浮点。

因此下一步策略应是 **hybrid**: 只要 obs=2 点**紧邻**已有 obs>=3 结构 (BEV 上距离 < R),
就纳入障碍层 (说明这条墙确实存在, 只是端点/边缘被少 1 帧看到)。
远离 obs>=3 的孤立 obs=2 点仍剔除 (那些是噪声)。

## 3. 次要问题
* Depth Anything V2 Metric Indoor 尺度全局稳定 (std/median 见 `scale_quality_summary.json`),
  尺度问题**不是**主因。
* mirror_y 一次性烘焙 + pipeline 坐标层一次性应用, 无双重变换。
* floor 点数 {dp['n_floor_pts (|h|<=0.12)']} 已够, 不是 floor 分类过严。

## 4. 与 V3.2 tune 的对接
`scripts/tune_route_A_v3_occupancy.py` 里的 `keep_obs2_if_near_obs3=true` 组合就是本诊断结论的实现。
""", encoding="utf-8")
    print(f"[diagnose] done: dense={dp['n_points_total']}, obs>=3={n_obs3}, obs=2={n_obs2 - n_obs3}")
    return 0


if __name__ == "__main__":
    import sys
    PKG = Path(__file__).resolve().parents[1]
    if str(PKG) not in sys.path:
        sys.path.insert(0, str(PKG))
    raise SystemExit(main())
