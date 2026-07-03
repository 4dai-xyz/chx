"""V3.1 阶段 7 附加: 3D 稠密点云 top-down 俯视展示图 (类 1811.10092 Figure 1)。

把 dense_global_static 点云按高度着色, 从正上方投影, 叠加相机轨迹和行人。
纯 matplotlib/numpy 实现, 不依赖显示。
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional

import numpy as np

from ..bev_alignment import apply_bev_alignment_xy
from ..static_map import _rot_matrix_align_a_to_b


def render_topdown_3d(
    dense_pts_world: np.ndarray,
    trajectory_world_xyz: np.ndarray,
    ground_plane: Dict,
    out_path: str,
    transform: str = "mirror_y",
    people_world_xyz: Optional[np.ndarray] = None,
    max_points: int = 200000,
) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    n = np.asarray(ground_plane["normal"], dtype=np.float64).reshape(3)
    d = float(ground_plane["d"])
    n = n / (np.linalg.norm(n) + 1e-12)
    R_align = _rot_matrix_align_a_to_b(n, np.array([0.0, 1.0, 0.0]))
    align_cfg = {"enabled": True, "transform": transform}

    def to_bev(pw):
        pa = (R_align @ pw.T).T
        h = pa[:, 1] + d
        xz = np.stack([pa[:, 0], pa[:, 2]], axis=1)
        xz = apply_bev_alignment_xy(xz, align_cfg)
        return xz, h

    pts = dense_pts_world
    if pts.shape[0] > max_points:
        idx = np.random.default_rng(0).choice(pts.shape[0], max_points, replace=False)
        pts = pts[idx]
    xz, h = to_bev(pts)
    traj_xz, cam_h = to_bev(trajectory_world_xyz)
    # 若相机在负 h 侧, 高度着色翻转符号使地面在下
    if np.median(cam_h) < 0:
        h = -h

    fig, ax = plt.subplots(figsize=(12, 10))
    # 按高度着色: 低=地面(浅), 高=障碍(深)
    order = np.argsort(h)   # 低的先画
    sc = ax.scatter(xz[order, 0], xz[order, 1], c=h[order], s=1.2,
                    cmap="cividis", alpha=0.6, linewidths=0)
    ax.plot(traj_xz[:, 0], traj_xz[:, 1], "-", color="#ff7800", lw=2.0, label="camera trajectory")
    ax.plot(traj_xz[0, 0], traj_xz[0, 1], "o", color="lime", ms=10, label="start")
    ax.plot(traj_xz[-1, 0], traj_xz[-1, 1], "o", color="red", ms=10, label="end")
    if people_world_xyz is not None and people_world_xyz.shape[0]:
        pxz, _ = to_bev(people_world_xyz)
        ax.plot(pxz[:, 0], pxz[:, 1], "o", color="#00b0ff", ms=6, label="people")
    ax.set_aspect("equal")
    ax.set_xlabel("BEV x (DPVO unit, mirror_y)")
    ax.set_ylabel("BEV z (DPVO unit, mirror_y)")
    ax.set_title("V3.1 dense static reconstruction — top-down view")
    ax.legend(loc="upper right", fontsize=9)
    cb = fig.colorbar(sc, ax=ax, shrink=0.7)
    cb.set_label("height above ground (DPVO unit)")
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
