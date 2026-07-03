"""scarf_like — Route A V3.1 ScaRF-inspired 纯视觉稠密静态建图。

核心原则 (借鉴 ScaRF-SLAM, 不复现官方代码):
    DPVO 稳定位姿 (不变)
      + 单目深度模型 (Depth Anything V2 Metric Indoor) 只管稠密建图
      + 单帧深度尺度对齐 (用地面高度约束, 借鉴 frame scale optimization)
      + 子图融合多帧一致性 (借鉴 submap + projection consistency)
    -> dense static point cloud -> 2D occupancy grid

不使用 VGGT 点云作为主几何来源 (只作对照)。
不修改任何官方 project code。
统一继承 mirror_y BEV 坐标方向。
"""

__all__ = [
    "keyframes",
    "depth_backend",
    "dynamic_mask",
    "scale_alignment",
    "submap_fusion",
    "occupancy_from_dense",
    "render_3d_topdown",
]
