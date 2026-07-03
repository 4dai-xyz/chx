# Route A V3.2 执行方案：官方 ScaRF-SLAM 基线复现 + 地图失败诊断 + 1811 Figure 1 风格展示准备

## 0. 当前结论

V3.1 已经完成：

```text
DPVO 主轨迹
  + mirror_y BEV 坐标校准
  + Depth Anything V2 Metric Indoor
  + 动态行人 mask
  + 全局尺度对齐
  + 子图多帧一致性融合
  + 2D occupancy
```

V3.1 的定量指标比 V2 有明显改善：

```text
active_free_ratio:              37.24% -> 49.88%
active_unknown_ratio:           49.51% -> 48.01%
largest_free_component_ratio:   35.12% -> 48.70%
trajectory_collision_ratio:      0.00% -> 0.00%
central_false_wall_removed:      true
```

但是用户主观观察仍然不满意：

```text
1. 黑白栅格不像清晰可导航地图。
2. 障碍和可通行区域标注不明显。
3. 三值图里的 occupied 障碍太少。
4. 3D top-down 点云仍然稀疏、散、结构不够像真实商场。
```

关键诊断：

```text
V3.1 的问题不是 free 不够，而是 occupied 障碍层过稀。
```

从 V3.1 质量报告看：

```text
occupied_ratio = 0.52% 全图
active_occupied_ratio = 2.11%
obstacle 点数 = 830
obs>=3 点数 = 2153
```

这说明：

```text
V3.1 为了删除 V2 的假墙，使用了较严格的多帧一致性过滤；
中央假墙确实消失了，但真实墙体/柜台/障碍也被过滤得太薄；
所以地图“通了”，但不像真实导航图。
```

下一阶段不能只追 `largest_free > 50%`，而要改成双目标：

```text
1. 保持主通道连续。
2. 让 occupied 障碍层清楚、连续、可解释。
```

---

## 1. 路线选择：先官方 ScaRF-SLAM，还是继续调 V3.1？

### 1.1 不建议只继续调 V3.1 参数

继续做：

```text
voxel 0.015 -> 0.025
obs>=3 -> obs>=2/3 hybrid
keyframe 140 -> 240
free close kernel
obstacle dilate kernel
```

有一定收益，但风险是：

```text
1. 可能只是把图调得更像地图，未必是真实几何。
2. obs 阈值一放松，V2 的假墙/反光噪声可能回来。
3. 仍然不知道问题来自 depth、scale、fusion，还是 occupancy 分层。
```

### 1.2 建议接官方 ScaRF-SLAM 做 V3.2 基线

ScaRF-SLAM 官方方法本身就是：

```text
classical visual SLAM poses
  + geometric foundation model depth
  + frame/submap scale optimization
  + projection-based point cloud fusion
  -> globally consistent dense reconstruction
```

官方 README 明确说明它支持 offline reconstruction：

```text
image folder + corresponding pose file
pose file 支持 TUM 格式:
timestamp tx ty tz qx qy qz qw
```

这正好能接本工程已有输入：

```text
resources/input_video.mp4 抽帧
output/route_A/trajectory_flat.txt
```

官方 ScaRF-SLAM 输出：

```text
recon/<trajectory>/pts_global*.pcd
recon/<trajectory>/pts_local*/
poses_*.csv / poses_*.txt
opt_graph*
```

然后我们再把 `pts_global*.pcd` 投影成：

```text
nav_binary_map.png
static_map_tricolor.png
topdown_3d_scene.png
BEV 视频
```

### 1.3 1811.10092v2 不应作为下一步建图算法

`1811.10092v2` 是：

```text
Reinforced Cross-Modal Matching and Self-Supervised Imitation Learning for Vision-Language Navigation
```

它研究的是 VLN：

```text
instruction
local visual scene
global trajectories in top-down view
cross-modal matching
self-supervised imitation learning
```

它的 Figure 1 是展示风格参考：

```text
已有 top-down scene / house map
  + initial position
  + target position
  + demonstration path
  + executed paths
```

它不是从单目视频重建地图的算法。

因此顺序应为：

```text
1. 先把 3D/2D 地图做好。
2. 再复现 1811 Figure 1 风格展示。
3. 最后若需要，扩展到 VLN / 语言导航。
```

---

## 2. 官方资料依据

ScaRF-SLAM 论文：

```text
arXiv:2606.00307v1
Title: ScaRF-SLAM: Scale-Consistent Reconstruction with Feed-Forward Models and Classical Visual SLAM
```

论文摘要要点：

```text
1. 直接用 GFM 几何预测做 tracking 会把几何误差传给位姿估计。
2. ScaRF-SLAM 解耦 tracking 和 mapping。
3. classical visual SLAM 负责鲁棒、低延迟 tracking。
4. GFM 只用于 mapping。
5. mapping 锚定在 SLAM 位姿上，并优化 depth scales。
6. 系统从多个 posed keyframes 构建 submaps。
7. 使用 frame/submap scale optimization 和 projection-based point cloud fusion。
```

官方 GitHub README 要点：

```text
1. ScaRF-SLAM 是 dense visual mapping framework。
2. 它把 classical visual SLAM 的鲁棒性和 GFM 的重建能力结合。
3. 它兼容 monocular、stereo、mono-inertial、multi-camera、fisheye 等配置。
4. Offline reconstruction 可以输入 image folder + pose file。
5. pose file 支持 TUM:
   timestamp tx ty tz qx qy qz qw
6. 如果图像是 rectified pinhole，需要更新 pinhole intrinsics/resolution，并移除 fisheye 配置。
7. 输出包含 recon/<trajectory>/pts_global*.pcd。
```

参考：

```text
论文: https://arxiv.org/abs/2606.00307v1
代码: https://github.com/ori-drs/ScaRF-SLAM
```

1811.10092v2 论文：

```text
Title: Reinforced Cross-Modal Matching and Self-Supervised Imitation Learning for Vision-Language Navigation
arXiv:1811.10092
```

参考：

```text
https://arxiv.org/abs/1811.10092
```

---

## 3. 下一阶段总体目标

本阶段命名：

```text
Route A V3.2
```

目标：

```text
1. 不再只看 V3.1 的 largest_free 指标。
2. 对 V3.1 做地图失败诊断，确认是深度、融合还是 occupancy 分层问题。
3. 复现官方 ScaRF-SLAM offline reconstruction，使用 DPVO TUM 作为外部 pose。
4. 用官方 ScaRF 输出的 pts_global*.pcd 生成新的二维导航栅格。
5. 与 V2 / V3.1 / 官方 ScaRF 结果做三方对比。
6. 如果官方 ScaRF 点云明显更好，再基于它制作 1811 Figure 1 风格 top-down 展示。
```

---

### 3.1 最终成品验收目标

用户最终想看到的不是单独的点云、单独的指标表或单独的轨迹图，而是接近论文 Figure 1 表达方式的导航场景总览：

```text
1. 一个清楚的俯视场景图：
   能看出商场通道、障碍、墙体、柜台/货架区域、可通行区域。

2. 一张可导航二维路线图：
   黑色/深色表示不可通行障碍，
   白色/浅色表示可通行区域，
   灰色/半透明表示未知区域，
   相机轨迹和方向箭头叠加在地图上。

3. 一个实时 BEV 视频：
   地图保持稳定，
   相机当前位置随时间更新，
   行人位置随时间更新，
   行人只显示在物理上合理的位置。

4. 一张 Figure 1 风格总览图：
   top-down scene/map
   + start/end markers
   + camera route
   + current/final people positions
   + optional local frame thumbnails。
```

因此，后续所有质量判断必须同时包含：

```text
1. 地图质量：
   障碍是否连续，可通行区域是否连通，unknown 是否合理。

2. 轨迹质量：
   相机轨迹方向、转向、尺度是否和真实世界一致。

3. 行人质量：
   行人是否只出现在相机附近、可观测范围内、可通行区域内。

4. 展示质量：
   最终图能否让人一眼看懂“我在哪里、我走过哪里、哪里能走、附近有哪些行人”。
```

---

### 3.2 动态行人物理过滤要求

当前动态行人层存在一个严重问题：

```text
部分行人点会出现在墙体里、障碍物里、未知区域深处，或者离相机过远。
```

这不符合物理规则，也会破坏最终 BEV 可视化。因此 V3.2 必须新增行人过滤与修正模块，不能再把检测到的所有行人点直接画到地图上。

新增模块：

```text
src/people_bev_tracker/people_bev_tracker/person_map_filter.py
```

新增脚本：

```text
src/people_bev_tracker/scripts/filter_people_tracks_on_map.py
```

输出：

```text
output/route_A_v3_2_scarf_official/people_filter/
├── people_tracks_raw.json
├── people_tracks_filtered.json
├── people_filter_report.md
├── people_filter_metrics.json
├── people_filter_debug_video.mp4
├── rejected_people_overlay.png
└── accepted_people_overlay.png
```

如果官方 ScaRF-SLAM 阶段还没有跑通，也必须先基于 V3.1 最佳地图输出：

```text
output/route_A_v3_scarf/people_filter/
```

每一个行人观测点必须经过以下过滤：

```text
1. 距离过滤：
   只显示相机附近的行人。
   默认只保留距离当前相机位置 0.5m 到 12m 的行人。
   如果当前尺度仍是相对尺度，则用已选 scale 转成近似米制。

2. 视野过滤：
   行人必须位于当前相机朝向前方的合理扇区内。
   默认水平 FOV 范围可以取相机内参估计值；
   如果不确定，则使用前向 +/- 70 度作为保守近似。

3. 可通行区域过滤：
   行人的 BEV 坐标所在 grid cell 必须是 free。
   如果落在 occupied 或 unknown，不能直接显示。

4. 障碍膨胀过滤：
   行人不能离 occupied 障碍过近。
   默认要求距离障碍至少 0.3m 到 0.5m。
   这可以避免行人贴墙、穿墙或进入柜台内部。

5. 最近可通行点修正：
   如果行人点落在墙体/障碍/unknown，但距离最近 free cell 很近，
   可以投影到最近 free cell。
   默认最大修正距离 0.8m。
   超过该距离则拒绝显示。

6. 时序稳定性过滤：
   同一个 track_id 至少连续出现 2 到 3 帧才显示。
   单帧误检、跳变点、瞬移点要拒绝。

7. 速度过滤：
   行人在 BEV 中的速度不能超过合理上限。
   默认人行速度上限 3.0m/s，宽松上限 5.0m/s。
   超过上限的点标记为 outlier，不显示或用上一帧预测位置。

8. 动态层和静态层分离：
   行人只能作为 overlay 显示，
   永远不能写入 static_map / occupancy grid。
```

每个行人观测都要保留过滤原因：

```text
accepted
rejected_too_far
rejected_too_close
rejected_out_of_fov
rejected_in_occupied
rejected_in_unknown
rejected_near_obstacle
corrected_to_nearest_free
rejected_no_near_free_cell
rejected_track_too_short
rejected_speed_outlier
rejected_low_confidence
```

保存到：

```text
people_tracks_filtered.json
```

每条记录至少包含：

```json
{
  "frame_index": 123,
  "track_id": 5,
  "raw_bev_xy": [1.2, -3.4],
  "filtered_bev_xy": [1.1, -3.1],
  "camera_distance_m": 4.8,
  "status": "corrected_to_nearest_free",
  "confidence": 0.76,
  "map_cell_type": "free",
  "nearest_obstacle_distance_m": 0.64
}
```

新增指标：

```text
people_raw_count
people_accepted_count
people_rejected_count
people_corrected_count
people_acceptance_ratio
people_in_occupied_ratio_before
people_in_occupied_ratio_after
people_in_unknown_ratio_before
people_in_unknown_ratio_after
people_near_obstacle_ratio_after
track_id_switch_suspect_count
people_speed_outlier_count
```

硬性目标：

```text
people_in_occupied_ratio_after = 0
people_in_unknown_ratio_after <= 5%
people_near_obstacle_ratio_after <= 5%
people_acceptance_ratio 不要求越高越好，宁可少显示，也不能把人画进墙里。
```

最终 BEV 视频中：

```text
1. accepted 行人正常显示。
2. corrected 行人用同样 ID 显示，但调试版视频可用虚线连接 raw -> corrected。
3. rejected 行人默认不显示。
4. 调试版视频可以用红色叉号显示 rejected，并标注拒绝原因。
5. 发布给用户看的最终版视频只显示 accepted/corrected 行人。
```

---

## 4. 阶段 A：V3.1 地图失败诊断

### 4.1 新增脚本

新增：

```text
src/people_bev_tracker/scripts/diagnose_route_A_v3_map.py
```

输入：

```text
--v3-dir output/route_A_v3_scarf
--v2-dir output/route_A_v2
--route-a-dir output/route_A
```

输出：

```text
output/route_A_v3_scarf/diagnostics/
├── diagnosis_report.md
├── depth_quality_summary.json
├── scale_quality_summary.json
├── dense_point_distribution.json
├── occupancy_layer_debug.png
├── floor_points_debug.png
├── obstacle_points_debug.png
├── obs2_points_topdown.png
├── obs3_points_topdown.png
├── obs2_vs_obs3_comparison.png
└── failure_regions.md
```

### 4.2 必查问题

Claude Code 必须回答：

```text
1. occupied 太少是因为 dense point cloud 缺墙，还是 occupancy 阈值太严？
2. obs=2 的 76274 点里，有多少其实是墙？
3. obs>=3 只有 2153 点，是否导致真实障碍被删掉？
4. floor_observation_ratio 只有 3.4%，是否说明 floor/ground 分类过严？
5. topdown_3d_scene 中的结构点是否已经足够，只是 2D 投影规则不合理？
6. Depth Anything V2 Metric Indoor 的深度是否在后半段发散？
7. scale=0.605 是否全局合理？是否局部段落需要分段 scale？
8. 行人 bbox mask 是否过大，误删了墙/柜台？
9. mirror_y 是否在 dense occupancy 和 BEV 渲染里被双重应用？
```

### 4.3 必做可视化

必须生成对比图：

```text
1. dense_global_static 全点 topdown。
2. floor 点 topdown。
3. obstacle 点 topdown。
4. obs=2 点 topdown。
5. obs>=3 点 topdown。
6. V3.1 当前 occupancy。
7. V3.1 如果使用 obs>=2 的 occupancy。
8. V3.1 如果使用 hybrid obs 策略的 occupancy。
```

### 4.4 诊断结论分类

最后必须把问题归类为下列之一或多个：

```text
A. 深度模型问题：
   单目深度本身无法恢复墙/玻璃/反光。

B. 尺度问题：
   全局 scale 不足，需要分段 scale 或 pairwise refine。

C. 融合问题：
   voxel / min_observations / keyframe overlap 太严格。

D. 占据分层问题：
   floor/obstacle height band 或 obs 阈值规则不合理。

E. 显示问题：
   真实点云还可以，但 2D render 没表达清楚。

F. 数据本身问题：
   视角没有看到某侧墙，纯视觉无法凭空恢复。
```

---

## 5. 阶段 B：V3.1 快速增强基线

在官方 ScaRF-SLAM 跑之前，先基于 V3.1 做一个轻量增强，用来判断是不是“规则过严”。

### 5.1 新增配置

新增：

```text
src/people_bev_tracker/config/route_A_v3_scarf_tune.yaml
```

参数搜索范围：

```yaml
v3_tune:
  voxel_size_unit_candidates: [0.015, 0.020, 0.025, 0.030]
  min_observations_obstacle_candidates: [2, 3]
  min_observations_floor_candidates: [1, 2]
  obstacle_height_range_candidates:
    - [0.08, 0.85]
    - [0.10, 1.00]
    - [0.12, 1.20]
  floor_height_abs_thresh_candidates: [0.08, 0.12, 0.16]
  obstacle_dilate_kernel_candidates: [3, 5, 7]
  obstacle_close_kernel_candidates: [7, 9, 13]
  free_close_kernel_candidates: [13, 17, 21]
  keep_obs2_if_near_obs3: [true, false]
```

### 5.2 新增脚本

新增：

```text
src/people_bev_tracker/scripts/tune_route_A_v3_occupancy.py
```

输入：

```text
output/route_A_v3_scarf/dense_global_static.npy
output/route_A_v3_scarf/dense_global_static_obs.npy
output/route_A/trajectory_flat.txt
output/route_A/ground_plane_final.json
output/route_A_v3_scarf/alignment_selected.json
```

输出：

```text
output/route_A_v3_scarf/v3_tune/
├── candidates/
├── tune_report.md
├── tune_report.json
└── best/
    ├── nav_binary_map.png
    ├── static_map_tricolor.png
    ├── paper_style_global_view.png
    ├── topdown_3d_scene.png
    ├── static_map.npy
    ├── static_map_meta.json
    └── quality.json
```

### 5.3 新质量指标

不要只看 largest_free。

新增：

```text
active_occupied_ratio_min: 0.04
obstacle_visibility_score
wall_continuity_score
free_continuity_score
trajectory_collision_ratio
dynamic_contamination_score
```

当前 V3.1 active_occupied_ratio = 2.11%，太少。目标：

```text
active_occupied_ratio >= 4% 或者障碍边界肉眼明显。
largest_free_component_ratio >= 45% 即可，不必死卡 50%。
trajectory_collision_ratio = 0。
```

---

## 6. 阶段 C：官方 ScaRF-SLAM V3.2 复现

### 6.1 安装策略

官方 ScaRF-SLAM 依赖：

```text
Python 3.11
Depth Anything 3
rosbags
open3d
gtsam
vismatch
可选 OV-SLAM
```

不要污染现有 `dpvo` 环境。建议新建：

```text
conda create -n scarf-slam python=3.11
conda activate scarf-slam
```

官方代码位置：

```text
project code/ScaRF-SLAM
project code/Depth-Anything-3
```

只允许调用，不允许修改官方源码。

所有适配脚本放在：

```text
src/people_bev_tracker/scripts/
src/people_bev_tracker/people_bev_tracker/scarf_official/
```

### 6.2 新增适配模块

新增：

```text
src/people_bev_tracker/people_bev_tracker/scarf_official/__init__.py
src/people_bev_tracker/people_bev_tracker/scarf_official/input_adapter.py
src/people_bev_tracker/people_bev_tracker/scarf_official/config_writer.py
src/people_bev_tracker/people_bev_tracker/scarf_official/output_adapter.py
```

新增脚本：

```text
src/people_bev_tracker/scripts/prepare_scarf_official_inputs.py
src/people_bev_tracker/scripts/run_scarf_official_mapping.py
src/people_bev_tracker/scripts/build_grid_from_scarf_official.py
src/people_bev_tracker/scripts/compare_v3_vs_scarf_official.py
```

### 6.3 输入适配

从视频抽帧：

```text
resources/input_video.mp4
```

生成：

```text
output/route_A_v3_2_scarf_official/scarf_input/images/
  image_<sec>_<nsec>.jpg
```

轨迹：

```text
output/route_A/trajectory_flat.txt
```

转换/同步为：

```text
output/route_A_v3_2_scarf_official/scarf_input/trajectory.txt
```

注意：

```text
1. 必须保持 image timestamp 和 TUM timestamp 可匹配。
2. DPVO stride=2，要明确 frame_index 与 timestamp 关系。
3. 如果抽关键帧而不是全帧，trajectory.txt 只写对应帧位姿。
```

### 6.4 配置适配

官方 README 提醒：

```text
如果图像是 rectified pinhole images，需要更新 pinhole_intrinsics 和 pinhole_resolution，并移除 fisheye sections。
```

因此生成：

```text
output/route_A_v3_2_scarf_official/scarf_config/mall_dpvo_offline.yaml
```

必须包含：

```yaml
use_slam: false
is_mono: true
pinhole_intrinsics: [fx, fy, cx, cy]
pinhole_resolution: [width, height]
trajectory: mall_dpvo
sec_skip: ...
kf_distance: ...
kf_angle_deg: ...
max_distance: ...
frame_scale_opt: true
submap_scale_opt: true
point_cloud_fusion: true
```

如果官方配置字段名和这里不同，以官方 README / config 文件为准，但报告里必须解释映射关系。

### 6.5 运行官方 ScaRF-SLAM

生成一个可执行脚本：

```text
output/route_A_v3_2_scarf_official/run_scarf_official.sh
```

内容类似：

```bash
cd "/home/ros/ros2_orbslam3/project code/ScaRF-SLAM"
conda activate scarf-slam
python3 run_mapping.py \
  --slam_folder /home/ros/ros2_orbslam3/output/route_A_v3_2_scarf_official/scarf_run \
  --image_folder /home/ros/ros2_orbslam3/output/route_A_v3_2_scarf_official/scarf_input/images \
  --poses /home/ros/ros2_orbslam3/output/route_A_v3_2_scarf_official/scarf_input/trajectory.txt \
  --config /home/ros/ros2_orbslam3/output/route_A_v3_2_scarf_official/scarf_config/mall_dpvo_offline.yaml
```

运行后查找：

```text
output/route_A_v3_2_scarf_official/scarf_run/recon/*/pts_global*.pcd
```

选择：

```text
点数最多 / 最新 / 配置匹配的 pts_global*.pcd
```

复制为：

```text
output/route_A_v3_2_scarf_official/scarf_dense_global.pcd
```

### 6.6 生成二维栅格

用官方 ScaRF 输出点云生成：

```text
output/route_A_v3_2_scarf_official/scarf_grid/
├── nav_binary_map.png
├── static_map_tricolor.png
├── paper_style_global_view.png
├── topdown_3d_scene.png
├── static_map.npy
├── static_map_meta.json
└── quality.json
```

必须使用：

```text
DPVO trajectory_flat
ground_plane_final
mirror_y alignment
同一套 quality metrics
```

这样才能与 V3.1 公平对比。

### 6.7 生成 BEV 视频

使用官方 ScaRF grid 重跑：

```text
offline_pipeline_A.py
```

输出：

```text
output/route_A_v3_2_scarf_official/bev_tracking_scarf_official.mp4
output/route_A_v3_2_scarf_official/final_frame_scarf_official.png
```

---

## 7. 阶段 D：三方对比

必须比较：

```text
V2: VGGT pointcloud map
V3.1: Depth Anything V2 lightweight ScaRF-inspired map
V3.2: official ScaRF-SLAM map
```

输出：

```text
output/route_A_v3_2_scarf_official/comparison/
├── comparison_report.md
├── v2_vs_v31_vs_scarf_metrics.csv
├── side_by_side_nav_binary.png
├── side_by_side_tricolor.png
├── side_by_side_topdown_3d.png
└── recommendation.md
```

对比指标：

```text
active_free_ratio
active_unknown_ratio
active_occupied_ratio
largest_free_component_ratio
trajectory_collision_ratio
obstacle_visibility_score
wall_continuity_score
central_false_wall_removed
back_half_wall_coverage_score
dynamic_contamination_score
people_in_occupied_ratio_after
people_in_unknown_ratio_after
people_near_obstacle_ratio_after
people_acceptance_ratio
```

人工视觉评价也必须写：

```text
1. 哪个图最像导航地图？
2. 哪个图障碍最清楚？
3. 哪个图 free/unknown 区分最好？
4. 哪个图最适合作为 1811 Figure 1 风格底图？
5. 哪个版本的行人显示最符合物理规则？
6. 是否还存在行人出现在墙体、障碍物或未知区域的问题？
```

---

## 8. 阶段 E：1811.10092v2 Figure 1 风格展示

只有当阶段 C/D 中至少有一个地图底图可用时才做。

不要把 1811 作为建图算法复现。

它在本项目里的作用是：

```text
top-down scene visualization style
```

新增脚本：

```text
src/people_bev_tracker/scripts/render_1811_style_topdown.py
```

输入：

```text
best map 或官方 ScaRF map
camera_trajectory_route_A_v3_dense.json
people_tracks_filtered.json
可选 target / route annotations
```

输出：

```text
output/route_A_1811_style/
├── figure1_style_global_view.png
├── figure1_style_global_view_with_people.png
├── figure1_style_report.md
└── optional_video.mp4
```

展示元素：

```text
1. top-down map / scene
2. initial position marker
3. final position marker
4. camera trajectory
5. dynamic people current/final positions
6. optional route A/B/C if later有多条轨迹
7. optional local visual scene thumbnails
```

注意：

```text
Figure 1 风格展示只能使用过滤后的行人轨迹。
不得把 raw people_tracks 直接画到最终图里。
如果 filtered people 数量很少，也要优先保证物理一致性。
```

---

## 9. 推荐执行顺序

Claude Code 按以下顺序执行：

```text
Step 1: V3.1 地图失败诊断
Step 2: V3.1 occupancy 快速增强搜索
Step 3: 官方 ScaRF-SLAM 环境准备和输入适配
Step 4: 官方 ScaRF-SLAM offline reconstruction
Step 5: 官方 ScaRF 点云 -> 2D grid
Step 6: 基于最佳地图过滤动态行人，只保留附近且物理合理的行人
Step 7: V2 / V3.1 / 官方 ScaRF 三方对比
Step 8: 选择最佳底图，使用过滤后的行人做 1811 Figure 1 风格展示
```

如果 Step 3/4 官方 ScaRF 环境失败：

```text
不要中断整个任务；
必须完成 Step 1/2；
写清楚失败原因和恢复命令。
```

---

## 10. 最终交付物

```text
output/route_A_v3_scarf/diagnostics/diagnosis_report.md
output/route_A_v3_scarf/v3_tune/tune_report.md
output/route_A_v3_scarf/v3_tune/best/nav_binary_map.png
output/route_A_v3_scarf/v3_tune/best/static_map_tricolor.png

output/route_A_v3_2_scarf_official/scarf_input/
output/route_A_v3_2_scarf_official/scarf_config/
output/route_A_v3_2_scarf_official/scarf_run/
output/route_A_v3_2_scarf_official/scarf_dense_global.pcd
output/route_A_v3_2_scarf_official/scarf_grid/nav_binary_map.png
output/route_A_v3_2_scarf_official/scarf_grid/static_map_tricolor.png
output/route_A_v3_2_scarf_official/scarf_grid/topdown_3d_scene.png
output/route_A_v3_2_scarf_official/bev_tracking_scarf_official.mp4

output/route_A_v3_2_scarf_official/people_filter/people_tracks_filtered.json
output/route_A_v3_2_scarf_official/people_filter/people_filter_report.md
output/route_A_v3_2_scarf_official/people_filter/people_filter_metrics.json
output/route_A_v3_2_scarf_official/people_filter/people_filter_debug_video.mp4

output/route_A_v3_2_scarf_official/comparison/comparison_report.md
output/route_A_v3_2_scarf_official/comparison/side_by_side_nav_binary.png
output/route_A_v3_2_scarf_official/comparison/side_by_side_tricolor.png
output/route_A_v3_2_scarf_official/comparison/recommendation.md

output/route_A_1811_style/figure1_style_global_view.png
output/route_A_1811_style/figure1_style_report.md
```

---

## 11. 不要做的事情

```text
1. 不要把 1811.10092v2 当成建图算法复现。
2. 不要只为了过 largest_free 50% 而把 unknown 强行改成 free。
3. 不要只调颜色来假装地图变清楚。
4. 不要把 DPVO 主轨迹换掉。
5. 不要修改 project code/DPVO、VGGT、KV-tracker 官方代码。
6. 如果 clone 官方 ScaRF-SLAM，也不要修改官方代码，只能调用。
7. 不要把行人融合进静态地图。
8. 不要只输出点云，不输出 2D grid 和 BEV 视频。
9. 不要把未过滤的 raw 行人点直接画进最终 BEV 图或 Figure 1 风格图。
10. 不要为了“显示更多行人”而允许行人出现在墙体、障碍物、柜台或未知区域深处。
```

---

## 12. 给 Claude Code 的一句话任务

```text
请先诊断 V3.1 地图为什么“数值变好但视觉仍不像导航图”，再接入官方 ScaRF-SLAM 做固定 DPVO 轨迹下的官方 dense reconstruction 基线；随后基于最佳地图对动态行人做物理过滤，只显示相机附近且位于可通行区域的行人，禁止把人画进墙体/障碍物/未知区域；最后把 V2 / V3.1 / 官方 ScaRF 三种地图做定量和视觉对比，只有选出可靠底图后，再用过滤后的行人轨迹做 1811.10092v2 Figure 1 风格的 top-down 展示。
```
