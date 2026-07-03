# Route A V3 执行方案：借鉴 ScaRF-SLAM 做稠密重建、黑白导航栅格和三维俯视地图

## 0. 给 Claude Code 的任务定位

本轮任务基于已经完成的 V2 结果继续优化。

V2 已经输出：

```text
output/route_A_v2/best/nav_binary_map.png
output/route_A_v2/best/static_map_tricolor.png
output/route_A_v2/best/paper_style_global_view.png
output/route_A_v2/bev_tracking_route_A_v2.mp4
output/route_A_v2/route_A_v2_optimization_report.md
```

但是当前二维地图仍然不合格：

```text
1. 黑白栅格图没有完整展示商场通行空间。
2. 中央区域存在疑似 VGGT 假墙/错误障碍，导致 free space 被切断。
3. largest_free_component_ratio = 35.12%，没有达到 V2 设定的 >= 50%。
4. V2 的白色 free 很大程度来自 corridor / frustum / semantic 推断，不是稳定稠密几何重建。
5. 当前 BEV 方向与真实世界方向不一致，路线像从地面下方看的“仰视图”，转向方向和真实场景相反。
```

本轮目标：

```text
第一目标:
  生成更可靠的黑白导航栅格图:
    黑色 = 不可通行 / 障碍 / 未知
    白色 = 可通行 free space

第二目标:
  生成更合理的三值 occupancy grid:
    黑色 = occupied
    白色 = free
    灰色 = unknown

第三目标:
  为后续类似 1811.10092v2 Figure 1 的三维场景俯视图做准备:
    3D dense reconstruction
    top-down projection
    camera trajectory
    dynamic people overlay
```

本轮不要推翻已有系统：

```text
DPVO 仍然是主相机轨迹。
YOLO/BoT-SORT 仍然是动态行人来源。
people_bev_tracker 仍然是 BEV 显示和行人投影主工程。
ScaRF-SLAM 只用于借鉴“可靠位姿 + 稠密映射 + 尺度一致性 + 子图融合”的思想。
```

特别注意：

```text
在做任何 ScaRF-style 稠密建图之前，必须先修正 BEV 坐标系方向。
如果二维地图是镜像/仰视图，后续 dense map、行人位置、导航栅格都会继承错误方向。
```

---

## 1. 必读文件

Claude Code 执行前按顺序读取：

```text
src/people_bev_tracker/docs/00_文档索引_阅读顺序.md
src/people_bev_tracker/docs/06_RouteA_V3_ScaRF_SLAM稠密重建与导航栅格执行方案_最新版.md
output/route_A_v2/route_A_v2_optimization_report.md
output/route_A_v2/tune_report.md
output/route_A_v2/best/quality.json
src/people_bev_tracker/docs/05_RouteA_V2静态地图优化执行方案_最新版.md
src/people_bev_tracker/docs/04_RouteA_V1执行总结与代码说明.md
```

然后读取核心代码：

```text
src/people_bev_tracker/config/route_A_v2.yaml
src/people_bev_tracker/people_bev_tracker/static_map.py
src/people_bev_tracker/people_bev_tracker/free_space.py
src/people_bev_tracker/people_bev_tracker/map_quality.py
src/people_bev_tracker/scripts/tune_static_map_v2.py
src/people_bev_tracker/scripts/offline_pipeline_A.py
```

---

## 1.5 前置必修：BEV 坐标系方向校准

### 1.5.1 当前问题描述

用户观察到：

```text
当前相机轨迹和真实世界路线不一致。
当前 BEV 更像是从地面下方向上看的“仰视图”。
转向方向和真实世界正好反过来。
```

这通常不是 DPVO 轨迹本身完全错误，而是 BEV 显示坐标存在镜像问题。

当前代码里，V2 的地图和轨迹大致使用：

```text
world point
  -> R_align 对齐地面法向
  -> select bev_axes ["x", "z"]
  -> px = W/2 + (x - origin_x) / resolution
  -> py = H/2 - (z - origin_z) / resolution
```

这个约定是数学上常见的俯视显示，但它不保证与真实商场地图方向一致。尤其是单目 SLAM 世界系没有真实 north/east 语义，`R_align` 只对齐了地面法向，没有确定“从上往下看”时的水平朝向和手性。

如果出现：

```text
真实世界左转 -> BEV 中像右转
真实世界右转 -> BEV 中像左转
```

说明大概率需要做 mirror / handedness correction，而不只是旋转 90° 或 180°。

### 1.5.2 必须新增统一的 BEV 方向变换

不要只在最终 PNG 上 `cv2.flip`。必须新增一个统一的 BEV 坐标变换，作用在：

```text
1. static_map grid
2. camera trajectory
3. camera heading arrow
4. people bev positions
5. free_space masks
6. obstacle masks
7. ScaRF-style dense point cloud top-down projection
8. final BEV video
```

新增配置：

```yaml
bev_alignment:
  enabled: true

  # manual | auto_candidates
  mode: "manual"

  # 初始建议先试 mirror_x 或 mirror_y。
  # 如果“左转变右转”，优先试 mirror_x / mirror_y；
  # 如果只是整张图倒着，才试 rotate_180。
  transform: "mirror_x"

  # 可选值:
  # identity
  # mirror_x
  # mirror_y
  # rotate_180
  # swap_xy
  # swap_xy_mirror_x
  # swap_xy_mirror_y
  # rotate_90_cw
  # rotate_90_ccw

  # 若有真实楼层图或用户标注的方向，可后续扩展成 similarity_2d。
  rotation_deg: 0.0
  scale: 1.0
  translation: [0.0, 0.0]

  # 调试输出
  save_candidates: true
  selected_by_user: true
```

### 1.5.3 新增模块

新增：

```text
src/people_bev_tracker/people_bev_tracker/bev_alignment.py
```

实现：

```python
def apply_bev_alignment_xy(xy: np.ndarray, cfg: dict) -> np.ndarray:
    """
    输入:
      xy: (..., 2), aligned BEV 坐标，不是像素坐标

    输出:
      xy_aligned: (..., 2)

    支持:
      identity
      mirror_x:          [-x,  y]
      mirror_y:          [ x, -y]
      rotate_180:        [-x, -y]
      swap_xy:           [ y,  x]
      swap_xy_mirror_x:  [-y,  x]
      swap_xy_mirror_y:  [ y, -x]
      rotate_90_cw:      [ y, -x]
      rotate_90_ccw:     [-y, x]
    """

def apply_bev_alignment_heading(vec_xy: np.ndarray, cfg: dict) -> np.ndarray:
    """
    heading 向量只应用旋转/镜像，不应用 translation。
    """

def transform_grid_image(grid: np.ndarray, transform: str) -> np.ndarray:
    """
    用于已经 rasterized 的 grid debug 图。
    注意最终推荐仍然在坐标层做变换，然后重新 rasterize。
    """

def generate_alignment_candidates(static_map, camera_json_or_traj, out_dir):
    """
    输出 8 种候选方向图，方便用户肉眼选哪一个和真实世界一致。
    """
```

### 1.5.4 新增方向校准脚本

新增：

```text
src/people_bev_tracker/scripts/calibrate_bev_alignment.py
```

输入：

```text
--static-map output/route_A_v2/best/static_map.npy
--static-map-meta output/route_A_v2/best/static_map_meta.json
--camera-json output/route_A_v2/camera_trajectory_route_A_v2.json
--people-json output/route_A_v2/people_tracks_route_A_v2.json
--output-dir output/route_A_v3_scarf/alignment_candidates
```

输出：

```text
output/route_A_v3_scarf/alignment_candidates/
├── identity.png
├── mirror_x.png
├── mirror_y.png
├── rotate_180.png
├── swap_xy.png
├── swap_xy_mirror_x.png
├── swap_xy_mirror_y.png
├── rotate_90_cw.png
├── rotate_90_ccw.png
└── alignment_report.md
```

每张候选图必须叠加：

```text
1. static_map_tricolor 背景
2. camera trajectory
3. 起点和终点
4. 每隔 N 帧的 heading arrow
5. 可选 people final positions
6. 图上写清 transform 名称
```

用户根据真实商场路线选择正确 transform。若用户已经明确说“现在像仰视图，转向正好反过来”，Claude Code 应优先尝试：

```text
mirror_x
mirror_y
swap_xy_mirror_x
swap_xy_mirror_y
```

不要默认只用 `rotate_180`，因为 180° 旋转通常不会改变轨迹的左右转手性，而镜像才会让左转/右转互换。

### 1.5.5 接入 V2 / V3 pipeline

修改：

```text
src/people_bev_tracker/scripts/offline_pipeline_A.py
src/people_bev_tracker/scripts/tune_static_map_v2.py
V3 新增的 build_route_A_v3_scarf_like.py
V3 新增的 occupancy_from_dense.py
```

要求：

```text
1. 在 select_bev_axes 后立即调用 apply_bev_alignment_xy。
2. camera heading 调用 apply_bev_alignment_heading。
3. static_map rasterization 阶段也使用同一个 transform。
4. output json 里保存 alignment config。
5. static_map_meta.json 里新增:
   "bev_alignment": {...}
6. final report 明确写:
   selected_transform = mirror_x / mirror_y / ...
```

具体位置：

```text
cam_xyz_a = R_align @ cam_xyz
cam_bev_raw = select_bev_axes(cam_xyz_a, bev_axes_cfg)
cam_bev = apply_bev_alignment_xy(cam_bev_raw, align_cfg)

forward_w_a = R_align @ forward_w
heading_raw = select_bev_axes(forward_w_a, bev_axes_cfg)
heading = apply_bev_alignment_heading(heading_raw, align_cfg)

Xw_a = R_align @ Xw
person_bev_raw = select_bev_axes(Xw_a, bev_axes_cfg)
person_bev = apply_bev_alignment_xy(person_bev_raw, align_cfg)
```

### 1.5.6 验收标准

本步骤完成后，必须输出：

```text
output/route_A_v3_scarf/alignment_candidates/alignment_report.md
output/route_A_v3_scarf/alignment_candidates/<selected_transform>.png
output/route_A_v3_scarf/alignment_selected.json
```

然后使用选中的方向重新生成：

```text
output/route_A_v3_scarf/aligned_preview/nav_binary_map.png
output/route_A_v3_scarf/aligned_preview/static_map_tricolor.png
output/route_A_v3_scarf/aligned_preview/paper_style_global_view.png
output/route_A_v3_scarf/aligned_preview/final_frame_alignment_preview.png
```

验收：

```text
1. 真实世界左转，在 BEV 中也表现为左转。
2. 真实世界右转，在 BEV 中也表现为右转。
3. 相机 heading arrow 与轨迹前进方向一致。
4. 行人位置仍然落在同一地图坐标系内。
5. 黑白栅格、三值图、paper-style 图方向完全一致。
```

只有完成坐标方向修正后，才能进入后面的 ScaRF-style dense reconstruction。

---

## 2. ScaRF-SLAM 能借鉴什么，不能直接解决什么

### 2.1 论文和代码核心事实

ScaRF-SLAM 论文：

```text
arXiv:2606.00307v1
Title: ScaRF-SLAM: Scale-Consistent Reconstruction with Feed-Forward Models and Classical Visual SLAM
Submitted: 2026-05-29
```

官方摘要和 README 的关键思想：

```text
1. 它把 tracking 和 mapping 解耦。
2. classical visual SLAM 负责鲁棒、低延迟位姿。
3. geometric foundation models / feed-forward depth 只用于 mapping。
4. dense mapping 锚定在 SLAM 位姿上。
5. 通过 frame scale optimization 和 submap scale optimization 约束深度尺度一致性。
6. 通过 projection-based point cloud fusion 在子图内融合点云。
7. 当 SLAM trajectory 更新时，submap 也可以在线更新。
```

这和我们当前工程高度匹配：

```text
ScaRF-SLAM 的 classical SLAM 位姿
  对应我们这里的 DPVO trajectory_flat.txt

ScaRF-SLAM 的 GFM / Depth Anything 3 深度
  对应我们需要替代 VGGT 点云的 dense depth keyframes

ScaRF-SLAM 的 submap fusion
  对应我们需要解决的“中央假墙”和“free space 被切断”问题
```

官方 ScaRF-SLAM 支持 offline reconstruction：

```text
输入 image folder + pose file
pose file 支持 TUM:
  timestamp tx ty tz qx qy qz qw
```

这意味着它理论上可以直接接：

```text
resources/input_video.mp4 抽帧
output/route_A/trajectory_flat.txt
```

官方 README 也说明它的输出包含：

```text
recon/<trajectory>/pts_global*.pcd
recon/<trajectory>/pts_local*/
poses_*.csv / poses_*.txt
opt_graph*
```

这些输出可以作为我们新的静态点云源，再投影成 2D occupancy grid。

参考链接：

```text
论文: https://arxiv.org/abs/2606.00307v1
代码: https://github.com/ori-drs/ScaRF-SLAM
README: https://raw.githubusercontent.com/ori-drs/ScaRF-SLAM/main/README.md
```

### 2.2 不能误解 ScaRF-SLAM

ScaRF-SLAM 不是直接输出“黑白二维导航栅格图”的工具。

它主要输出：

```text
3D dense reconstruction point cloud
keyframe/submap 点云
scale-consistent reconstruction
trajectory-related files
```

所以本工程要做的是：

```text
ScaRF/ScaRF-inspired dense reconstruction
  -> 3D dense static point cloud
  -> ground plane / floor segmentation
  -> 2D occupancy grid
  -> nav_binary_map / tricolor_map
  -> BEV camera trajectory + dynamic people overlay
```

也就是说：

```text
ScaRF-SLAM 解决“稠密、尺度一致、比 VGGT 更稳定的三维结构”。
people_bev_tracker 继续解决“二维导航栅格和动态行人显示”。
```

---

## 3. 本轮推荐路线

不要一开始就完全改造成官方 ScaRF-SLAM。建议分两层执行。

### V3.1 必做：ScaRF-inspired 轻量稠密建图

直接在当前工程里实现：

```text
DPVO fixed poses
  + keyframe selection
  + depth model prediction
  + per-frame depth scale alignment
  + submap scale smoothing
  + dynamic person masking
  + projection-based point cloud fusion
  + 2D occupancy projection
```

优点：

```text
1. 不需要先跑通官方 ScaRF-SLAM 全环境。
2. 不会修改 project code 下的官方库。
3. 可以直接修复当前 V2 的核心问题：VGGT 假墙、点云不连续、free 连通域断裂。
4. 输出仍然接入现有 offline_pipeline_A.py。
```

### V3.2 可选：官方 ScaRF-SLAM 接入

如果本地环境允许，再把官方代码拉到：

```text
project code/ScaRF-SLAM
```

注意：

```text
官方代码只允许调用，不要修改。
所有适配脚本写在 src/people_bev_tracker/scripts/ 或 src/people_bev_tracker/people_bev_tracker/。
```

官方 ScaRF-SLAM 需要 Depth Anything 3、`rosbags`、`open3d`、`gtsam`、`vismatch` 等依赖。若当前 `dpvo` 环境不兼容，则新建独立环境：

```text
scarf-slam
```

但不要重复下载已有模型和已有环境。

---

## 4. V3.1 详细实现：ScaRF-inspired 轻量稠密建图

### 4.1 新增配置

新增：

```text
src/people_bev_tracker/config/route_A_v3_scarf.yaml
```

建议内容：

```yaml
input:
  video: "resources/input_video.mp4"
  pose_tum: "output/route_A/trajectory_flat.txt"
  calib: "config/KannalaBrandt8_1280x720.yaml"
  person_tracks: "output/route_A_v2/people_tracks_route_A_v2.json"
  semantic_mask_video: "resources/input_video.mp4_bev.mp4"

output:
  dir: "output/route_A_v3_scarf"

keyframes:
  stride_frames: 15
  min_translation_unit: 0.06
  min_rotation_deg: 8.0
  max_keyframes: 240
  image_width: 960
  image_height: 540

depth:
  backend_priority: ["depth_anything_3", "depth_anything_v2", "zoedepth", "vggt_fallback"]
  cache_dir: "output/route_A_v3_scarf/depth_cache"
  max_depth_unit: 5.0
  min_depth_unit: 0.03
  confidence_min: 0.35
  use_person_mask: true
  use_semantic_floor_mask: true

scale:
  mode: "ground_height_and_overlap"
  camera_height_unit_from_v1: 0.7909
  per_frame_floor_percentile: 50
  scale_smooth_window: 7
  max_scale_jump_ratio: 1.25
  enable_pairwise_overlap_refine: true
  overlap_refine_stride: 1

submap:
  keyframes_per_submap: 12
  overlap_keyframes: 3
  voxel_size_unit: 0.015
  max_points_per_submap: 300000
  fusion_mode: "projection_consistency"
  min_observations: 2
  depth_consistency_thresh_unit: 0.06

occupancy:
  resolution_unit_per_px: 0.006
  width_px: 1200
  height_px: 1200
  auto_origin_from_v2: true
  obstacle_height_range_unit: [0.05, 0.85]
  floor_height_abs_thresh_unit: 0.035
  free_from_floor_points: true
  free_from_ray_carving: true
  unknown_as_obstacle_in_binary: true
  obstacle_min_observations: 2
  obstacle_close_kernel: 9
  obstacle_dilate_kernel: 3
  free_close_kernel: 17
  free_dilate_kernel: 3

render:
  save_3d_pointcloud: true
  save_topdown_3d_view: true
  save_nav_binary: true
  save_tricolor: true
  save_paper_style: true
```

### 4.2 新增模块

新增目录：

```text
src/people_bev_tracker/people_bev_tracker/scarf_like/
```

新增文件：

```text
src/people_bev_tracker/people_bev_tracker/scarf_like/__init__.py
src/people_bev_tracker/people_bev_tracker/scarf_like/keyframes.py
src/people_bev_tracker/people_bev_tracker/scarf_like/depth_backend.py
src/people_bev_tracker/people_bev_tracker/scarf_like/dynamic_mask.py
src/people_bev_tracker/people_bev_tracker/scarf_like/scale_alignment.py
src/people_bev_tracker/people_bev_tracker/scarf_like/submap_fusion.py
src/people_bev_tracker/people_bev_tracker/scarf_like/occupancy_from_dense.py
src/people_bev_tracker/people_bev_tracker/scarf_like/render_3d_topdown.py
```

新增脚本：

```text
src/people_bev_tracker/scripts/build_route_A_v3_scarf_like.py
src/people_bev_tracker/scripts/run_route_A_v3_pipeline.py
src/people_bev_tracker/scripts/inspect_route_A_v3_outputs.py
```

---

## 5. 核心算法设计

### 5.1 Keyframe selection

输入：

```text
resources/input_video.mp4
output/route_A/trajectory_flat.txt
```

选择关键帧：

```text
1. 每 stride_frames=15 帧候选一次。
2. 若相对上一个关键帧平移 > min_translation_unit，则保留。
3. 若相对旋转 > min_rotation_deg，则保留。
4. 转弯处强制保留更多关键帧。
5. 行人密集遮挡帧降低优先级。
```

输出：

```text
output/route_A_v3_scarf/keyframes/
├── images/
├── keyframes.json
└── keyframe_poses_tum.txt
```

### 5.2 Depth backend

优先级：

```text
1. Depth Anything 3，如果本地已有或官方 ScaRF-SLAM 环境能用。
2. Depth Anything V2，如果本地已有。
3. ZoeDepth，如果本地已有。
4. VGGT fallback，只作为兜底，不作为最终首选。
```

Claude Code 必须先检查本地，不要盲目下载：

```bash
find /home/ros/ros2_orbslam3 -maxdepth 5 \
  -iname "*Depth-Anything-3*" -o \
  -iname "*Depth-Anything-V2*" -o \
  -iname "*depth_anything*" -o \
  -iname "*zoedepth*"
```

如果没有深度模型：

```text
1. V3.1 不能伪装完成。
2. 报告中写明 depth backend missing。
3. 只输出 ScaRF-SLAM 官方接入准备和需要安装的依赖。
```

如果需要下载官方 ScaRF-SLAM 或 Depth Anything 3：

```text
1. ScaRF-SLAM 官方库放到 project code/ScaRF-SLAM。
2. Depth Anything 3 官方库放到 project code/Depth-Anything-3。
3. 不要修改官方代码。
4. 所有 wrapper 写在 src/people_bev_tracker/。
```

### 5.3 Dynamic masking

当前 V2 已经有：

```text
output/route_A_v2/people_tracks_route_A_v2.json
```

构建 dense map 前必须剔除动态行人：

```text
1. 从 people_tracks json 读取每帧 bbox / mask 信息。
2. 若有 mask，用 mask 膨胀 5-15 px 后置 invalid。
3. 若只有 bbox，用 bbox 下半部分 + 全 bbox 膨胀置 invalid。
4. 同时剔除低置信 track 和运动中的人体边缘。
```

目的：

```text
不要把行人融合进静态地图。
不要让行人产生“假墙”。
```

### 5.4 Per-frame scale alignment

这是借鉴 ScaRF-SLAM 的核心：不要直接相信单帧深度尺度。

每个关键帧有：

```text
深度模型输出 D_i(u, v)
DPVO 位姿 T_wc_i
地面平面 pi: n^T X + d = 0
V1 相机高度 camera_h_median ≈ 0.79 DPVO unit
```

用地面像素估计单帧尺度：

```text
1. 从 SAM mask 或图像下半部分提 floor candidates。
2. 对 floor pixels 反投影得到 X_i(s) = s * D_i(u,v) * K^-1 [u,v,1]。
3. 转到世界系。
4. 求这些点到地面平面的高度分布。
5. 找尺度 s_i，使 floor points 的高度中位数接近 0。
```

再做约束：

```text
1. s_i 不能突变，限制 max_scale_jump_ratio。
2. 对 s_i 做滑动窗口中值滤波。
3. 如果当前帧 floor mask 太少，使用邻近关键帧尺度。
```

输出：

```text
output/route_A_v3_scarf/depth_scales.json
```

### 5.5 Pairwise overlap scale refine

相邻关键帧之间有重叠区域时，进一步优化尺度。

思路：

```text
1. 把关键帧 i 的点云投影到关键帧 j。
2. 在重叠像素上比较 depth_i_to_j 和 depth_j。
3. 求一个小的 scale correction，使重叠深度残差最小。
4. 对一个 submap 内的多个关键帧做轻量 scale smoothing。
```

目标：

```text
减少单帧深度尺度漂移。
避免多个 keyframe 融合后墙面重影、假墙、断层。
```

### 5.6 Submap fusion

借鉴 ScaRF-SLAM 的 submap 思路，不要一次性把所有点硬堆到全局。

流程：

```text
1. 每 12 个关键帧组成一个 submap。
2. 相邻 submap overlap 3 个关键帧。
3. 每个 submap 内先做 voxel downsample。
4. 使用 projection consistency 过滤不一致点：
   - 同一个空间 voxel 至少被 2 个 keyframe 观测到。
   - 深度残差超过 threshold 的点丢弃。
   - 动态 mask 区域不参与融合。
5. submap 输出局部点云、局部 floor points、局部 obstacle points。
6. 全局融合时保留 submap confidence。
```

输出：

```text
output/route_A_v3_scarf/submaps/
├── submap_000.ply
├── submap_000_meta.json
├── submap_001.ply
└── ...

output/route_A_v3_scarf/dense_global_static.ply
output/route_A_v3_scarf/dense_global_static.npy
```

### 5.7 2D occupancy from dense reconstruction

这一步是把 ScaRF-style dense reconstruction 变成你真正需要的黑白栅格。

输入：

```text
dense_global_static.npy / .ply
floor_points.npy
obstacle_points.npy
trajectory_flat.txt
ground_plane_final.json
V2 best meta for origin / R_align
```

生成三层：

#### A. free layer

来源：

```text
1. floor points 投影到 BEV。
2. camera-to-floor ray carving。
3. trajectory corridor。
4. V2 semantic free 作为弱先验。
```

规则：

```text
floor 被多个 keyframe 观测到 -> high confidence free
trajectory 走过 -> always free
单帧 floor -> low confidence free
```

#### B. occupied layer

来源：

```text
1. 高于地面一定范围的 dense static points。
2. 多帧一致观测的竖直结构。
3. V2 obstacle 作为弱先验。
```

规则：

```text
必须满足 min_observations >= 2。
如果 obstacle 与 trajectory corridor 冲突，优先相信 trajectory/free。
如果 obstacle 只来自单个关键帧且附近 floor 被多次观测，降权或删除。
```

#### C. unknown layer

来源：

```text
没有 floor observation
没有 obstacle observation
不在相机视锥 ray carving 内
```

二值导航图中：

```text
unknown -> black
```

因为导航要保守。

输出：

```text
output/route_A_v3_scarf/best/nav_binary_map.png
output/route_A_v3_scarf/best/static_map_tricolor.png
output/route_A_v3_scarf/best/static_map.npy
output/route_A_v3_scarf/best/static_map_meta.json
```

### 5.8 3D top-down view

为了靠近 1811.10092v2 Figure 1 的展示风格，新增一个三维场景俯视图渲染。

输出：

```text
output/route_A_v3_scarf/best/topdown_3d_scene.png
output/route_A_v3_scarf/best/topdown_3d_scene_with_tracks.png
output/route_A_v3_scarf/bev_tracking_route_A_v3.mp4
```

渲染内容：

```text
1. dense_global_static.ply 的 top-down projection。
2. floor 用浅色。
3. obstacles/walls 用深色。
4. DPVO 相机轨迹用蓝色或橙色。
5. 行人轨迹/当前位置用高对比点。
```

注意：

```text
这个图是展示图，不是导航代价地图。
导航仍以 nav_binary_map.png 和 static_map.npy 为准。
```

---

## 6. V3.2 官方 ScaRF-SLAM 接入方案

这部分是可选增强。只有 V3.1 或环境检查通过后才执行。

### 6.1 安装/下载约束

检查是否已存在：

```bash
find "/home/ros/ros2_orbslam3/project code" -maxdepth 2 -type d -iname "*ScaRF*"
find "/home/ros/ros2_orbslam3/project code" -maxdepth 2 -type d -iname "*Depth-Anything-3*"
```

如果不存在，并且允许联网：

```bash
cd "/home/ros/ros2_orbslam3/project code"
git clone https://github.com/ori-drs/ScaRF-SLAM.git
git clone https://github.com/ByteDance-Seed/Depth-Anything-3.git
```

不要修改官方代码。

### 6.2 数据适配

ScaRF-SLAM offline mode 支持：

```text
image_folder
poses in TUM format
config yaml
```

因此新增：

```text
src/people_bev_tracker/scripts/prepare_scarf_slam_inputs.py
```

功能：

```text
1. 从 resources/input_video.mp4 抽帧。
2. 文件名按 ScaRF-SLAM 需要的 timestamp 格式保存。
3. 根据 output/route_A/trajectory_flat.txt 写 poses TUM。
4. 生成 ScaRF-SLAM config:
   output/route_A_v3_scarf/scarf_config/mall_dpvo_offline.yaml
```

输出：

```text
output/route_A_v3_scarf/scarf_input/
├── images/
├── trajectory_tum.txt
└── mall_dpvo_offline.yaml
```

### 6.3 配置注意事项

当前视频是 pinhole / KannalaBrandt 标定转换后的普通视频流，配置里要重点检查：

```text
pinhole_intrinsics
pinhole_resolution
camera_model
distortion_model
max_distance
sec_skip
kf_distance
kf_angle_deg
frame_scale_opt
submap_scale_opt
point_cloud_fusion
```

从官方 README 可知：

```text
frame_scale_opt: true
submap_scale_opt: true
point_cloud_fusion: true
```

应保持开启。

### 6.4 运行官方 ScaRF-SLAM

示例命令写入脚本，不要直接硬编码在文档里执行：

```bash
cd "/home/ros/ros2_orbslam3/project code/ScaRF-SLAM"
python3 run_mapping.py \
  --slam_folder /home/ros/ros2_orbslam3/output/route_A_v3_scarf/scarf_run \
  --image_folder /home/ros/ros2_orbslam3/output/route_A_v3_scarf/scarf_input/images \
  --poses /home/ros/ros2_orbslam3/output/route_A_v3_scarf/scarf_input/trajectory_tum.txt \
  --config /home/ros/ros2_orbslam3/output/route_A_v3_scarf/scarf_input/mall_dpvo_offline.yaml
```

运行完成后查找：

```text
output/route_A_v3_scarf/scarf_run/recon/*/pts_global*.pcd
```

如果有多个，选择最新或点数最多的作为：

```text
scarf_dense_global.pcd
```

### 6.5 接入 people_bev_tracker

新增：

```text
src/people_bev_tracker/scripts/build_grid_from_scarf_pcd.py
```

输入：

```text
--pcd output/route_A_v3_scarf/scarf_dense_global.pcd
--pose output/route_A/trajectory_flat.txt
--ground-plane output/route_A/ground_plane_final.json
--v2-meta output/route_A_v2/best/static_map_meta.json
```

输出：

```text
output/route_A_v3_scarf/scarf_grid/nav_binary_map.png
output/route_A_v3_scarf/scarf_grid/static_map_tricolor.png
output/route_A_v3_scarf/scarf_grid/static_map.npy
output/route_A_v3_scarf/scarf_grid/static_map_meta.json
```

然后复用：

```text
offline_pipeline_A.py
```

生成：

```text
output/route_A_v3_scarf/bev_tracking_route_A_v3.mp4
```

---

## 7. 与 V2 的对比和验收

必须把 V3 和 V2 进行定量对比。

读取 V2：

```text
output/route_A_v2/best/quality.json
```

V2 基准：

```text
active_free_ratio = 37.24%
active_unknown_ratio = 49.51%
trajectory_collision_ratio = 0.00%
obstacle_small_component_ratio = 0.00%
largest_free_component_ratio = 35.12%
```

V3 目标：

```text
largest_free_component_ratio >= 50%
trajectory_collision_ratio <= 1%
active_free_ratio >= 30%
active_unknown_ratio <= 50%
central_false_wall_removed = true
```

新增指标：

```text
central_blockage_score:
  衡量当前 V2 中央黑色假墙是否仍然切断主通道。

floor_observation_ratio:
  active area 内由多关键帧 floor points 支持的 free 占比。

obstacle_observation_consistency:
  occupied cell 中满足 min_observations>=2 的比例。

dynamic_contamination_score:
  与 people tracks 高重叠的 occupied cell 比例，越低越好。
```

输出：

```text
output/route_A_v3_scarf/route_A_v3_scarf_execution_report.md
```

报告必须包含：

```text
1. V2 问题复盘。
2. 是否使用官方 ScaRF-SLAM。
3. 是否使用 Depth Anything 3 / V2 / ZoeDepth。
4. 关键帧数量。
5. 深度尺度估计曲线。
6. submap 数量和点数。
7. V2 vs V3 质量指标表。
8. nav_binary_map、tricolor_map、topdown_3d_scene 输出路径。
9. 仍然失败的区域和原因。
```

---

## 8. 输出文件要求

V3 最低必须输出：

```text
output/route_A_v3_scarf/
├── route_A_v3_scarf_execution_report.md
├── keyframes/
│   ├── keyframes.json
│   └── images/
├── depth_scales.json
├── dense_global_static.ply 或 dense_global_static.npy
├── submaps/
├── best/
│   ├── nav_binary_map.png
│   ├── static_map_tricolor.png
│   ├── paper_style_global_view.png
│   ├── topdown_3d_scene.png
│   ├── static_map.npy
│   ├── static_map_meta.json
│   └── quality.json
├── bev_tracking_route_A_v3.mp4
├── people_tracks_route_A_v3.json
└── camera_trajectory_route_A_v3.json
```

如果官方 ScaRF-SLAM 被成功接入，额外输出：

```text
output/route_A_v3_scarf/scarf_input/
output/route_A_v3_scarf/scarf_run/
output/route_A_v3_scarf/scarf_grid/
```

---

## 9. 执行命令模板

### 9.1 ScaRF-inspired 轻量路线

```bash
cd /home/ros/ros2_orbslam3
conda activate dpvo

python src/people_bev_tracker/scripts/build_route_A_v3_scarf_like.py \
  --config src/people_bev_tracker/config/route_A_v3_scarf.yaml \
  --output-dir output/route_A_v3_scarf
```

然后：

```bash
python src/people_bev_tracker/scripts/run_route_A_v3_pipeline.py \
  --config src/people_bev_tracker/config/route_A_v3_scarf.yaml \
  --static-map output/route_A_v3_scarf/best/static_map.npy \
  --static-map-meta output/route_A_v3_scarf/best/static_map_meta.json \
  --output-dir output/route_A_v3_scarf
```

### 9.2 官方 ScaRF-SLAM 可选路线

```bash
python src/people_bev_tracker/scripts/prepare_scarf_slam_inputs.py \
  --config src/people_bev_tracker/config/route_A_v3_scarf.yaml \
  --output-dir output/route_A_v3_scarf/scarf_input
```

再根据生成的 `run_scarf_slam_command.sh` 执行官方 ScaRF-SLAM。

之后：

```bash
python src/people_bev_tracker/scripts/build_grid_from_scarf_pcd.py \
  --pcd output/route_A_v3_scarf/scarf_dense_global.pcd \
  --pose output/route_A/trajectory_flat.txt \
  --ground-plane output/route_A/ground_plane_final.json \
  --v2-meta output/route_A_v2/best/static_map_meta.json \
  --output-dir output/route_A_v3_scarf/scarf_grid
```

---

## 10. 本轮不要做的事情

不要做：

```text
1. 不要把 DPVO 主轨迹替换成 ScaRF-SLAM 或 OV-SLAM 的轨迹。
2. 不要修改 project code/ScaRF-SLAM 官方代码。
3. 不要修改 project code/DPVO、VGGT、KV-tracker 官方代码。
4. 不要只靠形态学把黑墙强行抹掉。
5. 不要把 unknown 全部改成 free。
6. 不要把动态行人融合进静态点云。
7. 不要只输出 3D 点云而不输出二维导航栅格。
```

可以做：

```text
1. 使用 ScaRF-SLAM 的代码和配置作为参考。
2. 调用官方 ScaRF-SLAM 生成 dense pcd。
3. 在 src/people_bev_tracker 里实现 ScaRF-inspired 轻量 mapper。
4. 使用 Depth Anything 3 / V2 / ZoeDepth 作为深度后端。
5. 用 DPVO 轨迹锚定深度重建。
6. 用动态 mask 剔除行人。
7. 用多关键帧一致性删除假墙。
```

---

## 11. 判断标准

本轮成功的标准不是“跑通一个新模型”，而是：

```text
1. nav_binary_map.png 比 V2 更完整，中央通道不再被假墙切断。
2. largest_free_component_ratio 从 35.12% 提升到 >= 50%。
3. 黑白导航图中，白色主通道必须形成连续可行区域。
4. 三值图中，黑色 occupied 必须有多帧几何支持，不能是单帧漂浮点云。
5. BEV 视频中，相机轨迹和动态行人仍然显示在同一坐标系。
6. 额外输出 3D top-down scene，为 1811.10092v2 Figure 1 风格展示做准备。
```

如果官方 ScaRF-SLAM 环境没有跑通，也不能算完全失败。只要完成：

```text
ScaRF-inspired depth keyframe + scale consistency + submap fusion + 2D occupancy
```

并且质量指标优于 V2，就算本轮有效推进。

---

## 12. 最终建议

当前 V2 的瓶颈已经不是“二维渲染不美观”，而是：

```text
静态几何来源不可靠。
VGGT 点云产生了疑似假墙。
free space 缺少稠密、多帧一致的地面观测支撑。
```

因此下一步应该从 ScaRF-SLAM 借鉴的不是“界面”，而是这条原则：

```text
用 DPVO 这种更稳定的前端负责定位；
用 Depth Anything / GFM 只负责稠密建图；
用尺度一致性和子图融合约束单帧深度；
最后再把 3D dense map 投影成 2D navigation grid。
```

最终路线：

```text
DPVO trajectory_flat
  -> keyframe RGB
  -> dynamic mask
  -> feed-forward depth
  -> scale alignment
  -> submap fusion
  -> dense static point cloud
  -> black/white occupancy grid
  -> BEV trajectory + dynamic people
  -> 3D top-down global view
```

这比继续调整 V2 的形态学参数更值得做。
