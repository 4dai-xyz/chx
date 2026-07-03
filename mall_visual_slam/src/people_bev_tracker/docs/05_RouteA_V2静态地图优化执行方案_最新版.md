# Route A V2 优化执行方案：从稀疏点云图改成可导航黑白栅格图

## 0. 本文档给 Claude Code 的任务定位

本任务不是重新做 DPVO，也不是重新复现 KV-Track3r。当前已经确认：

```text
主相机轨迹继续使用 DPVO:
  output/dpvo/trajectory_tum.txt

动态行人继续使用 people_bev_tracker:
  YOLO/BoT-SORT + footpoint + BEV projection

本次重点只优化二维静态地图:
  output/route_A/static_map.png
  output/route_A/static_map.npy
  output/route_A/static_map_meta.json
```

现在第一版的 `static_map.png` 不合格，原因不是单纯配色问题，而是建图语义不完整：

```text
当前统计:
  occupied: 2.95%
  free:     0.80%
  unknown:  96.26%

当前视觉效果:
  画面主体是深灰 unknown
  中间只有一条白色相机轨迹 corridor
  障碍物是稀疏灰色点云
  没有形成黑色墙体、白色通行区域、连续走廊边界
```

目标是把输出从“点云散点可视化”升级为“导航可读的 occupancy grid”：

```text
最低目标:
  static_map_bw.png 像普通楼层路线图:
    黑色 = 不可通行，包括障碍和未确认区域
    白色 = 已确认可通行区域
    相机轨迹和行人叠加显示

推荐目标:
  static_map_tricolor.png 像标准 ROS occupancy grid:
    黑色 = occupied / obstacle
    白色 = free / traversable
    灰色 = unknown / unobserved

展示目标:
  paper_style_global_view.png 类似 1811.10092v2.pdf Figure 1 的 top-down global view:
    底图清楚
    相机轨迹清楚
    行人动态点清楚
    不再是稀疏点云背景
```

注意：1811.10092v2.pdf 的 Figure 1 本质是“全局俯视地图 + 多条轨迹叠加”的展示风格，不是严格黑白 occupancy grid。因此本任务需要同时输出：

```text
1. nav_binary_map.png        用于导航语义，黑白二值
2. static_map_tricolor.png   用于调试，黑/白/灰三值
3. paper_style_global_view.png 或 BEV 视频背景，用于论文式展示
```

不要修改官方开源代码目录，例如：

```text
project code/DPVO
project code/VGGT
project code/KV-tracker
project code/ORB_SLAM3-master
```

所有新增和修改都放在：

```text
src/people_bev_tracker/
output/route_A_v2/
```

已有环境和依赖能用就复用，不要重复下载环境。若需要 Depth Anything V2、VGGT 重跑或其他模型，先检查本地是否已有；没有就把该步骤写成可选，并在报告中说明未执行原因。

---

## 1. 对第一版失败原因的诊断

### 1.1 当前 static_map 的根本问题

第一版 `static_map.py` 的建图逻辑是：

```text
VGGT/DPVO/KV pointcloud
  -> 地面对齐
  -> 高度过滤
  -> 2D histogram
  -> count_thresh
  -> dilate
  -> 相机轨迹附近画一条 free corridor
```

这会导致一个必然结果：

```text
点云足够密、尺度足够准、墙体连续时，地图才好看。
点云一旦稀疏或有孔洞，地图就会变成散点。
free 只来自相机轨迹 buffer，所以白色区域一定很窄。
unknown 没有被转成可通行区域，所以画面大部分是深灰。
```

当前报告已经证明：

```text
VGGT 点云虽然有 401K 点，但地面 inlier 只有 3.4%。
height band 内障碍点看似有 211K，但投影后仍然分散。
free_corridor_radius_px = 12，分辨率 0.004 DPVO unit/px，
实际只给轨迹周围极窄区域标白。
```

因此第一版不是“调个颜色就行”，而是缺少三个关键模块：

```text
1. free space carving:
   根据相机视野、地面语义、SAM 地面 mask 或深度图，把可通行地面主动填成白色。

2. obstacle regularization:
   对障碍散点做密度平滑、聚类、闭运算、连通域过滤，把散点变成连续黑色障碍。

3. map rendering modes:
   同一份栅格需要输出二值导航图、三值调试图、论文展示图三种渲染。
```

### 1.2 当前代码里还要顺手修正的细节

检查并修正 `src/people_bev_tracker/people_bev_tracker/static_map.py`：

```text
1. 膨胀/闭运算后再次执行:
     occupied = occupied & (~free)
   避免黑色障碍被形态学操作扩进相机走过路径。

2. stats 统计 unknown 时要用互斥 mask:
     free_final = free & (~occupied)
     unknown = ~(free_final | occupied)

3. 不要只保存 static_map.png。
   至少保存:
     static_map_tricolor.png
     nav_binary_map.png
     obstacle_density.png
     free_space_mask.png
```

---

## 2. V2 总体路线选择

本次推荐按三层推进，Claude Code 必须先完成 V2.1，再做 V2.2，最后视时间做 V2.3。

### V2.1 快速达标版：不用重跑 VGGT，先把现有结果变成合格黑白栅格

输入：

```text
output/route_A/pointcloud_vggt.npy
output/route_A/trajectory_flat.txt
output/route_A/ground_plane_final.json
output/route_A/static_map_meta.json
resources/input_video.mp4
resources/input_video.mp4_bev.mp4   如果存在，用作 SAM 地面/行人语义 mask
```

核心改动：

```text
1. 加大 free 区域，不再只用轨迹细线。
2. 从 SAM 处理视频或相机视锥生成 free space mask。
3. 对障碍点做密度平滑和形态学闭运算。
4. 输出黑白二值图和三值 occupancy 图。
```

预期：

```text
1 天内完成。
不依赖新模型。
地图能达到“黑色不可通行，白色可通行”的最低视觉目标。
```

### V2.2 高质量版：引入地面语义 / 单目深度补全

输入：

```text
原始视频
DPVO 位姿
SAM 地面 mask 或 Depth Anything V2 深度
YOLO 行人 mask
```

核心改动：

```text
1. 对关键帧提取 floor mask。
2. 把 floor pixels 投影到地面平面，直接生成 dense free area。
3. 可选使用 Depth Anything V2 对关键帧生成稠密深度。
4. 行人区域先 mask out，避免把人写进静态障碍。
5. 从深度图反投影出墙、柜台、货架等静态障碍。
```

预期：

```text
2 到 3 天完成。
free 区域明显变宽，障碍边界更连贯。
适合作为后续 ROS2 在线地图的基础。
```

### V2.3 展示增强版：如果有 F5 路线图或商场平面图，做 2D 配准

如果用户提供 F5 楼层路线图、商场 CAD、消防疏散图、导览图，则优先考虑这一层。

核心改动：

```text
1. 读取 floorplan image。
2. 提取或手动指定 3 到 5 对匹配点。
3. 求 2D similarity transform:
     map_xy = s * R * dpvo_bev_xy + t
4. 把 DPVO 相机轨迹、动态行人位置投影到真实楼层图上。
```

预期：

```text
这是最容易得到“像真实商场导航图”的方案。
但是它依赖用户提供真实 floorplan，不适合作为无先验地图的默认方案。
```

---

## 3. V2.1 必做实现任务

### 3.1 新增配置文件

新增：

```text
src/people_bev_tracker/config/route_A_v2.yaml
```

内容从 `route_A.yaml` 复制并增加以下字段：

```yaml
static_map_v2:
  output_dir: "output/route_A_v2"

  # 地图分辨率不要过高。第一版 0.004 太细，点云更容易显得稀疏。
  resolution_candidates: [0.006, 0.008, 0.010]
  default_resolution_unit_per_px: 0.008
  width_px: 1200
  height_px: 1200
  auto_crop_to_active_area: true
  active_area_margin_px: 120

  # 障碍密度
  obstacle_count_thresh_candidates: [1, 2, 3]
  obstacle_gaussian_sigma_candidates: [1.0, 1.5, 2.0]
  obstacle_density_percentile_candidates: [70, 80, 90]
  obstacle_min_component_area_px: 20
  obstacle_close_kernel_candidates: [5, 9, 13]
  obstacle_dilate_kernel_candidates: [3, 5, 7]

  # free 区域。不要再只用 12 px 细 corridor。
  free_corridor_radius_unit_candidates: [0.12, 0.20, 0.30]
  free_frustum_enable: true
  free_frustum_stride_frames: 15
  free_frustum_range_unit: 0.80
  free_frustum_half_fov_deg: 35.0
  free_close_kernel_candidates: [9, 15, 21]
  free_min_component_area_px: 100

  # SAM 处理视频。如果存在就用它生成更密的 free mask。
  semantic_mask_video: "resources/input_video.mp4_bev.mp4"
  use_semantic_floor_mask: true
  semantic_stride_frames: 10
  floor_color_mode: "yellow"
  floor_hsv_lower: [15, 60, 60]
  floor_hsv_upper: [45, 255, 255]
  floor_sample_step_px: 12
  floor_projection_max_range_unit: 1.50

  # 渲染模式
  render:
    nav_binary:
      free: [255, 255, 255]
      not_free: [0, 0, 0]
    tricolor:
      occupied: [0, 0, 0]
      free: [255, 255, 255]
      unknown: [160, 160, 160]
    paper_style:
      background: [245, 245, 245]
      occupied: [20, 20, 20]
      free: [255, 255, 255]
      unknown: [220, 220, 220]
      camera_traj: [255, 120, 0]
      people: [0, 120, 255]
```

### 3.2 新增地图质量评估模块

新增：

```text
src/people_bev_tracker/people_bev_tracker/map_quality.py
```

实现函数：

```python
def evaluate_grid_quality(grid, meta, trajectory_ij=None) -> dict:
    """
    返回:
      occupied_ratio
      free_ratio
      unknown_ratio
      active_unknown_ratio
      trajectory_collision_ratio
      obstacle_component_count
      obstacle_small_component_ratio
      largest_free_component_ratio
      score
    """
```

评分建议：

```text
1. free_ratio 不能太低。
   当前 0.80% 明显不合格。
   在 active crop 区域内，free_ratio 目标 >= 15%。

2. trajectory_collision_ratio 必须接近 0。
   相机走过的位置不能被标成黑色障碍。

3. obstacle_small_component_ratio 不能太高。
   如果大部分障碍都是小散点，说明仍是点云散点图。

4. largest_free_component_ratio 要高。
   商场走廊应该形成一个连续白色主连通域。

推荐硬性阈值:
  active_free_ratio >= 0.15
  active_unknown_ratio <= 0.60
  trajectory_collision_ratio <= 0.01
  obstacle_small_component_ratio <= 0.35
```

保存：

```text
output/route_A_v2/map_quality_report.json
output/route_A_v2/map_quality_report.md
```

### 3.3 新增 free space 生成模块

新增：

```text
src/people_bev_tracker/people_bev_tracker/free_space.py
```

实现三种 free 来源，按优先级融合：

#### A. trajectory corridor

当前已有，但要改成按物理单位配置，而不是固定 px：

```python
corridor_radius_px = int(round(free_corridor_radius_unit / resolution))
```

默认尝试：

```text
0.12, 0.20, 0.30 DPVO unit
```

#### B. camera frustum carving

每隔 N 帧取一个相机位姿，估计相机在 BEV 平面上的前向方向，画一个扇形或三角形作为“相机看见且大概率可通行”的候选区域。

输入：

```text
trajectory_flat.txt
camera orientation R_wc
R_align
ground plane
```

输出：

```text
free_frustum_mask
```

实现细节：

```text
1. 对每个 keyframe:
   - 取相机中心 C_w。
   - 取相机 forward 方向。若相机坐标约定不确定，可以从 T_wc 的第三列或负第三列各生成一版，在调参脚本里选择质量更好的。
   - 映射到 aligned BEV 的 xz 平面。
   - 用 half_fov_deg 和 range_unit 生成三角形/扇形 polygon。
   - cv2.fillPoly 写入 free mask。

2. frustum free 只是候选 free。
   最终要减掉 occupied:
     free = free & (~occupied)
```

这一步能立刻把白色区域从“一条线”变成“沿相机路径展开的走廊”。

#### C. semantic floor mask projection

如果存在：

```text
resources/input_video.mp4_bev.mp4
```

则用它提取地面 mask。根据之前视频发布节点日志，这个视频里可能使用黄色/绿色标记语义区域，因此先实现 HSV 阈值可配置，不要写死。

流程：

```text
1. 每 semantic_stride_frames 读取一帧 mask video。
2. HSV 阈值提取 floor pixels。
3. 对 floor pixels 下采样，例如每 12 px 取一点。
4. 用原始相机内参 K 把像素变成 ray。
5. 与世界地面平面求交。
6. 投影到 BEV grid，标为 free candidate。
7. 对 free candidate 做 close/dilate，补小洞。
```

注意：

```text
SAM 处理后的视频只用于地面 free mask。
YOLO 检测和行人跟踪仍使用原始视频 resources/input_video.mp4。
不要用 SAM 处理视频替代原始视频做 YOLO。
```

### 3.4 新增障碍正则化模块

新增或扩展：

```text
src/people_bev_tracker/people_bev_tracker/static_map.py
```

新增函数：

```python
def build_static_map_v2(points_world, trajectory_world_xyz, ground_plane, cfg):
    ...
```

障碍生成流程：

```text
1. 点云地面对齐。
2. 高度过滤:
   h_min = 0.06 * camera_h_median
   h_max = 1.10 * camera_h_median
   如果 camera_h_median 不可靠，使用 route_A_build_report 里的 [0.0395, 0.7514] 作为 fallback。

3. 2D density histogram:
   不要直接 count >= 5。
   先用较低阈值 count >= 1/2/3 得到候选，再 GaussianBlur 平滑。

4. density threshold:
   支持两类阈值:
     fixed count threshold
     percentile threshold，例如 density >= P80

5. morphology:
   - close: 连接断裂墙体
   - dilate: 增厚障碍
   - remove small components: 删除小散点
   - fill small holes: 填小洞

6. conflict resolve:
   occupied = occupied & (~free)
```

必须输出中间图：

```text
output/route_A_v2/debug_obstacle_count.png
output/route_A_v2/debug_obstacle_density.png
output/route_A_v2/debug_obstacle_raw.png
output/route_A_v2/debug_obstacle_regularized.png
```

### 3.5 新增参数搜索脚本

新增：

```text
src/people_bev_tracker/scripts/tune_static_map_v2.py
```

功能：

```text
读取 route_A_v2.yaml
自动组合搜索:
  resolution
  obstacle_count_thresh
  obstacle_gaussian_sigma
  obstacle_density_percentile
  obstacle_close_kernel
  obstacle_dilate_kernel
  free_corridor_radius_unit
  free_close_kernel

每组输出:
  candidate_xxx/nav_binary_map.png
  candidate_xxx/static_map_tricolor.png
  candidate_xxx/debug_*.png
  candidate_xxx/quality.json

按 map_quality.score 排序，复制最佳结果到:
  output/route_A_v2/best/
```

第一轮搜索不要太大，避免组合爆炸。推荐先固定一部分参数：

```text
resolution: [0.008, 0.010]
obstacle_count_thresh: [1, 2]
obstacle_gaussian_sigma: [1.0, 1.5]
obstacle_close_kernel: [9, 13]
obstacle_dilate_kernel: [5]
free_corridor_radius_unit: [0.20, 0.30]
free_close_kernel: [15]
```

总组合：

```text
2 * 2 * 2 * 2 * 1 * 2 * 1 = 32
```

足够第一轮筛选。

---

## 4. V2.2 可选高质量实现：Depth Anything V2 / 单目深度

只有在 V2.1 仍然不能得到清楚地图时，才执行 V2.2。

### 4.1 先检查本地是否已有 Depth Anything V2

Claude Code 先检查：

```bash
find "/home/ros/ros2_orbslam3" -maxdepth 5 -iname "*Depth*Anything*" -o -iname "*depth_anything*"
```

如果已有，复用现有代码和环境。

如果没有，不要直接重复下载环境。先在执行报告里说明：

```text
Depth Anything V2 not found locally.
V2.2 depth-based mapping skipped.
```

### 4.2 新增深度关键帧建图脚本

新增：

```text
src/people_bev_tracker/scripts/build_depth_static_map.py
```

流程：

```text
1. 每 10 到 15 帧选一个关键帧。
2. 用 YOLO person mask 或已有 SAM person mask 把行人区域 mask out。
3. 对关键帧跑 Depth Anything V2 或 ZoeDepth。
4. 用 floor mask + 已知相机高度做 scale alignment。
5. 反投影为 dense pointcloud。
6. 按高度过滤:
     near ground -> free candidate
     0.1h 到 1.2h -> obstacle candidate
7. 融合到 static_map_v2。
```

尺度恢复建议：

```text
已知相机高度 H_real 可以先不转真米，而是使用 DPVO 单位下的 camera_h_median。

对 floor pixels:
  用相对深度 d_rel 反投影出点 X_rel。
  计算这些点到地面的中位距离 h_rel。
  scale = camera_h_median / h_rel
  depth_metric_like = scale * d_rel
```

注意：

```text
Depth 网络输出可以不是真米。
只要和 DPVO 地面高度对齐，就可以用于本工程的 BEV 栅格。
```

---

## 5. 如果要重新生成 VGGT 点云，应该怎么做

当前 VGGT 点云在报告中是：

```text
output/vggt_aligned_full_run/aligned_full/aligned_full_scene.ply
过滤后 304812 点
地面 inlier 只有 3.4%
```

这说明它能用，但不是 map-grade。若要重新生成，不要覆盖旧文件，先输出到：

```text
output/route_A_v2/vggt_regen/
```

重生成原则：

```text
1. 增加关键帧覆盖，优先覆盖走廊转弯、开阔区域、障碍边界。
2. 保留 confidence，如果 VGGT 输出 confidence，则只用高置信点建 obstacle。
3. 不要只追求点数多，要追求墙体和地面连续。
4. 重生成后必须跑同一套 map_quality 对比，只有质量分更高才替换默认点云。
```

Claude Code 只需要把重生成做成可选命令，不要让整个 V2 依赖 VGGT 重跑。

---

## 6. 渲染规范

### 6.1 栅格内部值

为了兼容当前代码，内部可以继续使用：

```text
0   = unknown
127 = free
255 = occupied
```

但是输出图片必须分开渲染。

### 6.2 三值调试图

输出：

```text
output/route_A_v2/best/static_map_tricolor.png
```

颜色：

```text
occupied: black      [0, 0, 0]
free:     white      [255, 255, 255]
unknown:  light gray [160, 160, 160]
```

用途：

```text
检查哪些区域是真 free，哪些区域只是 unknown。
```

### 6.3 导航二值图

输出：

```text
output/route_A_v2/best/nav_binary_map.png
```

颜色：

```text
free: white
occupied + unknown: black
```

用途：

```text
满足“黑色不可通行，白色可通行”的最低目标。
```

### 6.4 展示图

输出：

```text
output/route_A_v2/best/paper_style_global_view.png
```

内容：

```text
底图:
  使用 static_map_tricolor 或 nav_binary_map 的柔和版本

叠加:
  DPVO camera trajectory
  current camera pose arrow
  people track points
  optional start/end markers
```

要求：

```text
1. 不要把文字、网格线、调试信息盖住地图主体。
2. 相机轨迹用高对比色，例如 orange 或 blue。
3. 动态行人用另一种高对比色，例如 red/cyan。
4. 输出一张静态最终帧图，以及一个 BEV 视频。
```

---

## 7. 集成到 offline_pipeline_A

新增或修改：

```text
src/people_bev_tracker/scripts/offline_pipeline_A.py
```

要求：

```text
1. 支持读取 V2 best 地图:
   --static-map output/route_A_v2/best/static_map.npy
   --static-map-meta output/route_A_v2/best/static_map_meta.json

2. 支持选择渲染模式:
   --map-render-mode tricolor
   --map-render-mode binary
   --map-render-mode paper

3. 输出:
   output/route_A_v2/bev_tracking_route_A_v2.mp4
   output/route_A_v2/bev_tracking_clean_route_A_v2.mp4
   output/route_A_v2/final_frame_route_A_v2.png
```

如果现有 `BEVCanvas` 只接受 BGR static layer，则保持这个接口，新增一个工具函数：

```python
load_static_layer_for_render_mode(static_map_npy, meta_json, mode) -> np.ndarray
```

不要把渲染逻辑散落在多个脚本里。

---

## 8. 执行命令

### 8.1 先跑 V2 参数搜索

```bash
cd /home/ros/ros2_orbslam3
conda activate dpvo

python src/people_bev_tracker/scripts/tune_static_map_v2.py \
  --config src/people_bev_tracker/config/route_A_v2.yaml \
  --pose output/route_A/trajectory_flat.txt \
  --pointcloud output/route_A/pointcloud_vggt.npy \
  --ground-plane output/route_A/ground_plane_final.json \
  --base-meta output/route_A/static_map_meta.json \
  --output-dir output/route_A_v2
```

### 8.2 用最佳地图重跑 BEV 视频

```bash
python src/people_bev_tracker/scripts/offline_pipeline_A.py \
  --config src/people_bev_tracker/config/route_A_v2.yaml \
  --pose output/route_A/trajectory_flat.txt \
  --static-map output/route_A_v2/best/static_map.npy \
  --static-map-meta output/route_A_v2/best/static_map_meta.json \
  --ground-plane output/route_A/ground_plane_final.json \
  --output-dir output/route_A_v2 \
  --map-render-mode paper
```

### 8.3 输出最终报告

新增：

```text
output/route_A_v2/route_A_v2_optimization_report.md
```

报告必须包含：

```text
1. 第一版问题复盘:
   - 第一版 occupied/free/unknown 比例
   - 第一版为什么稀疏

2. V2 使用的输入:
   - pointcloud path
   - trajectory path
   - semantic mask video 是否存在
   - 是否使用 Depth Anything / VGGT regen

3. 最佳参数:
   - resolution
   - obstacle threshold
   - morphology kernels
   - free generation mode

4. V2 质量指标:
   - active_free_ratio
   - active_unknown_ratio
   - trajectory_collision_ratio
   - obstacle_small_component_ratio
   - largest_free_component_ratio

5. 输出文件清单:
   - nav_binary_map.png
   - static_map_tricolor.png
   - paper_style_global_view.png
   - bev_tracking_route_A_v2.mp4

6. 仍然存在的问题:
   - 单目尺度
   - 商场玻璃/反光
   - 行人遮挡
   - 点云源质量
```

---

## 9. 验收标准

V2 不能只说“跑通”，必须满足以下最低标准。

### 9.1 图像输出标准

必须存在：

```text
output/route_A_v2/best/nav_binary_map.png
output/route_A_v2/best/static_map_tricolor.png
output/route_A_v2/best/paper_style_global_view.png
output/route_A_v2/bev_tracking_route_A_v2.mp4
output/route_A_v2/route_A_v2_optimization_report.md
```

### 9.2 视觉标准

打开 `nav_binary_map.png`：

```text
1. 白色区域必须形成连续可通行走廊，不应只是相机轨迹细线。
2. 黑色区域必须明显表示不可通行或未知区域。
3. 相机走过的路径不能被黑色障碍切断。
```

打开 `static_map_tricolor.png`：

```text
1. 黑色障碍不能是满屏散点。
2. 白色 free 应该覆盖相机视野内可通行地面。
3. 灰色 unknown 允许存在，但不能在 active area 内占绝大多数。
```

打开 `bev_tracking_route_A_v2.mp4`：

```text
1. 地图背景要清楚。
2. 相机轨迹要稳定贴在地图上。
3. 行人位置要叠加在同一坐标系。
4. 不要出现文字重叠、轨迹挡住整张图、地图过暗等问题。
```

### 9.3 定量标准

在 active crop 区域内，目标指标：

```text
active_free_ratio >= 0.15
active_unknown_ratio <= 0.60
trajectory_collision_ratio <= 0.01
obstacle_small_component_ratio <= 0.35
largest_free_component_ratio >= 0.50
```

如果语义 mask 不可用、Depth Anything 不可用，则可以放宽：

```text
active_free_ratio >= 0.08
active_unknown_ratio <= 0.75
```

但报告中必须明确说明原因。

---

## 10. 推荐执行顺序

Claude Code 请按这个顺序执行，不要跳步：

```text
Step 1:
  读取 output/route_A/route_A_report.md
  读取 output/route_A/route_A_build_report.md
  读取 output/route_A/static_map_meta.json
  确认第一版指标。

Step 2:
  新建 route_A_v2.yaml。
  新建 map_quality.py。
  新建 free_space.py。

Step 3:
  扩展 static_map.py，加入 build_static_map_v2 和多模式 render。
  修复形态学后 occupied/free 冲突。

Step 4:
  新建 tune_static_map_v2.py。
  先用现有 VGGT 点云跑 32 组轻量参数搜索。

Step 5:
  生成 output/route_A_v2/best。
  输出 nav_binary_map.png、static_map_tricolor.png、debug 图和 quality report。

Step 6:
  修改 offline_pipeline_A.py 支持 map-render-mode。
  用 best 地图重跑 BEV 视频。

Step 7:
  写 output/route_A_v2/route_A_v2_optimization_report.md。
```

---

## 11. 本次不要做的事情

不要做：

```text
1. 不要把 KV-Track3r 轨迹替换 DPVO 轨迹。
2. 不要修改官方 DPVO / VGGT / KV-Track3r 源码。
3. 不要第一步就重跑全部 VGGT。
4. 不要把 unknown 全部强行当 free。
5. 不要只调颜色而不改 free/occupied 语义。
6. 不要用 SAM 处理后的视频替代原始视频做 YOLO 行人检测。
7. 不要把动态行人写进 static_map。
```

可以做：

```text
1. 用 SAM 处理视频提取 floor mask，辅助 free space。
2. 用相机视锥补全可通行区域。
3. 用现有点云做障碍正则化。
4. 可选检查本地 Depth Anything V2，没有就跳过。
5. 可选重跑 VGGT，但输出到 route_A_v2，不覆盖第一版。
```

---

## 12. 最终判断

当前第一版的问题主要是：

```text
它把点云投影结果当成地图。
```

V2 要改成：

```text
用点云、语义地面、相机视锥、轨迹共同估计 occupancy grid。
```

也就是说：

```text
障碍物来自 pointcloud/depth。
可通行区域来自 trajectory corridor + camera frustum + floor mask。
未知区域保留为 unknown，但在二值导航图里按不可通行处理。
```

做到这一步后，输出才会从“稀疏点云图”变成“能给商场导航使用的二维俯视栅格地图”。
