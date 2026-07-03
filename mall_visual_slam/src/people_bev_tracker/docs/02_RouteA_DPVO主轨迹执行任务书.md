# 路线 A 执行报告：DPVO 主轨迹 + 二维栅格地图 + 动态行人 BEV

## 0. 报告目的

本报告用于指导 Claude Code 接手实现“商场单目视频 → 二维 BEV/栅格地图 → 实时/离线显示相机轨迹、动态行人、障碍和可通行区域”的路线 A。

现有两个计划文档：

```text
src/people_bev_tracker/docs/01_RouteA_初版执行方案.md
src/KV-tracker/docs/KV_Track3r_商场导航与BEV地图应用方案.md
```

本报告在它们基础上做查漏补缺和执行顺序修正。核心变化是：

```text
KV-Tracker 轨迹质量不如 DPVO，因此第一版不再让 KV-Tracker 参与主相机位姿。
DPVO 是唯一主轨迹源。
KV-Tracker / VGGT 只作为静态结构点云的可选增强源。
```

Claude Code 执行时应优先完成：

```text
DPVO trajectory_tum.txt
  + YOLO/BoT-SORT 行人
  + DPVO/KV/VGGT 点云生成静态占据栅格
  -> output/route_A/bev_tracking_route_A.mp4
```

---

## 1. 当前结论

### 1.1 已验证事实

当前仓库已有：

```text
DPVO 轨迹:
  output/dpvo/trajectory_tum.txt

DPVO 点云:
  project code/DPVO/mall_dpvo.ply

KV-Tracker 结果:
  output/kv_track3r_repro/traj.npy
  output/kv_track3r_repro/pcd.npy

VGGT 对齐点云:
  output/vggt_aligned_full_run/aligned_full/aligned_full_scene.ply
```

用户已经观察到：

```text
KV-Tracker 产生的相机位姿轨迹效果不如 DPVO。
```

因此路线 A 应改为：

```text
主轨迹: DPVO
动态行人: people_bev_tracker 已有 YOLO-seg + BoT-SORT
静态地图: 优先 DPVO 点云，备选 VGGT 点云，最后再尝试 KV 点云
KV-Tracker: 不参与主位姿，只作为结构/论文方法参考
```

### 1.2 原计划需要修正的地方

原计划中有以下风险：

1. **把 KV 轨迹接入 people BEV 作为 pose 源**

   现在不推荐。KV 轨迹已经被用户判断不如 DPVO，因此所有第一版输出都应以 DPVO 为准。

2. **第一步就做 KV → DPVO Sim(3) 对齐**

   不应作为主路径。Sim(3) 对齐可以保留为 V2 增强，但 V1 应先用 DPVO 点云和 DPVO 轨迹生成静态地图。

3. **把地面拟合完全依赖 KV 稠密点云**

   风险较高。KV 点云坐标系、尺度、轨迹稳定性都会影响地面。第一版应优先使用：

   ```text
   DPVO PLY + DPVO 轨迹 + camera-ground fallback
   ```

4. **“可通行区域”和“未知区域”没有严格区分**

   二维栅格地图必须明确：

   ```text
   occupied: 高于地面的静态障碍点，例如墙、柜台、货架
   free: 相机走过路径附近的可通行区域
   unknown: 没有观测到的区域
   dynamic: 当前行人位置，不写入静态地图
   ```

5. **缺少预检和现有 bug 修复**

   当前 `src/people_bev_tracker/people_bev_tracker/bev_canvas.py` 约第 139 行疑似有语法错误：

   ```python
   0.45,5
   self.cam_color,
   ```

   Claude Code 必须先跑 Python 编译检查并修复这类基础问题。

---

## 2. 最终目标

### 2.1 第一版目标

生成一个二维 BEV 视频：

```text
output/route_A/bev_tracking_route_A.mp4
```

画面包含：

```text
1. 静态二维栅格地图背景
   - unknown: 未观测区域
   - free: 相机轨迹附近可通行区域
   - occupied: 点云投影出的障碍区域

2. DPVO 相机轨迹
   - 历史轨迹线
   - 当前相机位置
   - 当前相机朝向箭头

3. 动态行人
   - 当前行人位置
   - track_id
   - 可选短历史轨迹
```

### 2.2 输出目录

所有路线 A 输出统一放到：

```text
output/route_A/
```

建议输出：

```text
output/route_A/
├── route_A_report.md
├── preflight_report.json
├── ground_plane_final.json
├── trajectory_flat.txt
├── static_map.npy
├── static_map.png
├── static_map_meta.json
├── bev_tracking_route_A.mp4
├── bev_tracking_clean_route_A.mp4
├── debug_overlay_route_A.mp4
├── people_tracks_route_A.json
├── camera_trajectory_route_A.json
└── validation_summary.json
```

可选增强输出：

```text
output/route_A/
├── pointcloud_dpvo.npy
├── pointcloud_vggt.npy
├── pointcloud_kv_aligned.npy
├── sim3_kv_to_dpvo.json
└── static_map_source_comparison/
```

---

## 3. 执行原则

### 3.1 主轨迹原则

第一版只使用：

```text
output/dpvo/trajectory_tum.txt
```

不要使用：

```text
output/kv_track3r_repro/trajectory_tum.txt
```

作为主轨迹。

KV 轨迹只能用于对比或可选增强，不得影响默认输出。

### 3.2 静态地图源优先级

默认优先级：

```text
1. DPVO 点云: project code/DPVO/mall_dpvo.ply
2. VGGT 对齐点云: output/vggt_aligned_full_run/aligned_full/aligned_full_scene.ply
3. KV 点云: output/kv_track3r_repro/pcd.npy，经 Sim(3) 对齐后再用
4. 纯轨迹 fallback: 只用 DPVO 轨迹生成 free corridor，不生成 occupied 障碍
```

原因：

```text
DPVO 坐标系与 DPVO 轨迹天然一致。
VGGT 已有 aligned_full 点云，适合作为结构参考，但要注意坐标系。
KV 点云密度高，但其轨迹质量和坐标系稳定性已被质疑，应后置。
```

### 3.3 先离线，后实时

第一版不做 ROS2 实时节点。先完成离线脚本：

```text
src/people_bev_tracker/scripts/build_route_A.py
src/people_bev_tracker/scripts/offline_pipeline_A.py
```

可选 `--live` 用 `cv2.imshow` 显示当前 BEV，但输出 MP4/JSON 是第一优先级。

---

## 4. 查漏补缺：需要新增或修改的文件

### 4.1 必须先修

文件：

```text
src/people_bev_tracker/people_bev_tracker/bev_canvas.py
```

任务：

```text
1. 修复疑似语法错误 `0.45,5`。
2. 确认 `python -m py_compile` 能通过。
3. 给 BEVCanvas 增加 static_layer 支持。
```

编译检查：

```bash
python -m py_compile \
  src/people_bev_tracker/people_bev_tracker/*.py \
  src/people_bev_tracker/scripts/*.py
```

### 4.2 新增模块

#### `ground_fit.py`

路径：

```text
src/people_bev_tracker/people_bev_tracker/ground_fit.py
```

职责：

```text
1. 从点云估计地面平面。
2. 支持 RANSAC、bottom-percentile PCA、camera-ground fallback。
3. 输出 ground_plane_final.json。
```

核心 API：

```python
def fit_ground_ransac(points: np.ndarray, cfg: dict) -> dict:
    ...

def fit_ground_bottom_pca(points: np.ndarray, cfg: dict) -> dict:
    ...

def make_camera_ground_fallback(cfg: dict) -> dict:
    ...

def choose_best_ground(candidates: list[dict]) -> dict:
    ...
```

输出格式：

```json
{
  "method": "ransac_dpvo",
  "normal": [0.0, 1.0, 0.0],
  "d": -0.1,
  "inlier_ratio": 0.72,
  "rmse": 0.012,
  "normal_axis_hint": "y",
  "source": "project code/DPVO/mall_dpvo.ply",
  "coordinate_frame": "dpvo_world"
}
```

#### `pointcloud_io.py`

路径：

```text
src/people_bev_tracker/people_bev_tracker/pointcloud_io.py
```

职责：

```text
1. 读取 ASCII/Binary PLY。
2. 读取 NPY 点云。
3. 稳健过滤 NaN、Inf、极端离群点。
4. 必要时抽样，避免一次性加载过大。
```

核心 API：

```python
def load_points(path: str, max_points: int = 0) -> np.ndarray:
    ...

def robust_filter_points(points: np.ndarray, percentile=(1, 99)) -> np.ndarray:
    ...

def save_points_npy(path: str, points: np.ndarray) -> None:
    ...
```

#### `trajectory_flatten.py`

路径：

```text
src/people_bev_tracker/people_bev_tracker/trajectory_flatten.py
```

职责：

```text
1. 读取 DPVO TUM 轨迹。
2. 根据 ground_plane_final.json 去除相机高度方向的步态抖动。
3. 输出 trajectory_flat.txt。
```

核心公式：

设地面：

```text
n^T X + d = 0
```

相机中心：

```text
C(t)
```

高度：

```text
h(t) = n^T C(t) + d
```

平面化：

```text
C_flat(t) = C(t) + (h_ref - h(t)) n
```

其中：

```text
h_ref = median(h(t))
```

第一版只改平移，不改旋转。

#### `static_map.py`

路径：

```text
src/people_bev_tracker/people_bev_tracker/static_map.py
```

职责：

```text
1. 把点云投影到地面对齐坐标系。
2. 按高度筛出障碍点。
3. 生成 occupied/free/unknown 栅格。
4. 保存 static_map.npy / static_map.png / static_map_meta.json。
```

栅格语义：

```text
0   unknown
127 free
255 occupied
```

或保存为多通道：

```text
unknown_mask
free_mask
occupied_mask
cost
```

核心 API：

```python
def build_static_map(
    points: np.ndarray,
    trajectory_bev: np.ndarray,
    ground_plane: dict,
    cfg: dict,
) -> tuple[np.ndarray, dict]:
    ...

def render_static_map(grid: np.ndarray, meta: dict) -> np.ndarray:
    ...
```

#### `static_layer` 支持

修改：

```text
src/people_bev_tracker/people_bev_tracker/bev_canvas.py
```

新增：

```python
class BEVCanvas:
    def __init__(..., static_layer: Optional[np.ndarray] = None, ...):
        self.static_layer = static_layer
```

`_make_base()` 中：

```text
如果 static_layer 不为空，先使用 static_layer 作为底图，再叠加网格线和坐标轴。
否则使用原来的纯色背景。
```

### 4.3 新增脚本

#### `build_route_A.py`

路径：

```text
src/people_bev_tracker/scripts/build_route_A.py
```

职责：

```text
阶段 1-4 一键构建静态地图和轨迹平面化。
```

输入：

```text
--video resources/input_video.mp4
--pose output/dpvo/trajectory_tum.txt
--pointcloud "project code/DPVO/mall_dpvo.ply"
--calib config/KannalaBrandt8_1280x720.yaml
--output-dir output/route_A
--config src/people_bev_tracker/config/route_A.yaml
```

输出：

```text
ground_plane_final.json
trajectory_flat.txt
static_map.npy
static_map.png
static_map_meta.json
route_A_report.md
```

#### `offline_pipeline_A.py`

路径：

```text
src/people_bev_tracker/scripts/offline_pipeline_A.py
```

职责：

```text
基于现有 offline_pipeline.py，增加静态地图背景和 route_A 输出命名。
```

输入：

```text
--video resources/input_video.mp4
--calib config/KannalaBrandt8_1280x720.yaml
--pose output/route_A/trajectory_flat.txt
--static-map output/route_A/static_map.png
--static-map-meta output/route_A/static_map_meta.json
--ground-plane output/route_A/ground_plane_final.json
--output-dir output/route_A
--live
```

输出：

```text
bev_tracking_route_A.mp4
bev_tracking_clean_route_A.mp4
debug_overlay_route_A.mp4
people_tracks_route_A.json
camera_trajectory_route_A.json
validation_summary.json
```

#### `tune_static_map_params.py`

路径：

```text
src/people_bev_tracker/scripts/tune_static_map_params.py
```

职责：

```text
扫描不同 resolution / height_range / count_thresh / dilation 参数，输出多张候选 static_map.png，方便人工挑选。
```

输出：

```text
output/route_A/static_map_candidates/
```

---

## 5. 新配置文件

新增：

```text
src/people_bev_tracker/config/route_A.yaml
```

建议初始内容：

```yaml
input:
  video: "resources/input_video.mp4"
  calib: "config/KannalaBrandt8_1280x720.yaml"
  pose_tum: "output/dpvo/trajectory_tum.txt"
  primary_pointcloud: "project code/DPVO/mall_dpvo.ply"
  optional_vggt_pointcloud: "output/vggt_aligned_full_run/aligned_full/aligned_full_scene.ply"
  optional_kv_traj: "output/kv_track3r_repro/traj.npy"
  optional_kv_pcd: "output/kv_track3r_repro/pcd.npy"

pose:
  source: "dpvo"
  timestamp_unit: "dpvo_tick"
  dpvo_stride: 2
  scale: 1.0
  flatten: true
  flatten_mode: "constant"

pointcloud:
  source: "dpvo"
  max_points: 500000
  outlier_percentile: [1.0, 99.0]

ground:
  method: "auto"
  axis_hint: "y"
  ransac_distance_threshold: 0.015
  ransac_iterations: 2000
  bottom_percentile: 20.0
  normal_max_angle_deg: 35.0
  fallback_mode: "camera"
  fallback_camera_height: 0.1
  fallback_camera_pitch_deg: 15.0

static_map:
  resolution_unit_per_px: 0.004
  width_px: 1200
  height_px: 1200
  origin_world: [0.0, 1.5]
  bev_axes: ["x", "z"]
  obstacle_height_range: [0.02, 0.35]
  count_thresh: 5
  dilate_kernel: 3
  gaussian_blur_sigma: 0.0
  free_corridor_radius_px: 12
  colors:
    unknown: [35, 35, 35]
    free: [205, 205, 205]
    occupied: [70, 70, 70]

people:
  model: "yolo11n-seg.pt"
  tracker: "botsort.yaml"
  conf_thres: 0.35
  imgsz: 960

output:
  dir: "output/route_A"
  live_window: false
  save_clean_video: true
```

---

## 6. 执行顺序

### Step 0: 预检和基础修复

Claude Code 先执行：

```bash
cd /home/ros/ros2_orbslam3
conda activate dpvo

python -m py_compile \
  src/people_bev_tracker/people_bev_tracker/*.py \
  src/people_bev_tracker/scripts/*.py
```

如果失败，先修复语法错误。

然后检查输入：

```bash
test -f resources/input_video.mp4
test -f output/dpvo/trajectory_tum.txt
test -f "project code/DPVO/mall_dpvo.ply"
```

生成：

```text
output/route_A/preflight_report.json
```

记录：

```text
video 是否存在
DPVO 轨迹是否存在
DPVO 点云是否存在
KV/VGGT 可选点云是否存在
Python 编译是否通过
```

### Step 1: 复现当前 people BEV baseline

先不做静态地图，只跑现有短测试：

```bash
python src/people_bev_tracker/scripts/offline_pipeline.py \
  --video resources/input_video.mp4 \
  --calib config/KannalaBrandt8_1280x720.yaml \
  --pose output/dpvo/trajectory_tum.txt \
  --output-dir output/people_bev_test \
  --max-frames 30
```

验收：

```text
output/people_bev_test/bev_tracking.mp4 存在
output/people_bev_test/debug_overlay.mp4 存在
people_tracks.json 非空
camera_trajectory.json 非空
```

如果 baseline 跑不通，不继续做 route A。

### Step 2: 构建 route_A 静态地图

运行：

```bash
python src/people_bev_tracker/scripts/build_route_A.py \
  --config src/people_bev_tracker/config/route_A.yaml \
  --video resources/input_video.mp4 \
  --pose output/dpvo/trajectory_tum.txt \
  --pointcloud "project code/DPVO/mall_dpvo.ply" \
  --output-dir output/route_A
```

输出必须包含：

```text
output/route_A/ground_plane_final.json
output/route_A/trajectory_flat.txt
output/route_A/static_map.npy
output/route_A/static_map.png
output/route_A/static_map_meta.json
output/route_A/route_A_report.md
```

### Step 3: 跑 route_A 主流水线

运行：

```bash
python src/people_bev_tracker/scripts/offline_pipeline_A.py \
  --video resources/input_video.mp4 \
  --calib config/KannalaBrandt8_1280x720.yaml \
  --pose output/route_A/trajectory_flat.txt \
  --static-map output/route_A/static_map.png \
  --static-map-meta output/route_A/static_map_meta.json \
  --ground-plane output/route_A/ground_plane_final.json \
  --output-dir output/route_A
```

输出：

```text
output/route_A/bev_tracking_route_A.mp4
output/route_A/bev_tracking_clean_route_A.mp4
output/route_A/debug_overlay_route_A.mp4
output/route_A/people_tracks_route_A.json
output/route_A/camera_trajectory_route_A.json
output/route_A/validation_summary.json
```

### Step 4: 参数扫描

如果 `static_map.png` 不好看，运行：

```bash
python src/people_bev_tracker/scripts/tune_static_map_params.py \
  --config src/people_bev_tracker/config/route_A.yaml \
  --pointcloud "project code/DPVO/mall_dpvo.ply" \
  --pose output/dpvo/trajectory_tum.txt \
  --output-dir output/route_A/static_map_candidates
```

输出多组候选：

```text
output/route_A/static_map_candidates/*.png
```

人工选择后把参数写回 `route_A.yaml`。

### Step 5: 可选 V2 增强，加入 VGGT/KV 点云

只有 V1 成功后再做。

VGGT 点云优先于 KV 点云：

```text
output/vggt_aligned_full_run/aligned_full/aligned_full_scene.ply
```

KV 点云需要先做 Sim(3)：

```text
output/kv_track3r_repro/pcd.npy
output/kv_track3r_repro/traj.npy
  -> Sim(3) 对齐到 DPVO
  -> output/route_A/pointcloud_kv_aligned.npy
```

注意：V2 只能增强静态图层，不改变 DPVO 主轨迹。

---

## 7. 静态地图算法细化

### 7.1 地面坐标系

地面平面：

```text
n^T X + d = 0
```

将点云变换到地面对齐坐标系：

```text
n -> +Y
```

然后 BEV 使用：

```text
bev_axes = ["x", "z"]
```

### 7.2 occupied 判定

点到地面高度：

```text
h = n^T X + d
```

障碍点满足：

```text
h_min <= h <= h_max
```

默认：

```text
h_min = 0.02 DPVO unit
h_max = 0.35 DPVO unit
```

这些不是米制，是 DPVO 单目尺度单位。

### 7.3 free 判定

第一版不要从“没有障碍点”直接推断 free，因为单目点云稀疏，未观测区域很多。

free 只来自：

```text
DPVO 相机轨迹附近 corridor
```

也就是把平面化相机轨迹投到 BEV，然后膨胀一个半径：

```text
free_corridor_radius_px = 12
```

这样语义更可靠：

```text
相机走过的地方 = 已知可通行
障碍点密集处 = occupied
其他地方 = unknown
```

### 7.4 dynamic 行人不写入静态地图

YOLO 检测到的动态行人只作为前景层显示：

```text
people layer
```

不要把行人点写进 `static_map.npy`，否则人走过的地方会变成障碍物。

---

## 8. 验收标准

### 8.1 代码验收

必须通过：

```bash
python -m py_compile \
  src/people_bev_tracker/people_bev_tracker/*.py \
  src/people_bev_tracker/scripts/*.py
```

### 8.2 Baseline 验收

```text
output/people_bev_test/bev_tracking.mp4
output/people_bev_test/debug_overlay.mp4
```

可正常播放。

### 8.3 Static Map 验收

```text
output/route_A/static_map.png
```

应满足：

```text
1. 图中能看到 unknown/free/occupied 三类区域。
2. 相机轨迹附近是 free。
3. occupied 区域不是整图全黑/全白。
4. validation_summary.json 中 occupied_ratio 在合理范围，例如 0.001 ~ 0.35。
```

### 8.4 Route A 视频验收

```text
output/route_A/bev_tracking_route_A.mp4
```

应包含：

```text
1. 静态栅格背景。
2. 相机历史轨迹和当前朝向。
3. 动态行人点和 track_id。
4. HUD 显示 frame / timestamp / active person count。
```

### 8.5 数据验收

`validation_summary.json` 至少包含：

```json
{
  "pose_hit": 0,
  "pose_miss": 0,
  "projection_ok": 0,
  "projection_fail": 0,
  "occupied_ratio": 0.0,
  "free_ratio": 0.0,
  "unknown_ratio": 0.0,
  "people_tracks": 0,
  "source_pose": "output/route_A/trajectory_flat.txt",
  "source_pointcloud": "project code/DPVO/mall_dpvo.ply"
}
```

要求：

```text
pose_miss 尽量为 0
projection_ok / (projection_ok + projection_fail) > 0.95
```

---

## 9. 风险与降级方案

| 风险 | 表现 | 降级 |
| :--- | :--- | :--- |
| `bev_canvas.py` 语法错误 | py_compile 失败 | 先修基础语法 |
| DPVO 点云太稀疏 | occupied 很少 | 用 VGGT aligned 点云增强 |
| KV 点云对齐失败 | Sim(3) ATE 大 | 跳过 KV，不影响主线 |
| 地面 RANSAC 找错平面 | 地图倾斜、障碍乱飞 | 使用 camera-ground fallback |
| occupancy 全黑/全白 | 阈值不合适 | 用 `tune_static_map_params.py` 扫参数 |
| 投影行人位置发散 | 地面/尺度不对 | 回退现有 `ground.mode=camera` 配置 |
| 端到端太慢 | YOLO/绘图慢 | 降低视频帧率、frame_stride=2、只每 N 帧显示 live |

---

## 10. Claude Code 执行清单

按顺序执行，不要跳步。

### 10.1 预检

```text
1. 读取本报告。
2. 读取 01_RouteA_初版执行方案.md。
3. 读取 people_bev_tracker/IMPLEMENTATION.md。
4. 执行 py_compile。
5. 修复基础语法错误。
```

### 10.2 先复现现有 baseline

```text
1. 跑 offline_pipeline.py --max-frames 30。
2. 确认 output/people_bev_test 有视频和 JSON。
3. 如果失败，先修 baseline，不写新功能。
```

### 10.3 实现 route A V1

新增：

```text
ground_fit.py
pointcloud_io.py
trajectory_flatten.py
static_map.py
config/route_A.yaml
scripts/build_route_A.py
scripts/offline_pipeline_A.py
scripts/tune_static_map_params.py
```

修改：

```text
bev_canvas.py
```

### 10.4 运行完整 V1

```bash
python src/people_bev_tracker/scripts/build_route_A.py \
  --config src/people_bev_tracker/config/route_A.yaml \
  --video resources/input_video.mp4 \
  --pose output/dpvo/trajectory_tum.txt \
  --pointcloud "project code/DPVO/mall_dpvo.ply" \
  --output-dir output/route_A

python src/people_bev_tracker/scripts/offline_pipeline_A.py \
  --video resources/input_video.mp4 \
  --calib config/KannalaBrandt8_1280x720.yaml \
  --pose output/route_A/trajectory_flat.txt \
  --static-map output/route_A/static_map.png \
  --static-map-meta output/route_A/static_map_meta.json \
  --ground-plane output/route_A/ground_plane_final.json \
  --output-dir output/route_A
```

### 10.5 生成最终报告

完成后写：

```text
output/route_A/route_A_report.md
```

内容：

```text
1. 使用的输入文件。
2. 使用的主轨迹源: DPVO。
3. 使用的静态点云源。
4. 地面拟合方法和指标。
5. static_map 统计。
6. people projection 统计。
7. 输出文件列表。
8. 当前限制和下一步建议。
```

---

## 11. 明确不做的事

第一版不要做：

```text
1. 不把 KV-Tracker 轨迹作为主轨迹。
2. 不改官方 KV-Tracker 源码。
3. 不做 ROS2 实时 people BEV node。
4. 不做真实商场 CAD 平面图配准。
5. 不强求米制尺度。
6. 不把未观测区域标为 free。
7. 不把行人写入静态 occupancy。
```

---

## 12. 后续 V2 / V3

### V2: 结构增强

```text
1. 使用 VGGT aligned 点云增强 occupied。
2. 尝试 KV pcd.npy 经 Sim(3) 对齐后增强 occupied。
3. 输出三组 static_map 对比:
   - DPVO only
   - DPVO + VGGT
   - DPVO + VGGT + KV
```

### V3: 实时 ROS2

```text
1. 把 people_bev_tracker 做成 ROS2 node。
2. 订阅 /camera/image_raw。
3. 订阅 /slam/pose 或读取 DPVO 在线节点。
4. 发布 /bev/map_image、/bev/people_markers、/bev/camera_path。
```

### V4: 真实地图配准

```text
1. 引入商场 CAD 或楼层平面图。
2. 选取若干控制点。
3. 做 2D Similarity / Homography / ICP 配准。
4. 输出真实地图坐标中的相机和行人轨迹。
```

---

## 13. 最终判断

本路线的正确优先级是：

```text
DPVO 主轨迹稳定性 > 静态地图美观度 > KV/VGGT 结构增强 > 实时 ROS2
```

所以 Claude Code 实现时必须先把以下链路跑通：

```text
output/dpvo/trajectory_tum.txt
  + project code/DPVO/mall_dpvo.ply
  + resources/input_video.mp4
  -> output/route_A/static_map.png
  -> output/route_A/bev_tracking_route_A.mp4
```

只有这条链路稳定后，再引入 VGGT/KV 点云增强。
