# KV-Tracker 商场室内导航与 BEV 地图应用方案

> 把 KV-Tracker 接入本仓库现有的商场单目视觉导航流水线 (DPVO / ORB-SLAM3 / people_bev_tracker), 输出统一的"二维真实地图 + 相机轨迹 + 行人动态位置"的工程方案。

---

## 0. 现状概览

本仓库已有的模块:

| 模块 | 路径 | 角色 |
| :--- | :--- | :--- |
| ORB-SLAM3 wrapper | [`src/orbslam3_wrapper`](../../orbslam3_wrapper) | 双目/单目稀疏特征 SLAM |
| DPVO localization | [`src/dpvo_localization`](../../dpvo_localization), [`project code/DPVO`](../../../project%20code/DPVO) | 单目稀疏 patch VO |
| people_bev_tracker | [`src/people_bev_tracker`](../../people_bev_tracker) | YOLO-seg + BoT-SORT + BEV 行人定位 |
| KV-Tracker (本任务) | [`src/KV-tracker`](..), [`project code/KV-tracker`](../../../project%20code/KV-tracker) | π³ + KV-cache, 输出相机位姿 + 局部结构 + 置信度 |

KV-Tracker 在这套系统里扮演的角色 (路线 A/B/C, 见 §6):

* **A**: 给 people_bev_tracker 提供更稠密的 3D 结构作为"可通行区域/障碍 mask", 主轨迹仍由 DPVO 提供。
* **B**: 完全替代 DPVO, 由 KV-Tracker 直接给出相机位姿 + 局部结构 + confidence。
* **C**: DPVO / ORB-SLAM3 / KV-Tracker 三源对比, 取最稳的当主轨迹。

---

## 1. 总体路线 (端到端流程)

```
                            ┌──────────────────────────┐
                            │  resources/input_video   │
                            │   1920x1080, 29.4 FPS    │
                            └─────────────┬────────────┘
                                          │
            ┌─────────────────────────────┼─────────────────────────────┐
            │                             │                             │
            ▼                             ▼                             ▼
   ┌────────────────┐           ┌────────────────┐           ┌────────────────┐
   │ YOLO-seg +     │           │  DPVO 或       │           │  KV-Tracker    │
   │ BoT-SORT       │           │  ORB-SLAM3     │           │  (本任务)       │
   │ (动态行人)     │           │  (相机位姿)     │           │  → T_wc        │
   │ → track_id +   │           │  → trajectory   │           │  → local pts   │
   │   mask + bbox  │           │    _tum.txt     │           │  → confidence  │
   └───────┬────────┘           └────────┬───────┘           └────────┬───────┘
           │                             │                             │
           │ foot_pixel                  │ T_wc / scale                │ 稠密点图 / 关键帧
           │                             │                             │
           └──────────────┬──────────────┴──────────────┬──────────────┘
                          │                             │
                          ▼                             ▼
                ┌──────────────────────┐    ┌──────────────────────┐
                │  地面 / 相机系平面   │    │  KV-Tracker 局部结构 │
                │  与脚底射线相交      │    │  → 地面/障碍/结构点  │
                │  X_w = C_w + λ r_w   │    │                      │
                └──────────┬───────────┘    └──────────┬───────────┘
                           │                            │
                           └────────────┬───────────────┘
                                        │
                                        ▼
                         ┌────────────────────────────┐
                         │  BEV / 商场真实平面图       │
                         │  ├─ 相机轨迹                │
                         │  ├─ 行人动态位置 + track_id │
                         │  ├─ 障碍 / 可通行区域       │
                         │  └─ 与 CAD 平面图配准 (可选)│
                         └────────────────────────────┘
```

---

## 2. 相机轨迹投影到二维地图

KV-Tracker 输出每帧 $T_{wc} \in SE(3)$。相机光心:
$$
C_w = T_{wc}[:3, 3] \in \mathbb{R}^3.
$$

注意 KV-Tracker 的"世界系" = **第一个关键帧的相机系** (`pi3_inference()` 强制做了 $T'_{wc} = T_{wc}(0)^{-1} T_{wc}$)。

### 2.1 选 BEV 平面的两个轴

要先经过实验确认 KV-Tracker 输出世界轴的"上方向"是 $y$ 还是 $z$ 还是别的方向。

**情况 A**: 世界 $+Y$ 向上 → BEV 取 $(x, z)$:

$$
p_{\text{bev}} = \begin{bmatrix} x_w \\ z_w \end{bmatrix}.
$$

**情况 B**: 世界 $+Z$ 向上 → BEV 取 $(x, y)$:

$$
p_{\text{bev}} = \begin{bmatrix} x_w \\ y_w \end{bmatrix}.
$$

判定方法 (任选一种):

1. 看几帧相机 $C_w$ 的 $y$ 和 $z$ 哪个变化最大 (走廊行走主要在 $z$ 上变化, 那 $y$ 就是上下)。
2. 直接画 3D 轨迹 (`rerun` 或 open3d), 肉眼看哪个是 "高度轴"。
3. 调取 π³ camera_head 训练时的约定 — 通常是 `+Y down` (图像 y 向下 → 相机 +Y 向下 → 世界 +Y 向下, 因为第一帧 = identity)。本仓库实测就是 $+Y$ 向下, 所以 BEV 取 $(x, z)$ 是对的。

### 2.2 像素映射

设 BEV 画布宽 $W$ 高 $H$, 分辨率 $r$ (米/像素), 画布中心对应世界 $(o_x, o_y)$:

$$
\text{px} = \tfrac{W}{2} + \tfrac{p_{\text{bev}, x} - o_x}{r},
\qquad
\text{py} = \tfrac{H}{2} - \tfrac{p_{\text{bev}, y} - o_y}{r}.
$$

(注意 py 是减号, 因为画布 y 像素向下而世界轴向上。)

---

## 3. 行人脚底点投影 (复用 `people_bev_tracker` 数学)

### 3.1 脚底像素

每个行人 bbox $[x_1, y_1, x_2, y_2]$, mask $M \in \{0,1\}^{H \times W}$。优先用 mask 底部中位:

$$
v_f = \max \{v: M(v, u) = 1\},\qquad
u_f = \text{median}\{u : M(v, u) = 1,\; v \ge v_f - \alpha (v_f - v_{\min})\}.
$$

没有 mask 时退化为 bbox 底中点:

$$
u_f = \tfrac{x_1 + x_2}{2},\qquad v_f = y_2.
$$

### 3.2 像素 → 相机系射线

$$
\tilde{r}_c = K^{-1} \begin{bmatrix} u_f \\ v_f \\ 1 \end{bmatrix},\qquad
r_c = \tilde{r}_c / \|\tilde{r}_c\|.
$$

KV-Tracker 默认把图像 resize 到 $308 \times \sim$, 内参也按比例缩。务必用**缩放后的 $K'$**, 否则 $r_c$ 方向有偏。

### 3.3 与地面相交 (两种参数化, 见 [people_bev_tracker IMPLEMENTATION §3.3-3.4](../../people_bev_tracker/IMPLEMENTATION.md))

#### 世界系地面 (`mode=world`)

$$
n_w^\top X + d = 0.
$$

把射线转到世界系: $r_w = R_{wc} r_c$, $C_w = T_{wc}[:3, 3]$。

$$
\lambda = -\frac{n_w^\top C_w + d}{n_w^\top r_w},\qquad
X_w = C_w + \lambda r_w.
$$

#### 相机系地面 (`mode=camera`, 推荐)

相机俯仰 $\alpha$, 离地高度 $h$:

$$
g_c = (0, \cos\alpha, \sin\alpha)^\top,\qquad
g_c^\top X_c = h.
$$

$$
\lambda = \frac{h}{g_c^\top r_c},\qquad
X_c = \lambda r_c,\qquad
X_w = R_{wc} X_c + C_w.
$$

对 KV-Tracker 而言, 相机系地面同样推荐, 因为 KV-Tracker 世界轴和重力对齐没有保证。

---

## 4. 用 KV-Tracker 局部结构生成地面 / 障碍 / 结构点

KV-Tracker 非 `--cam_only` 模式下输出 $P \in \mathbb{R}^{N \times H \times W \times 3}$ 稠密点图, 加置信度 $C \in [0,1]^{N \times H \times W}$。

### 4.1 高质量点 → "可通行区域" 候选

筛选规则:

$$
\text{valid}(u, v) \iff C(u, v) > \tau \;\land\; M_\text{static}(u, v) = 1 \;\land\; |y_w - \bar y_\text{ground}| < \delta.
$$

* $\tau$: 置信度阈值, 默认 $1.15 \times 0.6 \bar C_{\text{first kf}}$。
* $M_\text{static}$: 静态语义 mask (排除行人、车等动态对象), 用 YOLO-seg 或 SAM2 得到。
* $\bar y_\text{ground}$: 地面平面的 $y$ 高度, 由 RANSAC 拟合得到。
* $\delta$: 高度容差 (e.g. 10 cm)。

### 4.2 RANSAC 拟合地面

在筛选过的稠密点云上跑:

```python
import open3d as o3d
plane_model, inliers = pcd.segment_plane(
    distance_threshold=0.02,   # 2 cm
    ransac_n=3,
    num_iterations=1000,
)
# plane_model = [a, b, c, d] 满足 a*x + b*y + c*z + d = 0
n_w = plane_model[:3] / np.linalg.norm(plane_model[:3])
d_w = plane_model[3] / np.linalg.norm(plane_model[:3])
```

得到的 $(n_w, d_w)$ 直接喂给行人脚底点投影的世界系地面公式 (§3.3 第一种)。比第一版 hand-tune 的 `camera_pitch_deg + camera_height` 更稳。

### 4.3 障碍 → "墙 / 柜台 / 货架"

把不属于地面平面的高置信度点投到 BEV, 做密度栅格:

1. 按 $r$ (米/像素) 的网格累计点数。
2. 高度 $|y - \bar y_\text{ground}| > h_\text{floor}$ (e.g. 30 cm) 的点算"障碍"。
3. 栅格中点数 $> N_\text{thresh}$ 的 cell 标为 occupancy。

得到的 occupancy 栅格 + 相机轨迹 + 行人位置就是商场 BEV 的全部。

---

## 5. 和行人检测/跟踪模块结合

### 5.1 静态/动态 mask 分离

```
原始帧
  ├─ YOLO-seg (classes=[0])  →  动态行人 mask M_dyn, track_id
  └─ 静态 mask M_static = (~M_dyn) ∩ 图像有效区
```

### 5.2 给 KV-Tracker 的 mask token

KV-Tracker 在 mapping 和 tracking 时支持 `tokens_mask`, 这是 patch-level 的 binary 张量。把行人 mask 下采样到 token 分辨率, 取反作为 `tokens_mask`:

```python
# 假设 patch 大小 14x14, resize_dim=308 -> patch_grid=22x29
patch_mask = downsample_to_patches(M_static, patch_size=14)
results = model(images_tensor, tokens_mask=patch_mask, ...)
```

效果: π³ attention 只在静态 patch 上计算, 行人不会污染 KV-cache, 也不会贡献错位的 3D 点。

### 5.3 行人轨迹和相机轨迹的时间对齐

KV-Tracker 是逐帧的, people_bev_tracker 也是逐帧的, 用相同的源帧 `frame_index` 对齐即可。`output/kv_track3r_repro/trajectory.json` 已经把 frame_index 和 timestamp 都写进去:

```json
{
  "poses": [
    {"frame_index": 0, "timestamp": 0.0, "T_wc": [...], ...},
    {"frame_index": 1, "timestamp": 0.034, "T_wc": [...], ...}
  ]
}
```

people_bev_tracker 的 `people_tracks.json` 也是按 frame_index 索引, 一对一 join 即可。

---

## 6. 路线 A / B / C 的取舍

### 路线 A: **DPVO 主轨迹 + KV-Tracker 提供局部结构**

* DPVO: `output/dpvo/trajectory_tum.txt` (已有, 7.5 分钟跑完)
* KV-Tracker: 只跑非 cam_only 模式, 用它的稠密点图和 confidence 做地面拟合 / 障碍。
* people_bev_tracker: pose 来自 DPVO, ground 来自 KV-Tracker (RANSAC), 行人投影更准。

优点:

* 轨迹仍由 DPVO 提供, 实时 + 稳定。
* KV-Tracker 不需要全程跑, 可以只在初始化时跑几次 mapping 拿稠密结构。
* 不用解决 KV-Tracker 长序列稳定性问题。

缺点:

* DPVO 和 KV-Tracker 的世界系不同, 需要 Sim(3) 对齐 (Umeyama 算法, 代码里 `umeyama_alignment` 已有)。

### 路线 B: **KV-Tracker 直接做相机位姿和局部结构**

* KV-Tracker: 全程 tracking, 输出 trajectory_tum.txt + 稠密点图 + confidence。
* people_bev_tracker: pose、ground、地图全用 KV-Tracker。

优点:

* 单模型一致, 数据流简洁。
* 可视化效果接近论文 demo (rerun)。

缺点 / 风险:

* 关键帧上限 20, 长视频 (3181 帧) 需要扩展淘汰策略。
* π³ 单目尺度, 没有米制。
* 显存压力大, KV-cache 多层 K/V 全驻留。
* 长序列稳定性未验证 (论文实验主要在 < 1 分钟序列)。

### 路线 C: **三源 (ORB-SLAM3 / DPVO / KV-Tracker) 对比**

* 同一段视频, 三套独立估出 trajectory_tum.txt。
* 用 evo 工具计算 ATE, 取最稳。
* KV-Tracker 输出的稠密结构和 confidence 作为辅助。

优点:

* 鲁棒性最强, 失效模式不重叠。
* 适合做学术对比 / 论文实验。

缺点:

* 工程复杂, 三套环境管理。
* 离线为主, 难以实时。

### 推荐

**第一版用路线 A**。理由: DPVO 已经跑通且稳定, KV-Tracker 替它做主轨迹的风险比"做地面拟合"高得多。等 KV-Tracker 在长序列上验证稳定再切到路线 B。

---

## 7. 第一版可落地的工程架构

按时间顺序分四阶段:

### 阶段 1: 复现 KV-Tracker (本任务已完成)

* `python src/KV-tracker/scripts/run_official_kv_tracker.py --cam-only --rerun --export`
* 输出: `output/kv_track3r_repro/trajectory.json + .npy + tum`

### 阶段 2: 接入 BEV 流水线 (用 KV-Tracker 当 pose 源)

新增脚本 `src/people_bev_tracker/scripts/run_with_kv_tracker_pose.sh`:

```bash
# 复用现有的 offline_pipeline.py
python src/people_bev_tracker/scripts/offline_pipeline.py \
  --video resources/input_video.mp4 \
  --calib config/KannalaBrandt8_1280x720.yaml \
  --pose  output/kv_track3r_repro/trajectory_tum.txt \
  --output-dir output/people_bev_kvtracker
```

* 不需要改 `offline_pipeline.py` 的逻辑, 只是换 `--pose` 参数。
* 注意 KV-Tracker TUM 的时间戳就是 frame_index/fps (我们 `output_converter.py` 已经按 `fps=29.417` 填好)。

### 阶段 3: 用 KV-Tracker 稠密结构拟合地面

新增 `src/KV-tracker/kv_track3r_app/ground_fit.py`:

```python
def fit_ground_from_kv_pcd(local_structure_ply, conf_threshold=0.5):
    import open3d as o3d
    pcd = o3d.io.read_point_cloud(local_structure_ply)
    plane_model, _ = pcd.segment_plane(
        distance_threshold=0.02, ransac_n=3, num_iterations=1000
    )
    return plane_model   # [a, b, c, d]
```

把拟合到的 $(n_w, d_w)$ 写回 `people_bev_tracker.yaml`:

```yaml
ground:
  mode: "world"
  normal: [0.0123, 0.9989, -0.0432]
  d: -0.1023
```

### 阶段 4: BEV 加上 KV-Tracker 障碍栅格

新增 `src/people_bev_tracker/people_bev_tracker/static_map.py`:

```python
def build_static_occupancy_from_kv_pcd(
    local_structure_ply,
    ground_plane,
    resolution=0.05,
    height_thresh=0.3,
):
    ...
    return occupancy_grid   # HxW uint8
```

在 `bev_canvas.draw()` 里把 occupancy_grid 当成背景层叠加 (灰色: 自由空间, 深色: 墙壁/障碍)。

最终输出 (路线 A 完整版):

```
output/mall_bev/
├── mall_bev_tracking.mp4         # 静态 BEV + 相机轨迹 + 行人动态点
├── mall_bev_tracking_clean.mp4   # 同上, 不画行人轨迹线
├── camera_path.geojson           # 相机轨迹, 可导入 QGIS / Mapbox
├── people_tracks.json            # 行人逐帧定位 (人/帧/世界坐标)
├── static_structure_points.json  # KV-Tracker 稠密静态点云 (高置信度)
├── ground_plane.json             # 地面参数 {normal, d, RMSE}
└── aligned_floorplan.png         # (可选) 与商场 CAD 配准后的真实平面图
```

---

## 8. 升级到 ROS2 实时系统的 topic 设计

### 8.1 节点拓扑

```
                ┌──────────────────────┐
                │  /camera/image_raw   │  (sensor_msgs/Image)
                └──────────┬───────────┘
                           │
       ┌───────────────────┼───────────────────┐
       │                   │                   │
       ▼                   ▼                   ▼
┌──────────────┐  ┌──────────────────┐  ┌──────────────────┐
│ yolo_tracker │  │ kv_tracker_node  │  │ dpvo_node        │
│ (Python)     │  │ (Python, GPU)    │  │ (C++/Python)     │
└──────┬───────┘  └─────┬────────────┘  └─────┬────────────┘
       │                │                     │
       ▼                ▼                     ▼
/people/detections   /tf (kv_tracker)    /tf (dpvo)
       │                │                     │
       └────────┬───────┴────────┬────────────┘
                ▼                ▼
        ┌──────────────────────────┐
        │  ground_projector_node   │
        │  (Python)                │
        └──────────┬───────────────┘
                   ▼
           /people/world_positions
           /camera/bev_pose
                   │
                   ▼
            ┌──────────────────┐
            │ bev_renderer_node│
            │ (Python)         │
            └──────────┬───────┘
                       ▼
              /bev/map_image (sensor_msgs/Image)
              /bev/people_markers (visualization_msgs/MarkerArray)
              /bev/camera_path (nav_msgs/Path)
```

### 8.2 Topic 详细设计

| Topic | 类型 | 发布者 | 说明 |
| :--- | :--- | :--- | :--- |
| `/camera/image_raw` | `sensor_msgs/Image` | camera_driver | 原始 RGB |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | camera_driver | K, distortion, frame_id="camera" |
| `/people/detections` | `vision_msgs/Detection2DArray` | yolo_tracker | bbox + track_id + (optional) mask via image header |
| `/people/masks` | `sensor_msgs/Image` | yolo_tracker | uint8, mask 编码 (每像素 = track_id, 0=背景) |
| `/tf` (camera_kv ← world_kv) | `tf2_msgs/TFMessage` | kv_tracker_node | KV-Tracker 输出 |
| `/kv_tracker/confidence` | `std_msgs/Float32` | kv_tracker_node | 每帧 mean confidence |
| `/kv_tracker/keyframes` | `visualization_msgs/MarkerArray` | kv_tracker_node | 关键帧 frustum |
| `/kv_tracker/structure` | `sensor_msgs/PointCloud2` | kv_tracker_node | 稠密静态点云 (~50ms latency) |
| `/tf` (camera_dpvo ← world_dpvo) | `tf2_msgs/TFMessage` | dpvo_node | DPVO 输出 |
| `/people/world_positions` | `geometry_msgs/PoseArray` | ground_projector_node | 每人世界 3D, header.frame_id="world_kv" |
| `/camera/bev_pose` | `geometry_msgs/PoseStamped` | ground_projector_node | 相机在 BEV 的位姿 (Y 拍平) |
| `/bev/map_image` | `sensor_msgs/Image` | bev_renderer_node | 渲染好的 BEV PNG |
| `/bev/people_markers` | `visualization_msgs/MarkerArray` | bev_renderer_node | 给 RViz 显示 |
| `/bev/camera_path` | `nav_msgs/Path` | bev_renderer_node | 累计相机轨迹 |

### 8.3 参数化 (rosparam / launch)

```yaml
# kv_tracker_node.yaml
kv_tracker:
  official_root: "/home/ros/ros2_orbslam3/project code/KV-tracker/kv_tracker-main"
  resize_dim: 308
  cam_only: true
  kf_auto: 50
  device: "cuda:0"

# ground_projector_node.yaml
ground_projector:
  pose_source: "kv_tracker"   # or "dpvo"
  ground_mode: "world"
  ground_fit_topic: "/kv_tracker/structure"   # 接 KV 点云做在线拟合
  bev_axes: ["x", "z"]

# bev_renderer_node.yaml
bev_renderer:
  resolution: 0.05            # m / px
  width: 1200
  height: 1200
  origin: [0.0, 0.0]
  trail_length: 200
  show_people_trails: false   # 干净版
```

### 8.4 实时性考虑

* KV-Tracker 在 RTX 3060+ 上跑 308×308 大约 15-27 FPS, 接 ROS2 不掉帧。
* yolo_tracker (YOLOv11-seg-n) 在同 GPU 上 ~30 FPS。
* 两个 GPU node 共用一张卡时, 建议把 KV-Tracker 限到 ~15 FPS (decimation), 让 YOLO 占主导。
* people 投影 + BEV 渲染都是 CPU 任务, < 10 ms。
* 端到端延迟目标 < 100 ms。

---

## 9. 单目尺度对齐 (老问题, 这里集中处理)

### 9.1 KV-Tracker / DPVO 都没有真实米制

三种获得尺度的常见办法:

#### A) 已知相机高度

实际 1.6 m (头戴) / 1.2 m (推车), DPVO 单位下相机不动地面静态时, 用一次 RANSAC 拟出 $h_\text{dpvo}$, 然后:

$$
s = \frac{h_\text{real}}{h_\text{dpvo}}.
$$

整条轨迹的 $t_{wc} \mapsto s \cdot t_{wc}$, 点云 $P \mapsto s \cdot P$, 旋转不变。

#### B) 商场地砖 / AprilTag / 已知 landmark

放一张已知尺寸地砖在画面里, 测画面中地砖边长对应的真实长度 $L_\text{real}$, 单位下 $L_\text{dpvo}$ → $s = L_\text{real}/L_\text{dpvo}$。

更鲁棒: 多个 AprilTag 用 PnP 算外参, 直接得到米制相机外参, Umeyama 把 DPVO/KV-Tracker 轨迹对齐过去。

#### C) IMU / 已有 SLAM 系统

* 接 IMU → VIO 自带尺度 (DPVO 默认无 IMU)。
* 或者用 ORB-SLAM3 双目 (已知 baseline) 出米制轨迹, 再 Umeyama 对齐单目 KV-Tracker 轨迹过去。

### 9.2 Umeyama 公式

给定两组点 $\{x_i\}, \{y_i\}, i=1\ldots N$, 求解 $s, R, t$ 使 $\sum_i \|s R x_i + t - y_i\|^2$ 最小:

$$
\mu_x = \tfrac{1}{N}\sum_i x_i,\quad \mu_y = \tfrac{1}{N}\sum_i y_i.
$$

$$
\Sigma_{xy} = \tfrac{1}{N}\sum_i (y_i - \mu_y)(x_i - \mu_x)^\top.
$$

SVD: $\Sigma_{xy} = U S V^\top$。

$$
R = U \cdot \text{diag}(1, \ldots, 1, \det(UV^\top)) \cdot V^\top.
$$

$$
s = \frac{\operatorname{tr}(S \cdot \text{diag}(1, \ldots, 1, \det(UV^\top)))}{\sum_i \|x_i - \mu_x\|^2}.
$$

$$
t = \mu_y - s R \mu_x.
$$

代码已经在 `kv_tracker/geometry.py:umeyama_alignment` 实现好。

---

## 10. 最终目录建议

```
output/mall_bev/
├── mall_bev_tracking.mp4           # 主输出: BEV + camera trail + people
├── mall_bev_tracking_clean.mp4     # 干净版
├── camera_path.geojson             # camera_path.json 也可, 给 QGIS 用
├── people_tracks.json              # 复用 people_bev_tracker 格式
├── static_structure_points.json    # KV-Tracker 高置信度静态点云
├── ground_plane.json               # {normal, d, RMSE, scale_to_real}
├── kv_track3r_repro/               # KV-Tracker 复现原文件 (不动)
│   ├── trajectory.npy / tum / json
│   ├── keyframe_poses.npy + .json
│   ├── confidence.json + .npy
│   ├── local_structure.npy + .ply
│   ├── runtime.csv
│   └── summary.md
└── aligned_floorplan.png           # (可选) 配准后的真实商场平面图
```

第一版可以省略 `aligned_floorplan.png` — 与 CAD 配准属于二期任务, 需要拿到商场图纸 + 几个 ground control points 做 2D ICP。

---

## 11. 三条命令速查

```bash
# (1) DPVO 主轨迹 (已完成)
cd "project code/DPVO" && python demo.py --imagedir ../../resources/input_video.mp4 \
  --calib calib/custom_mall.txt --name input_video_clean --stride 2 --save_trajectory
cp saved_trajectories/input_video_clean.txt ../../output/dpvo/trajectory_tum.txt

# (2) KV-Tracker 复现 + 导出 (本任务)
python src/KV-tracker/scripts/run_official_kv_tracker.py \
  --official-root "project code/KV-tracker/kv_tracker-main" \
  --config src/KV-tracker/config/mall_video.yaml \
  --cam-only --resize-dim 308 --export

# (3) people BEV (路线 A: DPVO pose + KV-Tracker 后续地面拟合)
python src/people_bev_tracker/scripts/offline_pipeline.py \
  --video resources/input_video.mp4 \
  --calib config/KannalaBrandt8_1280x720.yaml \
  --pose  output/dpvo/trajectory_tum.txt \
  --output-dir output/people_bev
```

---

## 12. 已知限制 / 第一版不做的事

| 项 | 第一版 | 二期 |
| :--- | :--- | :--- |
| 单目尺度 | 不做, BEV 单位 = DPVO 单位 | §9 中三种方法之一 |
| 地面拟合 | 用相机高度 + 俯仰固定假设 | KV-Tracker 点云 + RANSAC |
| 障碍栅格 | 不画 | KV-Tracker 高置信度静态点云生成 |
| ROS2 节点 | 不做 (离线脚本) | §8 完整 topic 设计 |
| CAD 平面图配准 | 不做 | 2D ICP + GCPs |
| 多楼层 | 不做 | NetVLAD/DBoW2 切换 cache + Z 轴分层 |
| 关键帧上限突破 | 用默认 20 | LRU + spatial coverage 评分淘汰 |

最终成果: `output/people_bev/bev_tracking_clean.mp4` (路线 A) 或 `output/mall_bev/mall_bev_tracking_clean.mp4` (路线 A + KV-Tracker 增强)。
