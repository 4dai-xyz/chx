# 路线 A 执行方案 — 眼镜视频 → 2D BEV 实时定位 + 行人可视化

> **场景**: 眼镜头戴相机走路视频, 无先验商场平面图。
> **目标**: 实时显示 (a) SLAM 点云投出的 2D 障碍地图, (b) 相机在这张地图上的定位, (c) 动态行人位置。
> **本次不做**: CAD 平面图配准 (二期); ROS2 节点 (二期); 米制尺度 (先用 DPVO 单位)。

---

## 0. 关键约束 (你的输入)

| 项 | 说明 |
| :--- | :--- |
| 拍摄设备 | 眼镜 (头戴单目相机) |
| 运动特征 | 走路时头部上下颠簸 + 俯仰角小幅变化 |
| SLAM 输出 | DPVO 稀疏 patch 点云 + KV-Tracker 稠密 Pi3 点云都有 |
| 先验地图 | **无** — 必须从 SLAM 点云现算 2D 地图 |
| 相机高度 | 假设**近似恒定** (眼镜高度 ~1.6m real, DPVO 单位约 0.1) |
| 实时性要求 | 能一边跑一边看 (< 100ms 延迟即可) |
| 输出选型 | 精度高的方法胜出, 两种点云都试 |

## 1. 核心技术难点

### 1.1 头部颠簸 (Y 轴周期抖动)

走路一步 ~0.6s, 头部上下位移振幅 5-10cm real (~0.005-0.01 DPVO 单位)。若不处理:
* 相机在 BEV 上会周期性"漂浮"
* 点云投影会因俯仰变化产生条带状伪影

处理方法 (3 选 1 或多方法融合, §5.2):

$$
y_{\text{flat}}(t) = \text{LPF}\big(\mathbf{n}_g^\top \mathbf{C}_w(t) + d_g\big) \quad \text{s.t.} \quad y_{\text{flat}} \equiv \bar h
$$

即: 用拟合出的地面法向 $\mathbf{n}_g$ 计算相机到地面的距离, 低通滤波成恒定的 $\bar h$。

### 1.2 俯仰角小幅变化 → 地面法向漂移

同一段视频里, 若人低头看地或抬头看店招, 地面法向在相机系中偏移几度。对策:
* **全局单一地面**: 用整段轨迹的点云一次性拟合, 不用逐帧地面
* 逐帧地面法向 = 全局法向 (固定, 不受当前帧俯仰影响)

### 1.3 长走廊 DPVO 漂移

DPVO 关闭 loop closure 时长序列会漂 (前文实测 3181 帧 ~ 100s 尾部 Y 出现 ~0.75 单位偏移)。处理:
* 短期用 DPVO trajectory (~30s 内漂移可接受)
* 长期若要精度, KV-Tracker 隐式 KV cache 有轻度闭环能力, 用它的 traj 做修正 (但成本高)
* **本次方案先只用 DPVO trajectory, 不做闭环校正**

### 1.4 单目无尺度

* BEV 上"1m 网格"实际上是"1 DPVO 单位网格" (~10m real, 视场景而定)
* 不影响拓扑正确性, 影响的是米制标注
* 二期用相机高度先验 (1.6m) 反推 scale, 现在先不做

## 2. 数据流

```
resources/input_video.mp4  (1920x1080, 29.4 fps, 3181 帧)
    │
    ├──── DPVO (已跑, 7.5 min GPU)
    │       ├── output/dpvo/trajectory_tum.txt          (1590 pose, stride=2)
    │       └── project code/DPVO/mall_dpvo.ply         (25K 稀疏 patch 点)
    │
    ├──── KV-Tracker (已跑, 20 min GPU)
    │       ├── output/kv_track3r_repro/traj.npy        (3180 pose)
    │       ├── output/kv_track3r_repro/pcd.npy         (80 KF × 168×294 = 3.9M 稠密点)
    │       └── output/kv_track3r_repro/kf_idx.npy      (KF ↔ 源帧 index)
    │
    └──── YOLO-seg + BoT-SORT (people_bev_tracker 已有)
            └── 每帧 track_id + mask + bbox

                        │
                        ▼
  ┌───────────────────────────────────────────────────────────┐
  │  阶段 1: 点云对齐 (KV → DPVO 世界系)                        │
  │    Sim(3) via Umeyama on shared frame_indices             │
  │    输出: kv_pcd_in_dpvoframe.npy                          │
  └───────────────────────────────────────────────────────────┘
                        │
                        ▼
  ┌───────────────────────────────────────────────────────────┐
  │  阶段 2: 全局地面拟合 (3 方法比较, 取最优)                    │
  │    (a) RANSAC on all points                               │
  │    (b) 稳健百分位 + PCA                                    │
  │    (c) 相机 Y 时序 low-pass 反推                           │
  │    输出: ground_plane.json {normal, d, RMSE, method}      │
  └───────────────────────────────────────────────────────────┘
                        │
                        ▼
  ┌───────────────────────────────────────────────────────────┐
  │  阶段 3: 相机轨迹平面化 (bounce removal)                    │
  │    输入: DPVO trajectory + ground_plane                    │
  │    输出: trajectory_flat.txt (Y 恒定)                      │
  └───────────────────────────────────────────────────────────┘
                        │
                        ▼
  ┌───────────────────────────────────────────────────────────┐
  │  阶段 4: 点云 → 2D occupancy grid                          │
  │    1. 变到地面对齐坐标 (ground normal → world +Y)          │
  │    2. 过滤离群 (percentile) + 高度过滤 (obstacle band)     │
  │    3. 密度栅格化 (2D histogram)                            │
  │    4. 形态学膨胀 + 阈值化 → occupied / free / unknown       │
  │    输出: occupancy_grid.png + occupancy_grid.json         │
  └───────────────────────────────────────────────────────────┘
                        │
                        ▼
  ┌───────────────────────────────────────────────────────────┐
  │  阶段 5: 集成 people_bev_tracker + 实时可视化                │
  │    ├─ 静态背景层: occupancy_grid                            │
  │    ├─ 中层: DPVO 相机轨迹 + 当前位姿                          │
  │    ├─ 前层: 动态行人 (track_id + bev_xy)                    │
  │    └─ 实时窗口: cv2.imshow 或 Rerun live                    │
  └───────────────────────────────────────────────────────────┘
                        │
                        ▼
     output/route_A/
     ├── kv_pcd_in_dpvoframe.npy
     ├── ground_plane.json
     ├── trajectory_flat.txt
     ├── occupancy_grid.png / .npy / .json
     ├── bev_tracking_route_A.mp4     (主输出)
     ├── bev_tracking_clean_route_A.mp4
     ├── people_tracks.json (复用 people_bev_tracker 格式)
     └── camera_trajectory_flat.json
```

## 3. 参考的开源方法

| 开源项目 | 我们借鉴的部分 | 不借鉴的部分 |
| :--- | :--- | :--- |
| **Nav2 `costmap_2d` obstacle layer** | 点云投影 2D 栅格 + 阈值化的思路 | ROS2 消息层 (本次是纯离线) |
| **RTAB-Map `--Grid/CellSize`** | 3D → 2D projection 的 API 设计 | RGB-D 传感器依赖 |
| **ETH `elevation_mapping`** | Height field + 形态学后处理 | GPU 加速 |
| **Cartographer 2D grid submap** | 频率图 + Gaussian smoothing | 2D lidar 输入 |
| **ANYbotics `grid_map`** | grid data structure | (只借思路, 不引依赖) |
| **hector_mapping visualization** | 灰白栅格 + 相机 icon 叠加 | scan matching |
| **ORB-SLAM3 `octomap_mapping`** | 单目稀疏点 → 2D | full SLAM 依赖 |
| **DROID-SLAM demo (Rerun)** | 实时 3D + 2D 联动展示 | 训练依赖 |

我们**不重造** 2D SLAM (Cartographer / Nav2 等太重), 就写一个轻量的 "点云 → 密度栅格" 转换器, 借它们的**参数选择方式** (阈值、resolution、膨胀核大小)。

## 4. 系统架构 & 新增文件

```
src/
├── people_bev_tracker/
│   ├── people_bev_tracker/
│   │   ├── static_map.py              # 新: 点云 → 2D occupancy
│   │   ├── ground_fit.py              # 新: 3 种地面拟合方法
│   │   ├── trajectory_flatten.py      # 新: 平面化 + 平滑
│   │   └── bev_canvas.py              # 改: 加静态背景层
│   ├── config/
│   │   └── route_A.yaml               # 新: 路线 A 专用配置
│   ├── scripts/
│   │   ├── build_route_A.py           # 新: 一键跑通阶段 1-4
│   │   ├── offline_pipeline_A.py      # 新: 阶段 5 集成入口
│   │   └── live_bev_view.py           # 新: cv2.imshow 实时窗口
│   └── docs/
│       └── 01_RouteA_初版执行方案.md  # 本文档
│
└── KV-tracker/
    └── kv_track3r_app/
        └── sim3_alignment.py          # 新: Umeyama KV → DPVO
```

## 5. 核心算法

### 5.1 Sim(3) 对齐 (KV-Tracker → DPVO)

给定两组配对相机位置 $\{\mathbf{p}_i^\text{kv}\}, \{\mathbf{p}_i^\text{dpvo}\}, i=1\ldots N$ (按 frame_index 匹配):

$$
\min_{s, R, t} \sum_i \| s R \mathbf{p}_i^\text{kv} + t - \mathbf{p}_i^\text{dpvo} \|^2
$$

Umeyama 闭式解:
$$
\mu_a = \tfrac{1}{N}\sum \mathbf{p}_i^\text{kv}, \quad \mu_b = \tfrac{1}{N}\sum \mathbf{p}_i^\text{dpvo}
$$
$$
\Sigma = \tfrac{1}{N}\sum (\mathbf{p}_i^\text{dpvo}-\mu_b)(\mathbf{p}_i^\text{kv}-\mu_a)^\top = U D V^\top
$$
$$
R = U\, \text{diag}(1,1,\det(UV^\top))\, V^\top, \quad
s = \frac{\text{tr}(D S)}{\sigma_a^2}, \quad
t = \mu_b - s R \mu_a
$$

代码里 `kv_tracker/geometry.py:umeyama_alignment` 已有实现, 直接调。

**帧匹配约定** (关键):
* DPVO stride=2 → DPVO frame_i ↔ 源视频 frame 2i
* KV-Tracker 逐帧输出 → KV frame_i ↔ 源视频 frame_i
* 匹配: KV frame 2j ↔ DPVO frame_j (对齐源帧)

**点云变换** (对齐):
$$
\mathbf{X}_\text{dpvoframe} = s R \mathbf{X}_\text{kvframe} + t
$$

### 5.2 全局地面拟合 (3 种方法)

#### 方法 A: RANSAC on all points

```python
plane, inliers = pcd.segment_plane(
    distance_threshold=0.01,   # DPVO 单位 (~10cm real)
    ransac_n=3,
    num_iterations=2000,
)
# plane = [a,b,c,d] s.t. ax+by+cz+d=0
n_g = plane[:3] / np.linalg.norm(plane[:3])
d_g = plane[3] / np.linalg.norm(plane[:3])
```

要求: $\mathbf{n}_g$ 与相机 "近似向下" 方向 (先验 `[0, 1, 0]`) 内积 > 0.85; 否则法向翻转。

#### 方法 B: 稳健百分位 + PCA

只保留高度较低的点 (即已经处于地面附近的候选):

$$
\text{floor candidates} = \{ \mathbf{p} : y(\mathbf{p}) \le y_{20\%} \}
$$

对候选做 PCA, 最小奇异值对应的方向就是地面法向:

$$
C = \tfrac{1}{|\mathcal{F}|} \sum_{\mathbf{p}\in\mathcal{F}} (\mathbf{p}-\bar{\mathbf{p}})(\mathbf{p}-\bar{\mathbf{p}})^\top
$$
$$
C = U \Lambda U^\top, \quad \mathbf{n}_g = U[:, \arg\min \Lambda]
$$

对头戴相机稳: 因为"地面附近的点"筛选剔除了天花板和杂物。

#### 方法 C: 相机 Y 时序 low-pass

不用点云, 只用轨迹:

1. 假设相机在时序上 Y 位置只有周期性颠簸, 无长期漂移。
2. LPF 滤 `trajectory[:, Y]` 得到 $\bar y(t)$, 认为 $\bar y(t) \equiv c$。
3. 地面法向 = 世界 +Y (先验), $d_g = c + h$ (相机到地面固定距离 $h$)。

最简单最鲁棒, 但**依赖 DPVO 世界轴大致重力对齐**。若相机第一帧就是低头/抬头 15°, 世界 Y 会带这个偏移, C 方法不准。

### 5.3 三方法对比指标

对每种方法计算:

* **inlier ratio**: 距离 < 阈值的点占比
* **inlier RMSE**: $\sqrt{\tfrac{1}{|I|}\sum_{i\in I} (\mathbf{n}_g^\top \mathbf{p}_i + d_g)^2}$
* **法向与相机初始 +Y 的角度**: 应 $< 30°$
* **可视检查**: 把 fit 的平面渲染到 open3d 里肉眼看

取 inlier ratio 最高**且**法向合理的方法。写进 `ground_plane.json`。

### 5.4 相机轨迹平面化

拟合出 $(\mathbf{n}_g, d_g)$ 后, 相机中心到地面的**带符号距离**:

$$
h(t) = \mathbf{n}_g^\top \mathbf{C}_w(t) + d_g
$$

平面化 = 强制 $h(t) \equiv \bar h$:

$$
\mathbf{C}_w^{\text{flat}}(t) = \mathbf{C}_w(t) + (\bar h - h(t)) \mathbf{n}_g
$$

其中 $\bar h = \text{median}\{h(t)\}$ (稳健均值)。

姿态 $R_{wc}$ 保留原样 (不改朝向)。

低通滤波变体 (更平滑):
$$
h_\text{smooth}(t) = \text{LPF}_{f_c=0.3\text{Hz}}(h(t))
$$
$$
\mathbf{C}_w^{\text{flat}}(t) = \mathbf{C}_w(t) + (h_\text{smooth}(t) - h(t)) \mathbf{n}_g
$$

后者保留长期地形变化 (二楼→一楼), 只滤掉步态颠簸。

### 5.5 点云 → 2D occupancy grid

#### Step 1: 变到地面对齐系

构造旋转 $R_\text{align}$ 使 $R_\text{align} \mathbf{n}_g = (0, 1, 0)^\top$ (世界 +Y 与地面法向对齐):

```python
def align_rot(n_g):
    y = np.array([0., 1., 0.])
    axis = np.cross(n_g, y)
    c = n_g @ y
    if abs(c) > 0.999:
        return np.eye(3)   # 已经对齐
    axis /= np.linalg.norm(axis)
    theta = np.arccos(c)
    K = np.array([[0,-axis[2],axis[1]],[axis[2],0,-axis[0]],[-axis[1],axis[0],0]])
    return np.eye(3) + np.sin(theta)*K + (1-np.cos(theta))*(K@K)   # Rodrigues
```

对所有点和相机位置应用 $R_\text{align}$。

#### Step 2: 过滤

1. 稳健范围 (1-99 percentile) 剔离群
2. 高度过滤: 只保留 $h_\text{floor} < y < h_\text{ceiling}$ 的点 (障碍带), 例如 DPVO 单位 [0.02, 0.35], 对应真实 [0.3m, 3m]

#### Step 3: 密度栅格

BEV 网格 (x, z 两轴, y 已成地面法向):

$$
i = \lfloor \tfrac{x - x_{\min}}{r} \rfloor, \quad
j = \lfloor \tfrac{z - z_{\min}}{r} \rfloor
$$

`count[i, j] = ` 落到该 cell 的点数。

#### Step 4: 阈值化 + 后处理

1. `occupied = count > τ_occ` (`τ_occ = 5` 起步)
2. `free = 相机路径附近但未 occupied` (投影相机轨迹到栅格, 沿轨迹 buffer)
3. `unknown = 剩下的`
4. 形态学: `occupied = cv2.dilate(occupied, kernel=3)` 让墙"厚"一点
5. 可选高斯平滑 → cost map (0=free, 255=occupied)

#### Step 5: 渲染

用 3 色 (灰 = unknown, 白 = free, 深灰 = occupied) 生成 uint8 图 → 传给 `bev_canvas` 当背景。

### 5.6 集成 people_bev_tracker

修 `bev_canvas.py`:

```python
class BEVCanvas:
    def __init__(self, ..., static_layer: Optional[np.ndarray] = None):
        self.static_layer = static_layer   # (H, W, 3) uint8 or None
        self._base = self._make_base()

    def _make_base(self):
        if self.static_layer is not None:
            img = self.static_layer.copy()
        else:
            img = np.full((self.H, self.W, 3), self.bg, dtype=np.uint8)
        # 原有网格线绘制
        ...
```

`offline_pipeline_A.py` 相比原版 `offline_pipeline.py`:

1. 启动时加载 `occupancy_grid.png` 作为 `static_layer`
2. 加载 `trajectory_flat.txt` (平面化后) 替代 `output/dpvo/trajectory_tum.txt`
3. 主循环与原版一致 (YOLO → foot → project → filter → draw)
4. 加 `cv2.imshow` 实时窗口 (可选 --live)

### 5.7 实时可视化 (阶段 5.5)

两个层面:

#### 简易实时 (推荐入门)

用 `cv2.imshow` 在离线 pipeline 每一帧都显示 BEV:

```python
cv2.imshow("BEV Live", bev_frame)
if cv2.waitKey(1) & 0xFF == ord('q'): break
```

优势: 0 依赖, 立刻能看。劣势: 只能看当前 BEV, 不能拖时间轴。

#### 进阶: Rerun 联动

复用现有的 KV-Tracker rerun wrapper, 额外 log:

* `bev/occupancy` (`rr.Image`) — 静态栅格
* `bev/camera` (`rr.Points2D`) — 当前相机位置
* `bev/people/id_XX` (`rr.Points2D`) — 每个 track_id 一条

用户可以 3D + 2D 同屏, 拖时间轴回放。

## 6. 实现步骤 (按依赖顺序)

### Step 1 — Sim(3) 对齐 (KV → DPVO)

**文件**: `src/KV-tracker/kv_track3r_app/sim3_alignment.py`

**输入**:
- `output/kv_track3r_repro/traj.npy` (3180×4×4)
- `output/dpvo/trajectory_tum.txt` (1590×8)
- `output/kv_track3r_repro/pcd.npy` (80 KF × H × W × 3)

**输出**:
- `output/route_A/sim3.json` — `{s, R (3×3), t (3,), ate, inliers}`
- `output/route_A/kv_pcd_in_dpvoframe.npy` — 变换后的点云 (concat 展平成 N×3)

**关键步骤**:
1. Load KV traj, DPVO traj
2. 提取共同 frame_index 对: DPVO 的 frame_i (stride=2, 对应源帧 2i) ↔ KV 的 frame_2i
3. 位置对: `dpvo_pos[i] = tj_dpvo[i, 1:4]`, `kv_pos[i] = kv_traj[2i, :3, 3]`
4. `s, R, t = umeyama(kv_pos.T, dpvo_pos.T, with_scale=True)`
5. 应用到点云: `pcd_new = s * (pcd @ R.T) + t`
6. 计算 ATE 检验对齐质量
7. 保存 sim3.json + 变换后点云

**测试**: ATE < 0.05 DPVO 单位 (约 0.5m real) 认为 OK。

### Step 2 — 点云汇总 (两源合并)

**文件**: `src/people_bev_tracker/scripts/collect_pointclouds.py`

**输入**:
- DPVO PLY (直接读)
- KV 变换后 (Step 1 输出)

**输出**:
- `output/route_A/pointcloud_dpvo.npy` (25K, 稳健过滤后)
- `output/route_A/pointcloud_kv.npy` (~3.9M, 稳健过滤后)
- `output/route_A/pointcloud_merged.npy` (合并)

各源在下一步单独测试, 最后合并测试。

### Step 3 — 地面拟合 (3 方法对比)

**文件**: `src/people_bev_tracker/people_bev_tracker/ground_fit.py`

**输入**: 任一 pointcloud .npy + DPVO trajectory

**输出**: `output/route_A/ground_plane_<method>.json`

**输出对比表**:

| 方法 | 点云源 | inlier ratio | RMSE | 法向 vs +Y 夹角 |
| :--- | :--- | :--- | :--- | :--- |
| RANSAC | DPVO | | | |
| RANSAC | KV | | | |
| RANSAC | merged | | | |
| PCA on bottom 20% | DPVO | | | |
| PCA on bottom 20% | KV | | | |
| PCA on bottom 20% | merged | | | |
| Camera Y LPF | (只用轨迹) | | | |

一份对比表, 手动挑最优 (自动挑也可, 优先 inlier ratio > 0.6 且 RMSE < 0.02 且法向合理)。

### Step 4 — 相机轨迹平面化

**文件**: `src/people_bev_tracker/people_bev_tracker/trajectory_flatten.py`

**输入**: `output/dpvo/trajectory_tum.txt` + 选出的地面

**输出**: `output/route_A/trajectory_flat.txt` (TUM 格式, Y 已平面化)

**验收**: 平面化后 $h(t) = \mathbf{n}_g^\top \mathbf{C}^\text{flat}(t) + d_g$ 应为常数 $\bar h$; 绘制 $h(t)$ 曲线检查。

### Step 5 — 点云 → occupancy grid

**文件**: `src/people_bev_tracker/people_bev_tracker/static_map.py`

**输入**: pointcloud .npy + ground_plane.json

**输出**:
- `output/route_A/occupancy_grid.png` (uint8, 灰白)
- `output/route_A/occupancy_grid.npy` (float, 0-1 cost)
- `output/route_A/occupancy_grid.json` (metadata: resolution, origin, extent)

**参数**:
- `resolution_m_per_px`: 0.004 (DPVO 单位, 沿用 people_bev_tracker.yaml)
- `height_range`: [0.02, 0.35] (DPVO 单位, 障碍带)
- `count_thresh`: 5 (每 cell 最少点数)
- `dilate_kernel`: 3
- `gaussian_blur_sigma`: 1.0

**参数扫描脚本**: `scripts/tune_occupancy_params.py`, 输出 5-10 张不同参数的 png, 手挑最好看。

### Step 6 — 集成 people_bev_tracker

**改文件**:
- `src/people_bev_tracker/people_bev_tracker/bev_canvas.py` — 加 `static_layer`
- `src/people_bev_tracker/scripts/offline_pipeline_A.py` — 新脚本 (基于 `offline_pipeline.py`)

**新 CLI**:

```bash
python src/people_bev_tracker/scripts/offline_pipeline_A.py \
  --video     resources/input_video.mp4 \
  --calib     config/KannalaBrandt8_1280x720.yaml \
  --pose      output/route_A/trajectory_flat.txt \
  --static-map output/route_A/occupancy_grid.png \
  --static-map-meta output/route_A/occupancy_grid.json \
  --ground-plane output/route_A/ground_plane_final.json \
  --output-dir output/route_A \
  --live       # 可选: cv2.imshow 实时窗口
```

**输出**:
- `output/route_A/bev_tracking_route_A.mp4` — 主输出 (带静态图 + 相机 + 行人)
- `output/route_A/bev_tracking_clean_route_A.mp4` — 无行人轨迹线的版本
- `output/route_A/debug_overlay_route_A.mp4`
- `output/route_A/people_tracks_route_A.json`
- `output/route_A/camera_trajectory_route_A.json`

### Step 7 — 实时可视化 (可选)

**文件**: `src/people_bev_tracker/scripts/live_bev_view.py`

`--live` 打开 `cv2.imshow("BEV Live", ...)`, 每帧 waitKey(1), 键盘 `q` 退出。

进阶: 出个 Rerun 版本 log 到 2D SpaceView。

## 7. 参数默认值 (初始)

```yaml
# src/people_bev_tracker/config/route_A.yaml
pose:
  source: "flat"
  tum_path: "output/route_A/trajectory_flat.txt"

pointcloud:
  primary_source: "kv"       # 或 "dpvo" 或 "merged"
  kv_npy: "output/route_A/pointcloud_kv.npy"
  dpvo_npy: "output/route_A/pointcloud_dpvo.npy"
  outlier_percentile: [1, 99]

ground:
  method: "auto"             # RANSAC / PCA / camera_LPF / auto (自动挑最好)
  distance_threshold: 0.01
  ransac_iterations: 2000

flatten:
  mode: "constant"           # 或 "lpf"
  lpf_cutoff_hz: 0.3

static_map:
  resolution_m_per_px: 0.004
  height_range: [0.02, 0.35]
  count_thresh: 5
  dilate_kernel: 3
  gaussian_blur_sigma: 1.0
  occupied_color: [80, 80, 80]
  free_color: [200, 200, 200]
  unknown_color: [40, 40, 40]

bev:
  width_px: 1200
  height_px: 1200
  origin_world: [0.0, 1.5]
  grid_step_m: 0.5
  trail_length: 80

output:
  dir: "output/route_A"
  live_window: false
```

## 8. 验收标准

### 阶段 1 (Sim3) 验收
- ATE (KV vs DPVO after Sim3) < 0.05 DPVO 单位
- s (scale) ∈ [0.5, 2.0] (说明 KV 和 DPVO 尺度差不多)
- 变换后 KV 点云 X/Y/Z 范围与 DPVO 点云同数量级

### 阶段 2 (地面) 验收
- inlier ratio > 0.6
- inlier RMSE < 0.02 (~20cm real)
- 法向与相机初始 +Y 夹角 < 30°
- 可视: rerun 里点云和地面平面在一起, 平面确实"贴"在低处点上

### 阶段 3 (轨迹平面化) 验收
- 平面化后 $h(t)$ 标准差 < 0.005 DPVO 单位 (~5cm real)
- 曲线肉眼看不出 1Hz 抖动

### 阶段 4 (occupancy) 验收
- 走廊两侧墙壁在 BEV 上明显 (纵向条带)
- 相机走过的路径在栅格里是 free (未被误标 occupied)
- 打印 statistics: %occupied, %free, %unknown

### 阶段 5 (集成) 验收
- `bev_tracking_route_A.mp4` 每帧包含:
  - 灰白栅格背景
  - 白色相机轨迹 + 当前位姿 (箭头)
  - 彩色行人点 + track_id
- `pose hit / miss` 3181/0
- projection ok/fail > 99%
- FPS > 15 (端到端离线渲染)

## 9. 风险和降级方案

| 风险 | 触发条件 | 降级 |
| :--- | :--- | :--- |
| Sim3 对齐失败 | ATE > 0.1 或 s ∉ [0.3, 3.0] | 只用 DPVO 点云, 跳过 KV |
| 地面拟合无解 | 3 方法 inlier ratio 都 < 0.3 | fallback 到 camera-frame ground (people_bev_tracker 现有配置) |
| 长走廊 DPVO 漂移 > 1 单位 | 视觉判断 | 截前 1500 帧, 只跑短段 |
| 稠密点云 OOM | KV pcd.npy 3.9M 点 一次性加载 | 分块加载 + block-wise 栅格化 |
| occupancy 全黑或全白 | 参数不对 | `tune_occupancy_params.py` 扫参数 |
| 实时窗口卡 | cv2.imshow FPS < 5 | 每 5 帧 imshow 一次 |

## 10. 时间估算

| 步骤 | 我实现 | 你审 & 测试 |
| :--- | :--- | :--- |
| Sim3 对齐 | 1-2h | 20min |
| 点云汇总 | 30min | 10min |
| 地面拟合 (3 方法 + 对比) | 2-3h | 30min |
| 轨迹平面化 | 1h | 10min |
| static_map (含调参脚本) | 2-3h | 1h 调参 |
| 集成 offline_pipeline_A | 2h | 30min |
| 实时窗口 | 30min | 10min |
| 文档更新 | 1h | 10min |
| **合计** | **~13h 编码** | **~3h 审+测** |

我这边可以一次串完; 你审文档 + 中间关键节点测一下就行。

## 11. 你要决策的点 (改文档前)

1. **点云主源**: 三选一 (`dpvo` / `kv` / `merged`) 作为初始默认, 剩余作对照。默认建议 `merged`。
2. **地面拟合方法**: `auto` 自动选 / 手动指定 (`RANSAC` / `PCA` / `camera_LPF`)。默认建议 `auto`。
3. **平面化模式**: `constant` (Y 恒定, 完全去除颠簸) vs `lpf` (LPF, 保留长期地形变化)。默认建议 `constant` (你说过默认在一个高度平面)。
4. **实时窗口**: 要不要一开始就集成 cv2.imshow? 我倾向先出 mp4, 实时窗口作可选 flag。
5. **BEV 分辨率**: 沿用 `0.004 DPVO/px` (画布 4.8×4.8 DPVO 单位)? 或者你想让分辨率随点云范围自适应?
6. **占据栅格颜色**: 深灰 = 障碍 / 白 = 通行 / 黑 = 未知, 可以吗? 还是你想灰度反转?
7. **KV-Tracker 稠密点云要不要重新跑一份**: 现在有 80 KF 的, resize_dim=224。 要不要重跑一份更高清 (resize_dim=308)?
8. **是否输出米制 BEV**: 现在标"1 DPVO 单位" 而不是 "1m"。要不要用相机高度先验反推 scale, 输出真米制? (二期你说过, 但可以现在顺便加个选项。)

---

改完这份文档后告诉我, 我按最新版本按 Step 1-7 顺序实现。
