# route_A 执行报告 (给用户 / 后来读代码的人)

> 本文档回答 4 个问题:
> 1. 现在这套工作流程是什么?
> 2. 用到的函数/文件架构 + 原理?
> 3. 我要是想跑, 输入什么命令?
> 4. 后续优化方向?
> 5. 想完全搞懂这套代码, 学习路径是什么?

---

## 1. 当前工作流程 (一句话总结)

**DPVO 提供相机轨迹 → 用点云拟合出地面平面 → 把 DPVO 轨迹沿地面法向 "压平"（去头部颠簸）→ 把点云投到 2D 栅格得静态地图 → YOLO+BoT-SORT 检测行人, 用相机-地面模型投到同一 BEV → 输出 MP4/JSON**。

### 数据流全景

```
resources/input_video.mp4  ─┬─→ (已跑) DPVO ─→ output/dpvo/trajectory_tum.txt (1590 pose)
                            │                    └─ (备选) project code/DPVO/mall_dpvo.ply
                            │
                            ├─→ (已跑) VGGT/KV ─→ output/vggt_aligned_full_run/.../aligned_full_scene.ply (401K 点)
                            │                    └─ output/kv_track3r_repro/pcd.npy (KV, V2 用)
                            │
                            └────────────────────────────────────────────┐
                                                                          │ (在 pipeline_A 里实时用)
                             ┌───────────────────────────────────────────┘
                             │
                             ▼
                     ┌──────────────────────┐
                     │  build_route_A.py    │  ← 阶段 1-4
                     │  ------------------  │
                     │  1. 读点云 + 过滤    │
                     │  2. 3 方法拟合地面   │
                     │  3. 相机 h 定符号    │
                     │  4. 轨迹平面化       │
                     │  5. 点云 → BEV 栅格  │
                     └──────────┬───────────┘
                                │
                                ▼
                  output/route_A/
                  ├── ground_plane_final.json
                  ├── trajectory_flat.txt
                  ├── static_map.npy / .png / meta.json
                  └── route_A_build_report.md
                                │
                                ▼
                     ┌──────────────────────┐
                     │ offline_pipeline_A.py│  ← 阶段 5
                     │  ------------------  │
                     │  加载 static_map      │
                     │  加载 trajectory_flat │
                     │  逐帧 YOLO+BoT-SORT   │
                     │  相机-地面投影行人    │
                     │  EMA 平滑             │
                     │  BEV 绘制 + MP4/JSON  │
                     └──────────┬───────────┘
                                │
                                ▼
                  output/route_A/
                  ├── bev_tracking_route_A.mp4          (**主输出**)
                  ├── bev_tracking_clean_route_A.mp4
                  ├── debug_overlay_route_A.mp4
                  ├── people_tracks_route_A.json
                  ├── camera_trajectory_route_A.json
                  └── validation_summary.json
```

### 关键设计决策

| 决策 | 选择 | 理由 |
| :--- | :--- | :--- |
| 主轨迹源 | **DPVO** (不用 KV-Tracker) | 用户实测 DPVO 更稳; V1 只走 DPVO |
| 静态点云源 | **VGGT aligned** (不用 DPVO PLY) | DPVO PLY 尺度杂散, 地面拟合失败; VGGT 稠密且和 DPVO trajectory 同世界系 |
| 地面拟合 | RANSAC + PCA + fallback, 加权综合分选 | 单一 RANSAC 容易把墙当地板 (VGGT 场景), PCA 对 bottom-percentile 更稳 |
| 头部颠簸处理 | **轨迹平面化** (`constant` 模式) | 假设"运动过程都在同一高度" 是任务书明确要求; 只改平移, 不改姿态 |
| 行人投影 | **相机系地面** (`h=0.1, pitch=15°`), fall back to world plane | 中/远距离行人只露上半身, "脚底像素"在图像上半, world plane 会 lam<0 |
| free 语义 | 只从相机轨迹 buffer 推 | 保守: 未观测区域标 unknown, 而不是当作 free |
| 行人不写静态图 | 严格区分 people layer vs static | 否则人走过的位置会被误标为障碍 |

---

## 2. 文件架构 + 原理

### 2.1 文件树 (只列 route_A 相关)

```
src/people_bev_tracker/
├── config/
│   ├── people_bev_tracker.yaml           (旧版, baseline)
│   └── route_A.yaml                      (新, 本任务默认)
│
├── people_bev_tracker/                   (Python 库)
│   ├── __init__.py
│   ├── types.py                          数据类 (CameraPose / TrackedPerson / PersonWorldState)
│   ├── camera_model.py                   相机内参 K, 像素→射线
│   ├── pose_io.py                        TUM 读, 最近邻 pose 查询
│   ├── footpoint.py                      mask/bbox → 脚底像素 u,v
│   ├── ground_projection.py              像素射线 vs 世界平面 / 相机平面
│   ├── state_filter.py                   EMA + 速度门限
│   ├── bev_canvas.py                     BEV 画布 (加了 static_layer 支持)
│   ├── io_utils.py                       config yaml, video reader/writer, JSON
│   ├── person_yolo_tracker.py            Ultralytics YOLO-seg + BoT-SORT 封装
│   │
│   ├── pointcloud_io.py         [NEW]    PLY/NPY 读, 稳健过滤, 轨迹近邻过滤
│   ├── ground_fit.py            [NEW]    RANSAC / bottom-PCA / camera fallback 三方法 + 打分
│   ├── trajectory_flatten.py    [NEW]    constant / lpf 平面化 TUM
│   └── static_map.py            [NEW]    R_align + 高度过滤 + 密度栅格 + 后处理
│
├── scripts/
│   ├── inspect_video.py                  (旧) 打印视频元信息
│   ├── offline_pipeline.py               (旧, baseline)
│   ├── render_bev_from_json.py           (旧, 快速重渲染)
│   │
│   ├── build_route_A.py         [NEW]    阶段 1-4 一键构建
│   ├── offline_pipeline_A.py    [NEW]    阶段 5 (主流水线) 加静态图 + R_align
│   └── tune_static_map_params.py [NEW]   扫参数生成多张候选 png
│
├── docs/
│   ├── 01_RouteA_初版执行方案.md
│   ├── 02_RouteA_DPVO主轨迹执行任务书.md
│   ├── 03_RouteA_方案评估与路线选择报告.md
│   ├── 04_RouteA_V1执行总结与代码说明.md [本文档]
│   └── 05_RouteA_V2静态地图优化执行方案_最新版.md
│
└── IMPLEMENTATION.md                     (基础实现说明, baseline 的)
```

### 2.2 关键函数逐一说明

以下按调用顺序列出。

#### `pointcloud_io.load_points(path, max_points=0)`

* 支持 `.ply` (二进制/ASCII, 用 Open3D) 和 `.npy` (含 KV-Tracker 那种 `(K,H,W,3)` 展平)
* 剔除 NaN/Inf
* `max_points > 0` 随机降采样

#### `pointcloud_io.robust_filter_points(pts, percentile=(1, 99))`

* 每维按 percentile 剔除极端离群
* SLAM 点云通常有 <5% 极端 outlier, 用 (5, 95) 更狠

#### `pointcloud_io.trajectory_proximity_filter(pts, traj, max_ratio=10, min_radius=1)`

* 距离 trajectory centroid 超过 `max(min_radius, max_ratio * traj_extent)` 的点删掉
* 对 DPVO PLY 里"暴走"的 outlier 特别有效

#### `ground_fit.fit_ground_ransac(pts, cfg)`

* Open3D `segment_plane`
* 阈值自适应: `_adaptive_ransac_thresh` 会根据点云 IQR 的均值调大 (`3% * mean(IQR)`)
* 返回 `{normal, d, inlier_ratio, rmse, angle_vs_axis_hint_deg}`

#### `ground_fit.fit_ground_bottom_pca(pts, cfg)`

* 保留 axis_hint 方向"底部" `bottom_percentile=20%` 的点
* PCA (SVD 分解协方差矩阵), 最小奇异值方向 = 平面法向
* $C = \sum (p - \bar p)(p - \bar p)^T / N$, $C = U \Lambda U^T$, $\mathbf{n}_g = U[:, \arg\min\Lambda]$
* 对稠密点云 (VGGT/KV) 比 RANSAC 稳: 因为墙面积可能大于地板, RANSAC 会误认

#### `ground_fit.choose_best_ground(candidates, cfg)`

* 综合分: `score = inlier_ratio - 0.5 * (angle / 90) - 0.5 * min(rmse, 1)`
* 硬约束: `inlier_ratio >= 0.02` && `angle <= 35°`
* 都不合格 → camera_fallback

#### `trajectory_flatten.flatten_trajectory(tum_in, tum_out, ground, mode)`

* 相机中心 $\mathbf{C}(t)$, 地面 $\mathbf{n} \cdot X + d = 0$
* 相机到地面高度 $h(t) = \mathbf{n} \cdot \mathbf{C}(t) + d$
* **constant 模式**: $h_\text{ref} = \text{median}(h)$; $\mathbf{C}^\text{flat}(t) = \mathbf{C}(t) + (h_\text{ref} - h(t)) \mathbf{n}$
* **lpf 模式**: 用一阶 RC LPF (前向 + 反向消相位) 平滑 $h$
* **只改平移**, 姿态 $R_{wc}$ 保留原样

#### `build_route_A.py` (脚本)

流程:

1. 加载点云 → percentile filter → proximity filter
2. 加载 DPVO trajectory
3. `fit_ground_all_methods` 跑 3 方法, `choose_best_ground` 选一
4. **检测相机 h(t) 符号**, 若均值 < 0, 翻转平面 $(\mathbf{n}, d) \to (-\mathbf{n}, -d)$
5. `flatten_trajectory` 输出 trajectory_flat.txt
6. **auto_obstacle_height_range**: `[5%, 95%] × camera_h_median` (相机到地板到头顶范围内)
7. **auto_origin**: 让 canvas 中心对准 R_align 后的 trajectory 中心
8. `build_static_map` + `save_static_map`
9. 写 `route_A_build_report.md`

#### `static_map.build_static_map(pts, traj, ground, cfg)` — **核心**

流程:

1. **R_align** 使地面法向 → 世界 +Y: 用 Rodrigues 公式解 $R$ s.t. $R \mathbf{n} = \hat{y}$
2. 变换点云 + 轨迹到 aligned frame
3. **高度过滤**: `mask_band = (h_min ≤ pts_aligned_y + d ≤ h_max)`
4. **世界 (x, z) → 图像 (px, py)**:
   * $\text{px} = W/2 + (x - o_x)/r$
   * $\text{py} = H/2 - (z - o_z)/r$
5. **累积**: `np.add.at(count, (py, px), 1)` 逐点+1
6. **阈值化**: `occupied = count >= τ`
7. **free corridor**: `cv2.polylines` 沿 trajectory BEV 画粗线, 半径 `free_corridor_radius_px`
8. **冲突消解**: `occupied &= ~free` (相机走过 = 一定通行)
9. **形态学膨胀**: `cv2.dilate(occupied, k)` 让墙更 "厚"
10. 组装 uint8 grid: `0=unknown, 127=free, 255=occupied`
11. 输出 meta 含 `R_align.tolist()` (给 pipeline_A 用)

#### `offline_pipeline_A.py` (脚本)

流程:

1. 加载 `static_map.npy` + `meta.json` → `static_bgr = render_static_map(...)`
2. 读 `R_align` from meta
3. 构造 `BEVCanvas(static_layer=static_bgr)` (bev_canvas 的 `_make_base` 会用它当底图)
4. 加载 DPVO **平面化后** 轨迹作 poses
5. 加载 `ground_plane_final.json` 只用来当 fall-back 世界系投影
6. 逐帧:
   * `nearest_pose` 找 pose
   * `yolo.step(frame)` → `TrackedPerson[]`
   * 每人 `compute_footpoint` → foot_uv
   * **相机-地面投影** `intersect_footpoint_with_camera_ground(foot, K, T_wc, h=0.1, pitch=15°)`
     * 相机系地面: $\mathbf{g}_c = (0, \cos\alpha, \sin\alpha)^T$, $\lambda = h / (\mathbf{g}_c \cdot \mathbf{r}_c)$
     * 得 $X_c$, 转到世界 $X_w = R_{wc} X_c + \mathbf{C}_w$
   * 失败 (lam≤0) → fall back 到世界系
   * **`Xw` 应用 `R_align`** → BEV 坐标一致于 static_map
   * `PeopleStateFilter.update` EMA + 速度门限
   * `bev_canvas.draw` 画 (相机点 + 轨迹 + active 人员点 + ID 标签)
   * `cv2.imshow` (仅 `--live` 时)
7. 写 3 个 mp4 + 3 个 json

---

## 3. 如何运行 (常用命令)

### 3.1 最短命令 (跑通 baseline + 全套 route_A)

```bash
cd /home/ros/ros2_orbslam3
conda activate dpvo

# (前置: DPVO 轨迹 output/dpvo/trajectory_tum.txt 已有)

# 1. 构建静态地图 + 平面化轨迹 (~2s)
python src/people_bev_tracker/scripts/build_route_A.py \
  --config src/people_bev_tracker/config/route_A.yaml \
  --pose output/dpvo/trajectory_tum.txt \
  --pointcloud "output/vggt_aligned_full_run/aligned_full/aligned_full_scene.ply" \
  --pointcloud-source vggt \
  --output-dir output/route_A

# 2. 主流水线 (~5 分钟, 3181 帧)
python src/people_bev_tracker/scripts/offline_pipeline_A.py \
  --config src/people_bev_tracker/config/route_A.yaml \
  --pose output/route_A/trajectory_flat.txt \
  --static-map output/route_A/static_map.npy \
  --static-map-meta output/route_A/static_map_meta.json \
  --ground-plane output/route_A/ground_plane_final.json \
  --output-dir output/route_A
```

### 3.2 实时窗口 (边跑边看)

```bash
# 加 --live 会开 cv2.imshow "Route A BEV" 窗口, 每帧显示当前 BEV
# WSL 需要 WSLg (DISPLAY=:0)
python src/people_bev_tracker/scripts/offline_pipeline_A.py \
  --config src/people_bev_tracker/config/route_A.yaml \
  --pose output/route_A/trajectory_flat.txt \
  --static-map output/route_A/static_map.npy \
  --static-map-meta output/route_A/static_map_meta.json \
  --ground-plane output/route_A/ground_plane_final.json \
  --output-dir output/route_A \
  --live
```

按 `q` 提前退出。

### 3.3 短测试 (30 帧, ~10 秒)

```bash
python src/people_bev_tracker/scripts/offline_pipeline_A.py \
  --config src/people_bev_tracker/config/route_A.yaml \
  --pose output/route_A/trajectory_flat.txt \
  --static-map output/route_A/static_map.npy \
  --static-map-meta output/route_A/static_map_meta.json \
  --ground-plane output/route_A/ground_plane_final.json \
  --output-dir output/route_A \
  --max-frames 30
```

### 3.4 换点云源

```bash
# 用 DPVO PLY (效果差, 尺度杂散)
python src/people_bev_tracker/scripts/build_route_A.py \
  --config src/people_bev_tracker/config/route_A.yaml \
  --pointcloud "project code/DPVO/mall_dpvo.ply" \
  --pointcloud-source dpvo \
  --output-dir output/route_A_dpvo_pcd

# 用 KV-Tracker (V2, 需先做 Sim3 对齐 - 尚未实现)
```

### 3.5 调 static_map 参数

```bash
python src/people_bev_tracker/scripts/tune_static_map_params.py \
  --config src/people_bev_tracker/config/route_A.yaml \
  --pose output/dpvo/trajectory_tum.txt \
  --pointcloud "output/vggt_aligned_full_run/aligned_full/aligned_full_scene.ply" \
  --output-dir output/route_A/static_map_candidates
```

会生成 48 张 `hXX-XX_tX_dkX.png`, 手挑最好看的组合写回 `route_A.yaml`。

### 3.6 从头到尾一条链 (DPVO 也重跑)

```bash
# (1) DPVO 相机轨迹 (5-8 分钟 GPU)
cd "project code/DPVO"
python demo.py \
  --imagedir /home/ros/ros2_orbslam3/resources/input_video.mp4 \
  --calib calib/custom_mall.txt \
  --name input_video_clean \
  --stride 2 \
  --save_trajectory
cp saved_trajectories/input_video_clean.txt \
   /home/ros/ros2_orbslam3/output/dpvo/trajectory_tum.txt

# (2) VGGT / KV-Tracker 稠密点云 (若还没有的话, 参见对应目录 README)
#     output/vggt_aligned_full_run/aligned_full/aligned_full_scene.ply 应该已经存在

# (3) route_A build + pipeline (~5 分钟)
cd /home/ros/ros2_orbslam3
conda activate dpvo
python src/people_bev_tracker/scripts/build_route_A.py \
  --config src/people_bev_tracker/config/route_A.yaml \
  --pose output/dpvo/trajectory_tum.txt \
  --pointcloud "output/vggt_aligned_full_run/aligned_full/aligned_full_scene.ply" \
  --pointcloud-source vggt \
  --output-dir output/route_A

python src/people_bev_tracker/scripts/offline_pipeline_A.py \
  --config src/people_bev_tracker/config/route_A.yaml \
  --pose output/route_A/trajectory_flat.txt \
  --static-map output/route_A/static_map.npy \
  --static-map-meta output/route_A/static_map_meta.json \
  --ground-plane output/route_A/ground_plane_final.json \
  --output-dir output/route_A
```

---

## 4. 后续优化方向

按优先级 + 工程价值:

### 4.1 短期 (低 hanging fruit)

1. **微调 static_map 参数**: 跑 `tune_static_map_params.py` 找视觉最好的 (height range / count_thresh / dilate)
2. **加相机高度先验反推 scale**: 已知眼镜 1.6m real, camera_h_median = 0.79 DPVO 单位 → scale ≈ 2.0 m/DPVO 单位; 应用到所有 xyz 后 BEV 网格变真米制
3. **视频降帧率**: 把 offline_pipeline_A 加 `--frame-stride N`, 只对每 N 帧跑 YOLO, 速度提升 N×
4. **人员 ReID 合并**: 195 个 track_id 里很多是同一个人被断开; 用一个 appearance bank + 短时窗口拼接, 应能压到 ~30-50

### 4.2 中期 (点云增强)

1. **KV-Tracker → Sim(3) → DPVO 世界系**:
   * 用 `kv_tracker/geometry.py:umeyama_alignment`
   * 输入: DPVO trajectory (1590 pose) + KV traj.npy (3180 pose)
   * 按 frame_index 匹配 (KV frame_2i ↔ DPVO frame_i)
   * 求 $(s, R, t)$ s.t. $s R p_{kv} + t \approx p_{dpvo}$
   * 变换 pcd.npy → merge 进 static_map 的输入点云
2. **多轨迹融合 static_map**:
   * 同一场景多次扫描 → 累积密度 → 提高 occupancy 覆盖率
3. **稠密深度估计辅助**: 每 30 帧跑 Depth-Anything V2 拿单帧稠密深度 → 反投更多点; 需要相机 pose 拼接

### 4.3 地图质量

1. **RANSAC 用两阶段**: 先大阈值找主平面, 再小阈值精修
2. **PCA 换成 M-estimator**: 用 IRLS 代替 SVD, 对离群更稳
3. **地面高度回归**: 不只拟合固定平面, 拟合分段 (楼梯/斜坡场景)
4. **动态障碍剔除**: 用 YOLO mask 反向投影, 从点云里删除行人区域 (当前只在 people layer 处理)

### 4.4 轨迹平面化

1. **姿态平面化**: 现在只平移 Y, 姿态未修 → 走路时相机 pitch 波动也保留了; 二期可以把 T_wc 的 roll/pitch 也 LPF 平滑
2. **短窗自适应**: 假设"整段视频在同一高度平面" 太强; 每 N 秒重拟一次地面, 支持慢速上下坡

### 4.5 系统集成

1. **ROS2 节点**:
   * 订阅 `/camera/image_raw` + `/tf` (DPVO)
   * 发布 `/bev/map_image` (sensor_msgs/Image) + `/bev/people_markers` (visualization_msgs/MarkerArray)
   * static_map 在 background thread 增量更新
2. **实时 (< 30ms/frame)**:
   * 现在 11 FPS 端到端, 瓶颈是 YOLO (~50ms/frame @ imgsz=960)
   * 换 YOLO11n @ imgsz=640 → 3× 加速; 或 TensorRT export
3. **网页远程查看**: 把 static_map + 每帧 people 位置流式推 WebSocket, 前端 Canvas 实时画

### 4.6 商场平面图配准 (V4)

1. **2D ICP (点-点)**:
   * static_map 的 occupied 点 vs CAD 平面图的墙点
   * 求 2D 相似变换 (scale + rotation + translation)
   * 迭代最小化点对应距离
2. **人工控制点**:
   * 用户在 static_map 和 CAD 图上各点 3-5 个对应点
   * 求 4-parameter similarity, 或 8-parameter homography (若地图有透视失真)
3. **NCC 全局搜索**: 静态地图 vs CAD 二值图做归一化互相关, 网格搜索最优 transform

---

## 5. 学习路径 (从零彻底看懂这套代码)

### 5.1 基础知识 (2-3 天)

必需的三块前置:

* **计算机视觉入门**:
  * Multiple View Geometry (Hartley & Zisserman) 前 3 章: 齐次坐标, 相机模型, 投影
  * Or 高翔《视觉 SLAM 十四讲》第 4-6 章
* **Python + NumPy**: 会 slicing, broadcasting, matmul, 就够了
* **OpenCV 基础**: `cv2.imread/imwrite`, `cv2.rectangle/circle/polylines`, `cv2.VideoCapture/Writer`

### 5.2 学习顺序 (按依赖)

看代码建议按下面顺序, 每读一个文件跑一次 baseline / short-run 印证:

```
Day 1: 相机模型 + 位姿基础
  ├── people_bev_tracker/camera_model.py       (K, pixel_to_ray)
  ├── people_bev_tracker/types.py              (CameraPose 数据结构)
  ├── people_bev_tracker/pose_io.py            (TUM 读取, 四元数→R)
  └── people_bev_tracker/ground_projection.py  (射线-地面相交, camera-frame vs world-frame)

  实验: 打开一个 pose, 手算一个像素点投到地面的世界坐标 (对比 Python 输出)

Day 2: 行人检测 + 状态过滤
  ├── people_bev_tracker/footpoint.py          (mask/bbox → 脚底像素)
  ├── people_bev_tracker/person_yolo_tracker.py (YOLO+BoT-SORT 封装)
  ├── people_bev_tracker/state_filter.py       (EMA + 速度门限)
  └── scripts/offline_pipeline.py              (baseline pipeline 全流程)

  实验: 跑 --max-frames 30 baseline, 打开 debug_overlay.mp4 看 bbox + track_id + foot 是否合理

Day 3: BEV 绘制
  └── people_bev_tracker/bev_canvas.py         (画布, 世界→像素映射, static_layer)
  └── people_bev_tracker/io_utils.py           (yaml, video, json)

  实验: 修改 bev_canvas.py 改个颜色, 重新跑, 看 mp4 变化

Day 4: 点云 + 地面 + 平面化 (route_A 核心)
  ├── people_bev_tracker/pointcloud_io.py      (读 PLY/NPY, 过滤)
  ├── people_bev_tracker/ground_fit.py         (RANSAC / PCA / 打分)
  ├── people_bev_tracker/trajectory_flatten.py (相机 h(t) 平面化数学)
  └── scripts/build_route_A.py                 (阶段 1-4)

  实验: 加 print 到 ground_fit 里, 看 3 方法各自 inlier/rmse/angle;
        改 route_A.yaml 里 flatten_mode: constant → lpf, 对比 trajectory_flat 差异

Day 5: 静态地图 + 集成
  ├── people_bev_tracker/static_map.py         (R_align + 密度栅格 + 后处理)
  ├── scripts/offline_pipeline_A.py            (阶段 5 主流水线)
  └── scripts/tune_static_map_params.py        (参数扫描)

  实验: 扫参数 → 挑最好的 → 改 yaml → 重跑; 看 static_map.png 变化;
        改颜色/膨胀核感受形态学效果

Day 6-7: 论文原理
  ├── src/KV-tracker/docs/KV_Track3r_论文透彻解析.md  (可选, 只关心 π³ + KV-cache 原理)
  └── DPVO paper (https://arxiv.org/abs/2208.04726, 只看 Method 章)

  实验: 不需要跑训练, 只需理解 "为什么 DPVO 是单目 sparse patch VO" 和
        "为什么 KV-Tracker 稠密点云在同一场景但坐标系不同"
```

### 5.3 深入实验 (提高)

1. **手写一个简化版**: 只 100 行 Python, 用 numpy + cv2, 输入 (pose_tum, pointcloud_ply, video), 输出一张 BEV 图 (没有 YOLO). 帮助理解最小内核.
2. **换一个视频**: 拿你手机拍一段室内走廊, 跑一次 DPVO + VGGT (如果没有 VGGT 就只用 DPVO), 然后跑 route_A. 观察参数需要如何调整.
3. **加个 feature**: 例如
   * 在 BEV 上加 "北方箭头" 图标
   * 加一个自适应 zoom (相机走出画布自动缩放)
   * 加一个键盘控制 (--live 模式下按 +/- 缩放)

### 5.4 阅读建议

* **别一次读完所有代码**. 一天一个文件, 跑一次实验印证.
* **每读一个函数, 手动跑一次输入 → 中间输出 → 最终输出**, 别只看 signature.
* **有疑问先打 print/breakpoint**, 别猜.
* **画流程图/公式草稿**: 尤其 ground_projection 和 static_map 里的 R_align, 手画 3 张纸能顶读 3 遍代码.

### 5.5 参考文档

* [01_RouteA_初版执行方案.md](01_RouteA_初版执行方案.md) — 初始计划文档 (数学 + 决策)
* [02_RouteA_DPVO主轨迹执行任务书.md](02_RouteA_DPVO主轨迹执行任务书.md) — DPVO 主轨迹执行说明
* [03_RouteA_方案评估与路线选择报告.md](03_RouteA_方案评估与路线选择报告.md) — 路线选择与方案对比
* [05_RouteA_V2静态地图优化执行方案_最新版.md](05_RouteA_V2静态地图优化执行方案_最新版.md) — 当前最新版优化任务书
* [../IMPLEMENTATION.md](../IMPLEMENTATION.md) — baseline 实现说明 (含 §3 公式)
* [../../KV-tracker/docs/KV_Track3r_论文透彻解析.md](../../KV-tracker/docs/KV_Track3r_论文透彻解析.md) — KV-Tracker 原理
* [../../KV-tracker/docs/KV_Track3r_商场导航与BEV地图应用方案.md](../../KV-tracker/docs/KV_Track3r_商场导航与BEV地图应用方案.md) — BEV 应用方案 (含路线 A/B/C)
* [../../../output/route_A/route_A_report.md](../../../output/route_A/route_A_report.md) — 本次运行结果报告

---

## 6. 快速对照表

| 场景 | 命令 |
| :--- | :--- |
| 我想看效果 | 播放 `output/route_A/bev_tracking_route_A.mp4` |
| 我想重新跑一遍 | 上面 §3.1 两条命令 |
| 我想实时看跑 | 上面 §3.2 加 `--live` |
| 静态地图不好看 | §3.5 扫参数; 或改 `route_A.yaml` |
| 想换点云源 | §3.4 (换 dpvo 会失败, 保留了作对比) |
| DPVO 都要重跑 | §3.6 |
| 想搞懂原理 | §5.2 学习路径 |
| 想加功能 | §4 后续优化 |
