# 纯视觉 SLAM 商场导航项目完整知识库

> 面向场景：技术面试、项目答辩、论文/开源方法理解、后续工程迭代。  
> 当前项目根目录：`/home/ros/ros2_orbslam3`  
> 当前日期：2026-07-02  
> 项目关键词：纯视觉 SLAM、DPVO、ORB-SLAM3、VGGT、KV-Track3r、Depth Anything V2、ScaRF-inspired 稠密建图、YOLO/BoT-SORT、动态行人 BEV、二维导航栅格、商场室内导航。

---

## 0. 一句话项目概述

本项目目标是：**只使用单目 RGB 视频，在商场室内环境中估计相机轨迹、识别动态行人、生成二维俯视导航栅格，并把相机轨迹和行人位置实时/离线叠加到 BEV 地图上。**

当前工程已经形成了以下主线：

```text
单目 RGB 视频
  -> DPVO 估计相机 6DoF 位姿轨迹
  -> YOLO/BoT-SORT 检测和跟踪动态行人
  -> footpoint + ground projection 得到行人 BEV 坐标
  -> Depth Anything V2 Metric Indoor 生成关键帧单目深度
  -> DPVO 位姿锚定深度反投影
  -> 地面尺度对齐 + 子图多帧一致性融合
  -> dense static point cloud
  -> 2D occupancy grid / 黑白导航栅格
  -> BEV 视频显示相机轨迹 + 动态行人
```

如果面试官问“你这个项目做了什么”，可以简洁回答：

```text
我做的是一个纯视觉商场导航原型系统。前端用 DPVO 提供鲁棒相机轨迹，后端用单目深度和多帧融合生成静态场景结构，再投影成二维导航栅格；同时用 YOLO/BoT-SORT 跟踪行人，通过地面约束把行人投影到 BEV 地图中，实现相机轨迹、动态行人、障碍和可通行区域的统一展示。
```

---

## 1. 我在这个项目中的工作与贡献

### 1.1 工程集成贡献

你可以这样描述自己的参与：

```text
1. 搭建 ROS2 + 离线 Python 混合工程，整合 DPVO、ORB-SLAM3、VGGT、KV-Track3r、Depth Anything V2、YOLO 跟踪模块。
2. 设计了 people_bev_tracker 工程，把相机轨迹、行人检测、地面投影、BEV 渲染、静态地图生成统一起来。
3. 评估了 KV-Track3r、VGGT、DPVO 的轨迹/点云质量，最终选择 DPVO 作为主轨迹源。
4. 发现 VGGT 点云导致静态地图稀疏、假墙、缺墙问题，进一步设计了 ScaRF-inspired 的纯视觉稠密建图方案。
5. 实现了 BEV 坐标系镜像校准，解决轨迹方向和真实世界转向相反的问题。
6. 实现了 V3.1：DPVO 位姿 + Depth Anything V2 Metric Indoor + 动态行人 mask + 尺度对齐 + 子图融合 + 2D occupancy。
7. 每个阶段输出中文报告、质量指标和可视化结果，形成可复现工程闭环。
```

### 1.2 关键技术贡献

```text
1. 纯单目尺度处理：
   使用地面平面和相机高度估计，把单目深度模型输出统一到 DPVO 世界单位。

2. 动态物体剔除：
   用 YOLO/BoT-SORT 识别人，建图前用 bbox/mask 膨胀区域剔除行人，避免人被写入静态地图。

3. 地面投影：
   用脚底像素射线与地面相交，将 2D 检测框转换成 3D/BEV 行人位置。

4. BEV 坐标校准：
   发现地图像“仰视图”，实现 mirror_y 等 2D 变换，使真实世界转向和 BEV 转向一致。

5. 子图多帧一致性：
   把关键帧深度反投影为点云，按 voxel 聚合，只保留被多个关键帧观测到的结构，减少单帧深度假障碍。
```

---

## 2. 当前项目目录和数据流

### 2.1 关键目录

```text
project code/DPVO
  DPVO 官方代码，负责纯视觉相机轨迹。

project code/ORB_SLAM3-master
  ORB-SLAM3 官方代码，ROS2 wrapper 通过 libORB_SLAM3.so 调用。

project code/VGGT/vggt-main
  VGGT 官方代码，曾用于生成稠密点云和相机结构，但 V3 不再作为主几何。

project code/KV-tracker/kv_tracker-main
  KV-Track3r 官方代码，只读调用，用于论文复现和对比。

src/dpvo_localization
  ROS2 Python 包，封装 DPVO 视频运行、标定准备、环境检查。

src/orbslam3_wrapper
  C++ ROS2 wrapper，调用 ORB-SLAM3 官方库。

src/video_publisher
  ROS2 视频发布节点，把视频帧发布到 /camera/image_raw。

src/KV-tracker
  KV-Track3r 外部封装，不修改官方代码。

src/people_bev_tracker
  本项目核心工程：行人 BEV、静态地图、V1/V2/V3 路线、稠密建图。
```

### 2.2 当前最重要的输出

```text
output/dpvo/trajectory_tum.txt
  DPVO 主相机轨迹。

output/route_A/trajectory_flat.txt
  平面化后的 DPVO 轨迹，去除头部上下颠簸。

output/route_A_v2/
  V2 基于 VGGT 点云 + free space 补全的 BEV 地图。

output/route_A_v3_scarf/
  V3.1 基于 Depth Anything V2 + DPVO 位姿 + 子图融合的新稠密地图。

output/route_A_v3_scarf/best/nav_binary_map.png
  黑白导航图。

output/route_A_v3_scarf/best/static_map_tricolor.png
  黑/白/灰三值占据图。

output/route_A_v3_scarf/best/topdown_3d_scene.png
  三维点云俯视展示图。

output/route_A_v3_scarf/bev_tracking_route_A_v3_dense.mp4
  最终 BEV 视频：地图 + 相机轨迹 + 动态行人。
```

---

## 3. 纯视觉 SLAM 基础知识

### 3.1 SLAM 的问题定义

SLAM 是 Simultaneous Localization and Mapping，即同时定位与建图。数学上可写为：

```text
p(x_0:t, m | z_1:t, u_1:t)
```

其中：

```text
x_t = t 时刻相机/机器人位姿
m   = 地图
z_t = 观测，例如图像、特征点、深度、点云
u_t = 控制或运动输入；纯视觉里通常没有显式控制输入
```

纯视觉 SLAM 只用图像序列，典型目标是估计：

```text
1. 相机位姿 T_wc(t)
2. 地图点 X_j
3. 可选稠密结构 / 深度 / occupancy grid
```

### 3.2 坐标系

本项目常见坐标系：

```text
camera frame:
  X 右
  Y 下
  Z 前

world frame:
  DPVO / ORB-SLAM3 输出的全局坐标

aligned world:
  把地面法向旋转到 +Y 后的世界系

BEV frame:
  从 aligned world 中选 x-z 两轴作为俯视平面
```

位姿使用齐次矩阵：

```text
T_wc =
[ R_wc  t_wc ]
[ 0     1    ]
```

含义：

```text
X_w = R_wc X_c + t_wc
```

逆变换：

```text
T_cw = T_wc^{-1}
X_c = R_wc^T (X_w - t_wc)
```

### 3.3 相机针孔模型

相机内参：

```text
K =
[ fx  0  cx ]
[ 0   fy cy ]
[ 0   0  1  ]
```

三维点投影到像素：

```text
[u, v, 1]^T ~ K [X/Z, Y/Z, 1]^T
```

展开：

```text
u = fx * X / Z + cx
v = fy * Y / Z + cy
```

像素反投影为相机射线：

```text
r_c = normalize(K^{-1} [u, v, 1]^T)
```

代码对应：

```text
src/people_bev_tracker/people_bev_tracker/camera_model.py
src/people_bev_tracker/people_bev_tracker/ground_projection.py
```

### 3.4 单目尺度模糊

纯单目 SLAM 的核心限制：

```text
如果所有 3D 点和相机平移同时乘以尺度 s，投影到图像上完全不变。
```

即：

```text
X'_w = s X_w
t'_wc = s t_wc
```

则：

```text
u = fx * X / Z + cx
```

不变，因为 X 和 Z 同时乘以 s。

所以纯单目天然不知道“1 个 SLAM 单位等于多少米”。本项目解决方式：

```text
1. 工程内部先使用 DPVO 单位。
2. 用地面平面和相机高度估计相对尺度。
3. 若需要真米制，可用眼镜实际高度、地砖尺寸或商场 CAD 平面图做尺度锚定。
```

当前 V3 报告中：

```text
Depth Anything V2 Metric Indoor 输出米制深度。
通过地面高度约束估计全局 scale ≈ 0.605 DPVO 单位 / 米。
```

### 3.5 视觉里程计 VO 与 SLAM 的区别

```text
VO:
  只关心短期连续位姿估计。
  通常没有全局闭环或全局地图优化。

SLAM:
  同时维护位姿和地图。
  通常有关键帧、地图点、回环检测、全局优化。
```

本项目主轨迹用 DPVO，更偏视觉里程计；但后端 BEV 和 dense map 属于建图模块。

---

## 4. 后端优化基础：BA、Pose Graph、MAP

### 4.1 Bundle Adjustment

视觉 SLAM 常见优化目标：

```text
min_{T_i, X_j} Σ ρ( || z_ij - π(T_i^{-1} X_j) ||^2 )
```

其中：

```text
T_i = 第 i 帧相机位姿
X_j = 第 j 个地图点
z_ij = 第 i 帧观测到第 j 个点的像素坐标
π() = 相机投影函数
ρ() = 鲁棒核，例如 Huber，降低外点影响
```

重投影误差：

```text
e_ij = z_ij - π(T_i^{-1} X_j)
```

### 4.2 Pose Graph Optimization

当有相对位姿约束时：

```text
min_{T_i} Σ || Log( Z_ij^{-1} T_i^{-1} T_j ) ||^2_Ω
```

其中：

```text
Z_ij = i 到 j 的相对位姿观测
Ω    = 信息矩阵
Log  = SE(3) 到李代数 se(3) 的映射
```

Pose graph 常用于回环优化。

### 4.3 MAP 估计

Maximum A Posteriori：

```text
x* = argmax_x p(x | z)
   = argmax_x p(z | x) p(x)
```

取负对数后变成最小二乘：

```text
x* = argmin_x -log p(z | x) - log p(x)
```

ORB-SLAM3 等传统 SLAM 后端主要是 MAP/非线性最小二乘。

---

## 5. DPVO：本项目主相机轨迹

### 5.1 DPVO 是什么

DPVO 全称 Deep Patch Visual Odometry。其论文提出用稀疏 patch 而不是密集光流来做高效 VO，并结合循环更新网络与可微 Bundle Adjustment。论文指出 DPVO 在保持精度的同时，相比密集光流方法计算更高效。官方代码位于：

```text
project code/DPVO
```

本项目中：

```text
DPVO 是主相机轨迹来源。
KV-Track3r 和 VGGT 不参与主位姿。
```

参考：

```text
论文: https://arxiv.org/abs/2208.04726
代码: https://github.com/princeton-vl/DPVO
```

### 5.2 DPVO 在项目中的调用

代码入口：

```text
src/dpvo_localization/dpvo_localization/run_dpvo_video.py
```

主要作用：

```text
1. 接收视频/图片序列路径。
2. 设置 DPVO 官方代码路径。
3. 调用 project code/DPVO/demo.py。
4. 输出 TUM 格式轨迹。
```

常用命令：

```bash
cd /home/ros/ros2_orbslam3
conda activate dpvo

ros2 run dpvo_localization run_dpvo_video \
  --dpvo-root "project code/DPVO" \
  --imagedir "resources/input_video.mp4" \
  --calib "project code/DPVO/calib/custom_mall.txt" \
  --name input_video_clean \
  --stride 2 \
  --save_trajectory \
  --plot
```

输出：

```text
project code/DPVO/saved_trajectories/input_video_clean.txt
output/dpvo/trajectory_tum.txt
```

### 5.3 为什么选择 DPVO 做主轨迹

你的实测结论：

```text
1. DPVO 轨迹比 KV-Track3r 在当前商场视频上更稳定。
2. DPVO 在长走廊中轨迹更连续、少回跳。
3. KV-Track3r 虽能输出稠密结构和 confidence，但轨迹效果不如 DPVO。
4. ORB-SLAM3 在单目动态商场环境中初始化和跟踪较脆弱。
```

因此 Route A 后续所有版本都坚持：

```text
主轨迹 = output/dpvo/trajectory_tum.txt / output/route_A/trajectory_flat.txt
```

### 5.4 轨迹平面化

眼镜/头戴相机会有上下颠簸。若直接用于 BEV，会出现轨迹高度噪声。

地面平面：

```text
n^T X + d = 0
```

相机中心 C(t) 到地面的 signed height：

```text
h(t) = n^T C(t) + d
```

constant 平面化：

```text
h_ref = median(h(t))
C_flat(t) = C(t) + (h_ref - h(t)) n
```

只改平移，不改姿态。

代码：

```text
src/people_bev_tracker/people_bev_tracker/trajectory_flatten.py
```

输出：

```text
output/route_A/trajectory_flat.txt
output/route_A/trajectory_flat_stats.json
```

---

## 6. ORB-SLAM3：传统特征 SLAM 对照与 ROS2 在线链路

### 6.1 ORB-SLAM3 是什么

ORB-SLAM3 是特征点法 SLAM 系统，支持：

```text
monocular
stereo
RGB-D
visual-inertial
multi-map
pinhole / fisheye
```

论文强调其是视觉、视觉惯性、多地图 SLAM 系统，并使用 MAP 估计和回环/地图复用。参考：

```text
论文: https://arxiv.org/abs/2007.11898
代码: https://github.com/UZ-SLAMLab/ORB_SLAM3
```

### 6.2 本项目如何调用 ORB-SLAM3

代码：

```text
src/orbslam3_wrapper/src/mono_node.cpp
src/orbslam3_wrapper/CMakeLists.txt
launch/orbslam3_video.launch.py
```

核心调用：

```cpp
ORB_SLAM3::System(vocab_path, settings_path, ORB_SLAM3::System::MONOCULAR, enable_viewer)
SLAM_->TrackMonocular(im_gray, timestamp)
```

ROS2 数据流：

```text
video_publisher_node
  -> /camera/image_raw
  -> orbslam3_wrapper mono_node
  -> ORB-SLAM3 TrackMonocular
  -> /slam/pose
  -> /slam/odom
  -> /slam/map_points
  -> /slam/debug_image
```

### 6.3 ORB-SLAM3 在项目中的定位

```text
1. 用作传统 SLAM 对照。
2. 提供 ROS2 在线链路样例。
3. 当前不作为主轨迹来源，因为商场动态场景、单目初始化、动态行人和遮挡对其影响较大。
```

### 6.4 ORB-SLAM3 常见面试点

```text
Q: ORB-SLAM3 和 DPVO 有什么区别？
A: ORB-SLAM3 是传统特征点 SLAM，依赖 ORB 特征、BoW 回环、BA 和地图点；DPVO 是学习型 VO，用深度网络跟踪 patch 并结合可微 BA。ORB-SLAM3 有完整地图和回环，DPVO 在当前视频上轨迹更稳定。
```

---

## 7. VIO：为什么本项目是纯视觉，但仍要理解 VIO

### 7.1 VIO 是什么

VIO 是 Visual-Inertial Odometry，融合视觉和 IMU：

```text
camera:
  提供相对几何和尺度约束的一部分

IMU:
  提供角速度、加速度、高频运动预测、重力方向、尺度可观性
```

典型状态：

```text
x = {R, p, v, b_g, b_a}
```

其中：

```text
R   = 姿态
p   = 位置
v   = 速度
b_g = 陀螺仪 bias
b_a = 加速度计 bias
```

### 7.2 IMU 预积分基本公式

IMU 测量：

```text
ω_m = ω + b_g + n_g
a_m = R^T (a - g) + b_a + n_a
```

离散预积分：

```text
ΔR_ij ≈ Π Exp((ω_m - b_g) Δt)
Δv_ij ≈ Σ ΔR_ik (a_m - b_a) Δt
Δp_ij ≈ Σ [Δv Δt + 1/2 ΔR_ik (a_m - b_a) Δt^2]
```

VIO 残差大致包括：

```text
r_R = Log( ΔR_ij^T R_i^T R_j )
r_v = R_i^T (v_j - v_i - g Δt) - Δv_ij
r_p = R_i^T (p_j - p_i - v_i Δt - 1/2 g Δt^2) - Δp_ij
```

### 7.3 VINS-Fusion

VINS-Fusion 是优化式多传感器状态估计器，支持：

```text
stereo cameras
mono camera + IMU
stereo cameras + IMU
visual loop closure
online spatial calibration
online temporal calibration
```

参考：

```text
代码: https://github.com/HKUST-Aerial-Robotics/VINS-Fusion
```

### 7.4 为什么本项目没用 VIO

```text
1. 当前输入是普通单目视频，没有同步 IMU。
2. VIO 对硬件要求高，需要相机-IMU 时间同步、外参标定、IMU 噪声模型。
3. 本项目目标是验证纯视觉低成本商场导航。
4. 后续如果换成手机/眼镜 IMU，可扩展为 VIO，提高尺度和姿态稳定性。
```

面试可答：

```text
我了解 VIO 能解决单目尺度、重力方向和快速运动问题，但当前数据源没有 IMU，所以我先走纯视觉路线。工程上我把 DPVO 位姿、地面约束和单目深度尺度对齐结合起来，作为无 IMU 条件下的替代方案。
```

---

## 8. VGGT：为什么曾经使用，为什么后来替换

### 8.1 VGGT 是什么

VGGT，全称 Visual Geometry Grounded Transformer，是前馈式几何基础模型。它可以从一张或多张图像直接预测：

```text
camera parameters
depth maps
point maps
3D point tracks
dense point cloud
```

参考：

```text
论文: https://arxiv.org/abs/2503.11651
代码: https://github.com/facebookresearch/vggt
```

### 8.2 项目里如何使用 VGGT

主要脚本：

```text
scripts/run_vggt_video.py
scripts/run_vggt_scene.py
scripts/aggregate_vggt_aligned.py
scripts/render_aligned_scene.py
```

输出：

```text
output/vggt_aligned_full_run/aligned_full/aligned_full_scene.ply
output/route_A/pointcloud_vggt.npy
```

V1/V2 曾使用 VGGT 点云生成静态地图：

```text
VGGT point cloud
  -> ground fit
  -> height filtering
  -> 2D histogram
  -> occupancy grid
```

### 8.3 VGGT 在本项目中的问题

V1/V2 报告显示：

```text
1. VGGT 地面 inlier 只有 3.4%。
2. 静态地图 unknown 占比过高。
3. 中央出现假墙，把 free space 切断。
4. 后半段走廊只有一侧墙，另一侧缺失。
5. 商场玻璃、反光、开阔区域会导致点云不稳定。
```

因此 V3.1 中：

```text
VGGT 不再作为主几何来源。
VGGT 只作为 baseline / fallback / 对照。
```

### 8.4 面试可答

```text
我先尝试用 VGGT 直接生成稠密点云做静态地图，但发现它在商场玻璃、反光和长走廊场景中会出现假墙和缺墙。后续我没有继续靠形态学强行修图，而是改成 DPVO 固定位姿 + Depth Anything 单目深度 + 多帧一致性融合，从几何来源上解决问题。
```

---

## 9. KV-Track3r：论文复现、对比与工程定位

### 9.1 KV-Track3r 是什么

KV-Track3r 是基于 key-value memory / tracker 思想的 scene-level tracking 方法。项目中复现了官方代码：

```text
project code/KV-tracker/kv_tracker-main
src/KV-tracker
```

本项目通过 wrapper 调用官方代码，不直接修改官方源码。

关键封装：

```text
src/KV-tracker/scripts/run_official_kv_tracker.py
src/KV-tracker/scripts/export_repro_outputs.py
src/KV-tracker/kv_track3r_app/official_bridge.py
src/KV-tracker/kv_track3r_app/output_converter.py
```

输出：

```text
output/kv_track3r_repro/trajectory.json
output/kv_track3r_repro/confidence.json
output/kv_track3r_repro/keyframes.json
output/kv_track3r_repro/summary.md
```

### 9.2 为什么没有把 KV-Track3r 作为主轨迹

实测结论：

```text
1. 当前商场视频中 KV-Track3r 轨迹不如 DPVO 稳。
2. 长走廊场景中 KV-Track3r 关键帧数量限制导致后段依赖早期 KV-cache。
3. DPVO 输出 1590 个 pose，更连续，更适合作为主轨迹。
4. KV-Track3r 的优势是结构点和 confidence，可作为对照或辅助，但不适合替换主轨迹。
```

### 9.3 面试可答

```text
我复现了 KV-Track3r，并把它输出转换成工程 JSON/TUM 格式。但我没有盲目采用论文方法作为主模块，而是做了数据驱动的工程评估：在当前商场视频上，DPVO 轨迹质量更好，因此保留 DPVO 做主定位，KV-Track3r 作为结构和置信度参考。
```

---

## 10. Depth Anything V2 与 ScaRF-inspired V3.1

### 10.1 Depth Anything V2 是什么

Depth Anything V2 是单目深度基础模型。论文提出相比 V1 更细、更稳，并提供不同规模模型和 metric depth 模型。参考：

```text
论文: https://arxiv.org/abs/2406.09414
代码: https://github.com/DepthAnything/Depth-Anything-V2
```

当前项目使用：

```text
Depth Anything V2 Metric Indoor Small
HuggingFace transformers 版本
```

体检/安装报告：

```text
output/route_A_v3_scarf/reports/00_深度后端体检与安装报告.md
```

### 10.2 ScaRF-SLAM 思想

ScaRF-SLAM 核心思路：

```text
1. tracking 和 mapping 解耦。
2. classical visual SLAM 提供稳定低延迟位姿。
3. feed-forward depth / geometric foundation model 提供稠密结构。
4. 通过 frame scale optimization 和 submap scale optimization 保持尺度一致。
5. 通过 projection-based point cloud fusion 融合多帧点云。
```

参考：

```text
论文: https://arxiv.org/abs/2606.00307v1
代码: https://github.com/ori-drs/ScaRF-SLAM
```

本项目没有逐行复现官方 ScaRF-SLAM，而是实现：

```text
ScaRF-inspired lightweight mapper
```

对应代码：

```text
src/people_bev_tracker/people_bev_tracker/scarf_like/
src/people_bev_tracker/scripts/build_route_A_v3_scarf_like.py
```

### 10.3 V3.1 实际数据流

```text
1. 关键帧选择:
   input_video.mp4 + trajectory_flat.txt
   -> 140 keyframes

2. 动态 mask:
   people_tracks_route_A_v3.json
   -> dynamic_masks

3. 单目深度:
   Depth Anything V2 Metric Indoor
   -> depth_cache/*.npy

4. 尺度对齐:
   depth meters -> DPVO units
   scale ≈ 0.605 DPVO unit / meter

5. 子图融合:
   12 keyframes per submap, overlap 3
   voxel size 0.015 DPVO unit
   min_observations = 2
   -> dense_global_static.npy / ply

6. Occupancy:
   dense point cloud
   -> floor layer / obstacle layer / unknown
   -> nav_binary_map / tricolor / topdown_3d
```

### 10.4 V3.1 结果

来自：

```text
output/route_A_v3_scarf/route_A_v3_scarf_execution_report.md
```

关键指标：

```text
V2 largest_free_component_ratio = 35.12%
V3 largest_free_component_ratio = 48.70%

V2 active_free_ratio = 37.24%
V3 active_free_ratio = 49.88%

V3 trajectory_collision_ratio = 0.00%
```

结论：

```text
1. 中央假墙已消除。
2. 后半段双侧障碍覆盖改善。
3. 地图从 VGGT 主几何切换到 Depth Anything + DPVO 多帧融合。
4. 仍需进一步提升 largest_free 到 50% 以上。
```

---

## 11. 动态行人检测、跟踪与 BEV 投影

### 11.1 检测与跟踪

代码：

```text
src/people_bev_tracker/people_bev_tracker/person_yolo_tracker.py
```

配置：

```text
src/people_bev_tracker/config/route_A_v2.yaml
src/people_bev_tracker/config/route_A_v3_scarf.yaml
```

使用：

```text
YOLO segmentation / detection
BoT-SORT / ByteTrack style tracker
```

输出每帧：

```text
track_id
bbox_xyxy
score
mask optional
foot_pixel
world_xyz
bev_xy
filtered_bev_xy
```

### 11.2 footpoint

为什么用脚底点：

```text
行人在地图中的位置应是脚落地点，而不是 bbox 中心或人体中心。
```

计算：

```text
mask 底部区域 or bbox bottom center
```

代码：

```text
src/people_bev_tracker/people_bev_tracker/footpoint.py
```

### 11.3 脚底像素到地面交点

像素射线：

```text
r_c = K^{-1} [u, v, 1]^T
```

世界射线：

```text
r_w = R_wc r_c
C_w = t_wc
```

地面：

```text
n^T X + d = 0
```

求交：

```text
X_w = C_w + λ r_w
n^T (C_w + λ r_w) + d = 0
λ = - (n^T C_w + d) / (n^T r_w)
```

代码：

```text
src/people_bev_tracker/people_bev_tracker/ground_projection.py
```

对于第一人称视频，远处行人常只露上半身，世界地面投影可能失败；项目中还用了 camera-frame ground fallback：

```text
g_c · X_c = h
λ = h / (g_c · r_c)
X_w = R_wc (λ r_c) + C_w
```

### 11.4 轨迹平滑

行人投影会抖动，使用 EMA：

```text
p_t^filtered = α p_t + (1 - α) p_{t-1}^filtered
```

代码：

```text
src/people_bev_tracker/people_bev_tracker/state_filter.py
```

同时使用速度门限过滤大跳变。

---

## 12. BEV 地图、占据栅格和坐标校准

### 12.1 Occupancy Grid

三值地图：

```text
occupied = 障碍
free     = 可通行
unknown  = 未观测
```

二值导航图：

```text
free -> white
occupied + unknown -> black
```

原因：

```text
导航要保守，未知区域不能当作可通行。
```

### 12.2 Log-odds 占据更新

经典占据栅格可用 log-odds：

```text
l_t(m_i) = log( p(m_i | z_1:t) / (1 - p(m_i | z_1:t)) )
```

递推：

```text
l_t(m_i) = l_{t-1}(m_i) + log( p(m_i | z_t) / (1 - p(m_i | z_t)) ) - l_0
```

本项目早期没有直接用 log-odds，而是用：

```text
point density
multi-frame observations
floor projection
ray carving
morphology
```

来近似生成 occupancy。

### 12.3 地面对齐

地面平面：

```text
n^T X + d = 0
```

求旋转矩阵：

```text
R_align n = [0, 1, 0]^T
```

使用 Rodrigues 公式。

代码：

```text
src/people_bev_tracker/people_bev_tracker/static_map.py
_rot_matrix_align_a_to_b
```

### 12.4 BEV 坐标

从 aligned world 选：

```text
bev_axes = ["x", "z"]
```

世界到像素：

```text
px = W/2 + (x - origin_x) / resolution
py = H/2 - (z - origin_z) / resolution
```

代码：

```text
src/people_bev_tracker/people_bev_tracker/bev_canvas.py
```

### 12.5 mirror_y 坐标校准

发现问题：

```text
BEV 像仰视图，真实世界转向和地图转向相反。
```

解决：

```text
选择 mirror_y
[x, y] -> [x, -y]
```

配置：

```text
output/route_A_v3_scarf/alignment_selected.json
```

代码：

```text
src/people_bev_tracker/people_bev_tracker/bev_alignment.py
src/people_bev_tracker/scripts/calibrate_bev_alignment.py
src/people_bev_tracker/scripts/apply_bev_alignment.py
```

注意：

```text
不能只 flip 最终图片。
必须统一作用到 static_map、相机轨迹、heading、行人 BEV 坐标和 dense map projection。
```

---

## 13. V1 / V2 / V3 技术演进

### 13.1 V1

```text
DPVO trajectory
VGGT pointcloud
ground fitting
trajectory flatten
static_map
YOLO people
BEV render
```

问题：

```text
free 只有 0.80%
unknown 96.26%
地图像稀疏点云投影
```

### 13.2 V2

增强：

```text
free corridor
camera frustum
SAM floor mask
obstacle morphology
multi-mode render
map quality metrics
```

效果：

```text
active_free_ratio = 37.24%
largest_free_component_ratio = 35.12%
```

问题：

```text
仍依赖 VGGT 点云。
中央假墙和后半段缺墙无法靠后处理解决。
```

### 13.3 V3 前置

```text
BEV 坐标 mirror_y 校准
```

解决：

```text
轨迹方向和真实世界转向一致。
```

### 13.4 V3.1

```text
DPVO fixed poses
Depth Anything V2 Metric Indoor
dynamic mask
scale alignment
submap fusion
dense occupancy
```

效果：

```text
central false wall removed
active_free_ratio = 49.88%
largest_free_component_ratio = 48.70%
trajectory_collision_ratio = 0.00%
```

---

## 14. LINGBOT MAP / 语义地图 / 语言地图的理解

你提到的 “LINGBOT MAP” 在当前仓库中没有发现独立代码模块或标准开源目录。可以在答辩中把它归纳为：

```text
language / semantic map for navigation
```

即在几何地图上叠加语义：

```text
1. 地面 / 可通行区域
2. 墙 / 柜台 / 店铺边界
3. 行人 / 动态障碍
4. 商铺入口 / 电梯 / 扶梯 / 导航目标
5. 自然语言指令与地图区域绑定
```

如果未来扩展到 LingBot Map 类似能力，可以做：

```text
RGB frame
  -> semantic segmentation / open-vocabulary detection
  -> object landmarks
  -> BEV semantic layer
  -> language query:
       “去扶梯旁边”
       “避开人多区域”
       “沿着右侧店铺走”
```

底层仍依赖：

```text
SLAM pose
2D/3D map
semantic label
topological graph
```

面试可答：

```text
当前我的系统主要完成 metric/geometric map 和 dynamic people layer。语言地图还没有作为完整模块落地，但工程结构已经为 semantic layer 预留了接口，后续可接 open-vocabulary segmentation 或 VLM，把商铺、扶梯、入口等语义绑定到 BEV 栅格和拓扑图上。
```

---

## 15. 主流 SLAM 方法扩展对比

### 15.1 传统滤波类

```text
EKF-SLAM
FastSLAM
Particle Filter SLAM
```

特点：

```text
适合早期低维状态；
理论清晰；
大规模视觉地图中扩展性较弱。
```

### 15.2 特征点法

代表：

```text
PTAM
ORB-SLAM / ORB-SLAM2 / ORB-SLAM3
VINS-Mono / VINS-Fusion
OpenVINS
```

特点：

```text
提取角点/ORB 特征；
匹配、PnP、三角化、BA；
稀疏地图；
可做回环和重定位。
```

优点：

```text
效率高、解释性强、工程成熟。
```

缺点：

```text
弱纹理、动态物体、玻璃反光下不稳定。
```

### 15.3 直接法 / 半直接法

代表：

```text
LSD-SLAM
DSO
SVO
```

特点：

```text
直接最小化光度误差，不完全依赖特征点。
```

光度误差：

```text
E = Σ || I_i(u) - I_j( π(T_ji, D_i(u)) ) ||^2
```

优点：

```text
可利用更多像素信息。
```

缺点：

```text
依赖光照一致性，对曝光变化、动态物体敏感。
```

### 15.4 学习型 VO / SLAM

代表：

```text
DeepVO
DROID-SLAM
DPVO
TartanVO
```

特点：

```text
用神经网络学习匹配、深度、运动或更新算子。
```

本项目选择 DPVO 的原因：

```text
在当前商场视频上，轨迹更稳，且易于离线跑出 TUM 轨迹。
```

### 15.5 视觉惯性 SLAM / VIO

代表：

```text
VINS-Mono
VINS-Fusion
OpenVINS
ORB-SLAM3 VI
Kimera-VIO
```

优点：

```text
尺度可观；
重力方向稳定；
快速运动更稳。
```

缺点：

```text
需要 IMU、时间同步、外参、噪声标定。
```

### 15.6 神经隐式地图 / NeRF / 3DGS / Dense SLAM

代表：

```text
iMAP
NICE-SLAM
Co-SLAM
NeRF-SLAM
Gaussian Splatting SLAM
ScaRF-SLAM
```

特点：

```text
追求稠密几何、可渲染地图或神经场表示。
```

和本项目关系：

```text
本项目 V3.1 借鉴 ScaRF-SLAM 的 tracking/mapping 解耦和子图尺度一致性，但没有完整训练神经场，而是更工程化地用 Depth Anything V2 + 点云融合 + occupancy grid。
```

---

## 16. 关键代码文件索引

### 16.1 DPVO

```text
src/dpvo_localization/dpvo_localization/run_dpvo_video.py
src/dpvo_localization/dpvo_localization/prepare_dpvo_calib.py
src/dpvo_localization/dpvo_localization/dpvo_enhancement.py
```

### 16.2 ORB-SLAM3

```text
src/orbslam3_wrapper/src/mono_node.cpp
src/orbslam3_wrapper/CMakeLists.txt
launch/orbslam3_video.launch.py
launch/orbslam3_only.launch.py
```

### 16.3 People BEV

```text
src/people_bev_tracker/people_bev_tracker/person_yolo_tracker.py
src/people_bev_tracker/people_bev_tracker/footpoint.py
src/people_bev_tracker/people_bev_tracker/ground_projection.py
src/people_bev_tracker/people_bev_tracker/state_filter.py
src/people_bev_tracker/people_bev_tracker/bev_canvas.py
src/people_bev_tracker/scripts/offline_pipeline_A.py
```

### 16.4 静态地图 V1/V2

```text
src/people_bev_tracker/people_bev_tracker/pointcloud_io.py
src/people_bev_tracker/people_bev_tracker/ground_fit.py
src/people_bev_tracker/people_bev_tracker/trajectory_flatten.py
src/people_bev_tracker/people_bev_tracker/static_map.py
src/people_bev_tracker/people_bev_tracker/free_space.py
src/people_bev_tracker/people_bev_tracker/map_quality.py
src/people_bev_tracker/scripts/build_route_A.py
src/people_bev_tracker/scripts/tune_static_map_v2.py
```

### 16.5 V3 ScaRF-inspired

```text
src/people_bev_tracker/people_bev_tracker/bev_alignment.py
src/people_bev_tracker/people_bev_tracker/scarf_like/keyframes.py
src/people_bev_tracker/people_bev_tracker/scarf_like/dynamic_mask.py
src/people_bev_tracker/people_bev_tracker/scarf_like/depth_backend.py
src/people_bev_tracker/people_bev_tracker/scarf_like/scale_alignment.py
src/people_bev_tracker/people_bev_tracker/scarf_like/submap_fusion.py
src/people_bev_tracker/people_bev_tracker/scarf_like/occupancy_from_dense.py
src/people_bev_tracker/people_bev_tracker/scarf_like/render_3d_topdown.py
src/people_bev_tracker/scripts/check_depth_backends.py
src/people_bev_tracker/scripts/build_route_A_v3_scarf_like.py
src/people_bev_tracker/scripts/run_route_A_v3_pipeline.py
```

### 16.6 KV-Track3r

```text
src/KV-tracker/scripts/run_official_kv_tracker.py
src/KV-tracker/scripts/export_repro_outputs.py
src/KV-tracker/kv_track3r_app/official_bridge.py
src/KV-tracker/kv_track3r_app/output_converter.py
```

---

## 17. 常见面试问题与回答

### 17.1 第一轮：基础与项目概述

**Q1：你的项目输入输出是什么？**

```text
输入是单目商场视频。输出包括相机轨迹、动态行人轨迹、二维黑白导航栅格、三值 occupancy grid、三维点云俯视图和 BEV 视频。
```

**Q2：为什么说是纯视觉？**

```text
因为定位和建图都只使用 RGB 视频，没有 IMU、LiDAR、RGB-D。DPVO 是纯视觉 VO，Depth Anything 是单目 RGB 深度估计，行人检测也是 RGB 检测。
```

**Q3：单目没有尺度怎么解决？**

```text
内部先使用 DPVO 单位；然后用地面平面和相机高度约束把单目深度统一到 DPVO 世界单位。若要真米制，可用眼镜实际高度、地砖尺寸或 CAD 平面图进一步标定。
```

**Q4：为什么不用 ORB-SLAM3 做主轨迹？**

```text
我跑了 ORB-SLAM3 ROS2 wrapper，但商场单目视频动态人多、纹理反光、初始化敏感。DPVO 在当前视频上轨迹更连续，所以工程上选 DPVO 做主轨迹。
```

### 17.2 第二轮：算法细节

**Q5：行人 2D 检测框如何变成 BEV 坐标？**

```text
先从 mask/bbox 取脚底像素，再用 K^{-1}[u,v,1] 得到相机射线，与地面平面求交得到 3D 世界点，最后通过 R_align 和 BEV 坐标变换投到二维地图。
```

公式：

```text
λ = - (n^T C_w + d) / (n^T r_w)
X_w = C_w + λ r_w
```

**Q6：为什么要做 trajectory flatten？**

```text
眼镜视频有头部上下颠簸，DPVO 轨迹的高度会抖。BEV 地图假设商场地面近似平坦，所以用地面法向把相机中心投到恒定高度，去除垂直方向周期噪声。
```

**Q7：V2 为什么失败？**

```text
V2 静态障碍主要来自 VGGT 点云。VGGT 在当前商场视频里有假墙、缺墙、地面 inlier 低的问题。虽然 V2 用 corridor、frustum、SAM floor 补 free，但几何源不可靠，导致主 free 连通域只有 35%。
```

**Q8：V3 怎么改善？**

```text
V3 用 DPVO 位姿锚定每个关键帧的 Depth Anything V2 度量深度，动态行人先 mask 掉，再通过地面尺度对齐和子图多帧一致性过滤生成 dense static point cloud，最后投成 occupancy。它替换了 VGGT 主几何。
```

**Q9：子图融合怎么减少假墙？**

```text
单帧深度可能产生漂浮点。V3 把多个关键帧反投影点云按 voxel 聚合，只保留被至少两个关键帧观测到的 voxel。这样单帧噪声和动态物体更容易被过滤。
```

### 17.3 第三轮：工程与优化

**Q10：你如何评估地图质量？**

```text
我设计了 active_free_ratio、active_unknown_ratio、trajectory_collision_ratio、obstacle_small_component_ratio、largest_free_component_ratio 等指标。V3 相比 V2，largest_free 从 35.12% 提升到 48.70%，active_free 从 37.24% 提升到 49.88%。
```

**Q11：如何保证动态行人不污染静态地图？**

```text
建图前读取 people_tracks，生成 bbox/mask invalid 区域，并膨胀覆盖人体边缘。深度反投影时这些像素不生成静态点，所以行人不会写进 static_map。
```

**Q12：如果地图方向反了怎么办？**

```text
我实现了 bev_alignment 模块，支持 mirror_x、mirror_y、rotate_180 等候选变换。最终用户确认 mirror_y 正确，该变换统一作用于 static map、相机轨迹、heading 和行人坐标，而不是只翻转 PNG。
```

**Q13：如果要进一步优化，你会做什么？**

```text
1. 增加关键帧和子图重叠，提高 obs>=3 的墙体覆盖。
2. 使用更大的 Depth Anything V2 Metric Indoor Base/Large。
3. 开启相邻关键帧 overlap scale refine。
4. 接官方 ScaRF-SLAM 作为 V3.2，对比其 pts_global。
5. 使用 CAD/floorplan 做 2D 配准，获得真米制和真实楼层方向。
```

---

## 18. 项目答辩中的技术叙事模板

### 18.1 简短版

```text
这个项目是一个纯视觉商场导航原型。我使用 DPVO 从单目视频估计相机轨迹，用 YOLO/BoT-SORT 跟踪行人，并通过脚底点和地面约束把行人投到 BEV 地图。早期我尝试用 VGGT 点云生成静态地图，但发现商场玻璃和长走廊会导致假墙和缺墙，所以后续借鉴 ScaRF-SLAM 思路，用 DPVO 位姿锚定 Depth Anything V2 单目深度，通过尺度对齐和子图多帧一致性融合生成新的 dense static map，最终投影成黑白导航栅格。整个系统保持纯视觉，不依赖 LiDAR、RGB-D 或 IMU。
```

### 18.2 面试官追问“你的创新点是什么”

```text
不是提出新 SLAM 理论，而是完成了一个复杂工程系统的路线选择和集成优化。我做了多种开源方法的实测对比：DPVO、ORB-SLAM3、VGGT、KV-Track3r。最后选择 DPVO 做主轨迹，使用 Depth Anything 和 ScaRF-style 子图融合替代不可靠的 VGGT 点云，并把动态行人过滤、地面投影、BEV 坐标校准、occupancy grid 质量评估做成完整闭环。
```

### 18.3 面试官追问“你写了哪些代码”

可以回答：

```text
我主要写/组织的是 people_bev_tracker 工程，包括：

1. 行人 footpoint 和地面投影。
2. BEV canvas 和动态行人轨迹渲染。
3. 地面拟合、轨迹平面化、静态地图生成。
4. V2 free_space、map_quality、参数搜索。
5. V3 bev_alignment、Depth Anything wrapper、keyframe selection、dynamic mask、scale alignment、submap fusion、occupancy_from_dense。
6. 各阶段脚本和中文报告生成。
```

---

## 19. 参考资料与出处

本知识库参考了本仓库的真实文档、代码与以下公开资料：

```text
DPVO:
  https://arxiv.org/abs/2208.04726
  https://github.com/princeton-vl/DPVO

ORB-SLAM3:
  https://arxiv.org/abs/2007.11898
  https://github.com/UZ-SLAMLab/ORB_SLAM3

VGGT:
  https://arxiv.org/abs/2503.11651
  https://github.com/facebookresearch/vggt

Depth Anything V2:
  https://arxiv.org/abs/2406.09414
  https://github.com/DepthAnything/Depth-Anything-V2

ScaRF-SLAM:
  https://arxiv.org/abs/2606.00307v1
  https://github.com/ori-drs/ScaRF-SLAM

VINS-Fusion:
  https://github.com/HKUST-Aerial-Robotics/VINS-Fusion

SLAM 基础:
  Cadena et al., Past, Present, and Future of Simultaneous Localization and Mapping
  以及本仓库中 `系统运行总说明.md`、`src代码结构与数据流说明.md`、Route A 系列文档和 output 阶段报告。
```

---

## 20. 最后复习重点

面试前优先记住这几条：

```text
1. 主轨迹为什么选 DPVO：实测稳定，比 KV-Track3r/ORB-SLAM3 更适合当前视频。
2. VGGT 为什么被替换：假墙、缺墙、地面 inlier 低，不能做主几何。
3. 行人怎么投影：footpoint -> ray -> ground intersection -> BEV。
4. 单目尺度怎么处理：内部 DPVO 单位 + 地面高度约束 + metric depth scale。
5. V3 怎么借鉴 ScaRF：tracking/mapping 解耦，SLAM 位姿锚定前馈深度，尺度一致性，子图融合。
6. 地图质量怎么量化：active_free、unknown、collision、largest_free。
7. 工程上你做了什么：开源方法评估、模块封装、坐标校准、稠密建图、BEV 渲染、报告与指标闭环。
```

