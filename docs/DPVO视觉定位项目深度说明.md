# DPVO 视觉定位项目深度说明

本文档用于向项目负责人、研发同事和面试/评审人员解释：官方 DPVO 算法本身是如何工作的，本项目又如何把 DPVO 落地到商场室内纯视觉定位任务中。文档重点说明工程边界：本项目没有重写 DPVO 的核心网络、patch 图优化和 CUDA BA 内核，而是在其上做了 ROS2 工程封装、环境隔离、相机标定适配、输入视频适配、动态区域弱化、轨迹输出、质量评估、关键帧记忆和后续二维/三维地图接口。

## 1. 项目定位

本项目目标是在单目相机条件下，为商场室内导航系统提供一条稳定的相机位姿轨迹，并把这条轨迹作为后续 BEV 行人定位、二维栅格地图、三维俯视场景重建和机器人导航接口的基础。

整体上可以拆成四层：

1. 官方 DPVO 层：负责从连续图像中估计相机相对运动，输出相机轨迹和稀疏 patch 结构。
2. 本项目 DPVO 封装层：负责让官方 DPVO 在当前 ROS2/WSL/GPU 环境下可重复运行，并把输入、标定、输出路径固定下来。
3. 本项目增强分析层：读取 DPVO 输出轨迹和原始视频，生成关键帧、质量曲线、弱跟踪片段和几何重定位候选。
4. 后续地图/导航层：使用 DPVO 轨迹对行人、深度点云、二维栅格地图和三维俯视图做全局对齐。

一句话概括：

> DPVO 是“相机自身在哪里”的前端定位引擎；本项目代码是“如何把这个引擎接入商场导航工程”的集成层和后处理层。

## 2. 目录关系

当前公开 GitHub 整理版中，DPVO 相关代码位于：

```text
mall_visual_slam/
├── launch/
│   └── dpvo_offline.launch.py
├── src/
│   └── dpvo_localization/
│       ├── package.xml
│       ├── setup.py
│       └── dpvo_localization/
│           ├── check_dpvo_env.py
│           ├── prepare_dpvo_calib.py
│           ├── run_dpvo_video.py
│           └── dpvo_enhancement.py
└── config/
    ├── KannalaBrandt8.yaml
    ├── KannalaBrandt8_960x540.yaml
    └── KannalaBrandt8_1280x720.yaml
```

原始工作区中，官方 DPVO 源码通常放在：

```text
project code/DPVO/
├── demo.py
├── dpvo.pth
├── config/default.yaml
├── calib/custom_mall.txt
└── dpvo/
    ├── dpvo.py
    ├── net.py
    ├── patchgraph.py
    ├── stream.py
    ├── ba.py
    ├── projective_ops.py
    ├── fastba/
    ├── altcorr/
    └── lietorch/
```

公开仓库一般不上传官方完整 DPVO 源码和模型权重，原因是：

1. 官方源码有独立许可证和依赖编译流程，应保持上游边界清晰。
2. `dpvo.pth` 属于模型权重，文件较大，且需要确认权重分发许可。
3. 本项目主要体现的是工程调用、ROS2 集成、轨迹增强和地图接口，不应该把第三方上游源码混成自研代码。

## 3. 官方 DPVO 是什么

DPVO 全称通常理解为 Deep Patch Visual Odometry。它是一种基于深度网络和稀疏 patch 图优化的视觉里程计算法。传统 VO/SLAM 常以角点、描述子、直接法像素块或者光流作为前端观测，而 DPVO 把图像中的局部 patch 作为基本跟踪单元，通过神经网络预测 patch 在不同帧之间的重投影修正量和置信权重，再使用 bundle adjustment 对相机位姿和 patch 深度进行联合优化。

官方 DPVO 的核心思想可以归纳为：

1. 从每帧图像中抽取一定数量的局部 patch。
2. 为每个 patch 维护一个中心位置、局部图像特征、逆深度和颜色。
3. 在相邻帧和局部时间窗口中建立 patch 到 frame 的观测边。
4. 使用神经网络根据相关性体预测重投影 residual 的修正量 delta 和权重 weight。
5. 使用 BA 在李群 SE(3) 上优化相机位姿，同时优化 patch 逆深度。
6. 在线维护滑动窗口，剔除弱关键帧，必要时通过 loop closure 或 global BA 改善轨迹一致性。

它不是普通的“检测特征点 + 匹配 + PnP”流程。DPVO 更像一个深度学习驱动的稀疏直接法/patch 法 VO：网络负责更鲁棒地估计 patch 运动和置信度，几何优化负责把这些观测转化为全局一致的相机轨迹。

## 4. 官方 DPVO 的输入输出

### 4.1 输入

官方 `demo.py` 主要需要：

```text
--imagedir   输入图片目录或视频文件
--calib      相机内参文件
--network    DPVO 网络权重，例如 dpvo.pth
--stride     抽帧间隔
--skip       起始跳过帧数
--config     算法配置，例如 config/default.yaml
```

相机标定文件格式是纯文本：

```text
fx fy cx cy k1 k2 p1 p2 k3
```

其中前四个是针孔相机内参，后面是畸变参数。本项目使用 `prepare_dpvo_calib.py` 从 ORB-SLAM3 的 YAML 标定文件中提取这些数值，写成 DPVO 可读的格式。

### 4.2 输出

官方 `demo.py` 支持：

```text
--save_trajectory   保存 TUM 格式轨迹
--plot              保存轨迹 PDF 图
--save_ply          保存稀疏 patch 点云
--save_colmap       保存 COLMAP 风格结果
```

TUM 轨迹格式如下：

```text
timestamp tx ty tz qx qy qz qw
```

在本项目里，DPVO 输出轨迹会作为后续行人 BEV 投影、深度点云融合、二维栅格地图生成和全局地图显示的位姿基准。

## 5. 官方 DPVO 的核心流程

### 5.1 读图与预处理

官方入口 `demo.py` 会根据输入是目录还是视频，选择：

```python
image_stream(queue, imagedir, calib, stride, skip)
video_stream(queue, imagedir, calib, stride, skip)
```

`dpvo/stream.py` 的作用是：

1. 读取相机内参 `fx, fy, cx, cy`。
2. 如果有畸变参数，则用 OpenCV `cv2.undistort` 去畸变。
3. 对视频帧做缩放和裁剪，使图像宽高能被 16 整除。
4. 把图像和缩放后的内参放入队列，供 DPVO 主线程消费。

本项目对 `stream.py` 做过工程适配：当输入视频名中包含 `_bev` 或提供 mask 源时，会识别黄/绿语义覆盖区域，并把这些区域填成中性灰色，减少它们对 DPVO patch 特征和运动估计的干扰。这个改动不是 DPVO 算法核心，而是为了处理已经叠加过 SAM/BEV 可视化遮罩的视频。

### 5.2 初始化 DPVO 对象

官方 `demo.py` 中的核心代码逻辑是：

```python
slam = DPVO(cfg, network, ht=H, wd=W, viz=viz)
slam(t, image, intrinsics)
```

第一次读到图像时，创建 `DPVO` 对象。DPVO 对象内部会：

1. 加载 `dpvo.pth` 网络权重。
2. 创建 `VONet` 网络。
3. 创建 `PatchGraph`，存储帧、patch、位姿、逆深度、因子边和优化状态。
4. 初始化 CUDA 张量缓存，例如图像特征 `fmap`、patch 特征 `gmap`、上下文特征 `imap`。
5. 根据配置决定是否开启 viewer、loop closure、classic loop closure 等。

### 5.3 Patch 抽取

DPVO 的基本跟踪单元不是完整图像，也不是传统 ORB 角点，而是 patch。官方 `net.py` 中 `Patchifier` 做三件事：

1. 使用 CNN encoder 提取图像特征。
2. 选取 patch 中心点。默认配置是随机采样，也可使用梯度偏置采样。
3. 在中心点周围取局部特征和 patch 几何变量。

默认配置：

```yaml
PATCHES_PER_FRAME: 96
CENTROID_SEL_STRAT: 'RANDOM'
```

每帧大约抽取 96 个 patch。每个 patch 维护：

```text
x, y, inverse_depth
```

其中 `inverse_depth` 是逆深度。逆深度比深度更适合单目视觉优化，因为远处物体深度可能非常大，而逆深度在远处趋近 0，数值更稳定。

### 5.4 PatchGraph

`dpvo/patchgraph.py` 维护了 DPVO 的局部图结构：

```text
frames:      当前滑动窗口中的帧
patches:     每帧抽取的 patch
poses:       每帧相机位姿，SE(3) 格式
intrinsics:  每帧内参
edges:       patch 到 frame 的投影观测关系
points:      patch 反投影得到的稀疏三维点
```

其中关键索引是：

```text
ii: patch 所属源帧索引
jj: patch 被投影到的目标帧索引
kk: patch 全局索引
```

一条边可以理解为：

> 第 `ii` 帧中的第 `kk` 个 patch，被当前位姿和深度投影到第 `jj` 帧时，应该落在图像的某个位置。

如果投影位置和网络预测的目标位置不一致，就产生重投影误差。后端 BA 会最小化这些误差。

### 5.5 相机位姿与 SE(3)

DPVO 使用李群 SE(3) 表达相机刚体变换。一个相机位姿可以写成：

```text
T = [ R  t ]
    [ 0  1 ]
```

其中：

```text
R 是 3x3 旋转矩阵
t 是 3x1 平移向量
```

三维点从源帧投影到目标帧，大致遵循：

```text
X_i = backproject(u_i, d_i, K)
X_j = T_j * inv(T_i) * X_i
u_j = project(K, X_j)
```

其中：

```text
u_i: patch 在源图像中的像素坐标
d_i: patch 深度或逆深度
K:   相机内参矩阵
T_i: 源帧位姿
T_j: 目标帧位姿
u_j: 投影到目标图像的像素坐标
```

单目视觉中没有绝对尺度，所以 DPVO 原生输出的平移尺度一般是相对尺度。后续如果要得到米制地图，需要通过相机高度、地面平面、已知长度、CAD 地图或其他传感器做尺度对齐。

### 5.6 网络更新：delta 和 weight

在每次优化前，DPVO 会先把当前估计的 patch 投影到目标帧，然后计算局部相关性：

```python
coords = self.reproject()
corr = self.corr(coords)
ctx = self.imap[:, self.pg.kk % (self.M * self.pmem)]
net, (delta, weight, _) = self.network.update(...)
```

这一步的含义是：

1. 根据当前位姿和深度，把 patch 投影到目标帧。
2. 在目标帧特征图上查找该 patch 附近的相关性体。
3. 网络根据相关性和上下文特征，预测一个二维修正量 `delta`。
4. 网络同时预测置信权重 `weight`。

可以把它理解为：

```text
u_target = u_projected + delta
```

`delta` 表示网络认为当前投影位置还应该向哪里修正，`weight` 表示这条观测有多可信。动态物体、模糊区域、弱纹理区域、反光区域通常应该得到更低的可信度。

### 5.7 BA 优化

DPVO 的后端优化目标可以简化理解为：

```text
min over T, z:
    sum_k w_k * || project(T_j * inv(T_i), patch_k, z_k, K) - target_k ||^2
```

其中：

```text
T:      待优化的相机位姿
z:      patch 逆深度
w_k:    网络预测的观测权重
target: 网络预测的目标位置
```

官方 `ba.py` 和 CUDA `fastba` 的作用是快速求解这个非线性最小二乘问题。代码中使用 Schur complement，把位姿变量和深度变量组织成块矩阵求解。这和传统视觉 SLAM 后端的 Bundle Adjustment 原理一致，只是观测来自神经网络辅助的 patch residual。

BA 的核心直觉是：

1. 如果相机位姿错了，同一个 patch 投影到其他帧的位置会系统性偏移。
2. 如果 patch 深度错了，不同视角下的投影偏移会随视差变化。
3. 同时调整位姿和深度，使所有可信 patch 的投影误差尽量小。

### 5.8 初始化策略

DPVO 不会在第一帧就立即输出稳定轨迹。它会等待足够运动：

```python
if self.n > 0 and not self.is_initialized:
    if self.motion_probe() < 2.0:
        self.pg.delta[self.counter - 1] = (self.counter - 2, Id[0])
        return
```

如果相邻帧运动太小，特别是单目纯旋转或几乎静止，DPVO 很难获得可靠视差，因此会先延迟初始化。默认代码中，当窗口达到约 8 帧后，会执行多轮 update：

```python
if self.n == 8 and not self.is_initialized:
    self.is_initialized = True
    for itr in range(12):
        self.update()
```

这就是为什么刚开始几帧可能没有明显位姿变化，或者轨迹前段不稳定。

### 5.9 滑动窗口和关键帧剔除

DPVO 维护一个局部滑动窗口。配置中：

```yaml
REMOVAL_WINDOW: 22
OPTIMIZATION_WINDOW: 10
PATCH_LIFETIME: 13
KEYFRAME_THRESH: 15.0
```

含义可以理解为：

1. `OPTIMIZATION_WINDOW`：每次局部 BA 主要优化最近若干帧。
2. `PATCH_LIFETIME`：patch 在多少帧内保持活跃。
3. `REMOVAL_WINDOW`：超过窗口的边会被转为 inactive 或移除。
4. `KEYFRAME_THRESH`：如果某帧和相邻关键帧之间运动太小，可能被剔除。

`keyframe()` 的逻辑是计算某些帧之间的光流运动量，如果运动太小，就将该帧从活动窗口中移除，同时保存相对位姿 `delta`，用于最后插值恢复完整轨迹。

### 5.10 终止与轨迹恢复

视频读完后，官方 `terminate()` 会：

1. 如果启用了 loop closure，添加 loop closure factors。
2. 再执行多轮 update 和 BA。
3. 把滑动窗口中保存的关键帧位姿和被剔除帧的相对位姿组合起来。
4. 输出每个输入时间戳对应的相机 pose。

最终 `demo.py` 使用 evo 写出 TUM 轨迹：

```python
file_interface.write_tum_trajectory_file(
    f"saved_trajectories/{args.name}.txt",
    trajectory
)
```

## 6. 本项目是怎么调用官方 DPVO 的

本项目入口是 ROS2 包 `dpvo_localization`，最常用命令是：

```bash
cd /home/ros/ros2_orbslam3
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ./launch/dpvo_offline.launch.py
```

launch 文件实际启动的是：

```text
package:    dpvo_localization
executable: run_dpvo_video
```

也可以直接运行：

```bash
ros2 run dpvo_localization run_dpvo_video \
  --dpvo-root "/home/ros/ros2_orbslam3/project code/DPVO" \
  --imagedir "/home/ros/ros2_orbslam3/resources/input_video.mp4_bev.mp4" \
  --calib "/home/ros/ros2_orbslam3/project code/DPVO/calib/custom_mall.txt" \
  --name mall_dpvo \
  --stride 2 \
  --save_trajectory \
  --plot
```

`run_dpvo_video.py` 不是重新实现 DPVO，而是做了下面几件事：

1. 解析 ROS2 传入的命令行参数。
2. 解析 DPVO 根目录、视频路径、标定路径、模型权重路径。
3. 设置 DPVO 专用 Python 环境。
4. 检查 PyTorch CUDA 是否可用。
5. 通过 `subprocess.run` 调用官方 `demo.py`。
6. 根据参数决定是否保存轨迹、PDF、PLY、COLMAP 输出。
7. 在 DPVO 完成后，自动调用本项目的 `dpvo_enhancement.py` 做增强分析。

核心调用关系是：

```text
ROS2 launch
  -> dpvo_localization/run_dpvo_video.py
      -> official DPVO/demo.py
          -> dpvo/stream.py
          -> dpvo/dpvo.py
          -> dpvo/net.py
          -> dpvo/ba.py + fastba CUDA
      -> dpvo_localization/dpvo_enhancement.py
```

## 7. 本项目为什么要封装环境

DPVO 依赖 CUDA、PyTorch、C++/CUDA 扩展、Pangolin viewer、evo、lietorch、altcorr、fastba 等组件。普通 ROS2 Python 环境不一定能直接加载这些库。本项目在 `run_dpvo_video.py` 中构造了独立环境：

```python
env['PYTHONPATH'] = str(root) + os.pathsep + env.get('PYTHONPATH', '')
env['CUDA_HOME'] = env.get('CUDA_HOME', '/usr/local/cuda-12.1')
env['LD_LIBRARY_PATH'] = ...
```

同时还设置：

```python
env['MPLBACKEND'] = 'Agg'
env['MPLCONFIGDIR'] = runtime_home / '.config' / 'matplotlib'
```

这样做的原因：

1. 避免 ROS2 的系统 Python 和 DPVO 的 conda Python 冲突。
2. 确保 PyTorch 能找到 CUDA 动态库。
3. 确保 DPVO 的自定义 CUDA 扩展能被 import。
4. 离线运行时不依赖桌面显示，避免 TkAgg、Pangolin、DISPLAY 引发崩溃。
5. 在 WSL 或服务器环境中能稳定保存轨迹和图像，而不是必须打开 viewer。

`check_dpvo_env.py` 用于检查：

```text
nvidia-smi
nvcc
torch
torch.cuda.is_available()
dpvo
cuda_corr
cuda_ba
lietorch_backends
evo
yacs
cv2
numpy
```

如果其中 CUDA 或 DPVO 扩展缺失，DPVO 无法正常运行。

## 8. 相机标定适配

ORB-SLAM3 配置通常是 YAML，例如：

```yaml
Camera.fx: 571.257
Camera.fy: 604.479
Camera.cx: 497.86
Camera.cy: 249.702
Camera.k1: 0.298296
Camera.k2: -0.863055
Camera.p1: 0.020866
Camera.p2: 0.002167
Camera.k3: 1.04161
```

DPVO 需要一行文本：

```text
571.257000 604.479000 497.860000 249.702000 0.298296 -0.863055 0.020866 0.002167 1.041610
```

本项目 `prepare_dpvo_calib.py` 的作用就是把 YAML 转成文本。命令示例：

```bash
ros2 run dpvo_localization prepare_dpvo_calib \
  --input config/KannalaBrandt8_960x540.yaml \
  --output "project code/DPVO/calib/custom_mall.txt" \
  --scale 1.0
```

需要注意：如果视频在进入 DPVO 前被缩放，比如 `stream.py` 中 `fx=0.5, fy=0.5`，那么内参也必须同步缩放。官方 `video_stream` 中已经做了：

```python
intrinsics = np.array([fx*.5, fy*.5, cx*.5, cy*.5])
```

这保证图像缩小一半后，投影几何仍然一致。

## 9. 本项目对输入视频的处理

本项目在商场视频中遇到一个特殊问题：有些输入视频已经叠加了 SAM/BEV 的彩色分割区域，例如黄色地面、绿色区域或动态遮挡提示。如果直接把这些彩色覆盖层送入 DPVO，网络会把覆盖层当成真实纹理，导致 patch 跟踪偏移。

因此在 `stream.py` 和 `dpvo_enhancement.py` 中，项目增加了彩色遮挡 mask 识别：

```text
yellow: HSV [15,70,70] 到 [42,255,255]
green:  HSV [40,60,60] 到 [95,255,255]
```

然后对这些 mask 区域做形态学闭运算和膨胀，并填成灰色：

```python
cleaned[mask > 0] = 127
```

这样做的意图不是“让 DPVO 理解语义”，而是：

1. 减少可视化覆盖层对 patch 特征的污染。
2. 避免动态行人 mask、地面 mask 被误认为稳定纹理。
3. 尽量让 DPVO 关注真实场景中的墙、柱子、货架、地砖纹理。

## 10. 本项目 DPVO 增强模块

官方 DPVO 输出的是轨迹和可选稀疏点云，但工程落地还需要回答：

1. 哪些时间段跟踪质量好？
2. 哪些时间段轨迹可能漂移或停滞？
3. 哪些帧适合作为关键帧？
4. 后续如果要重定位，应该和哪些历史关键帧匹配？
5. 输出能否被二维地图和 BEV 行人定位模块稳定消费？

因此本项目增加了 `dpvo_enhancement.py`。它的输入是：

```text
DPVO TUM 轨迹
原始视频或处理后视频
相机标定文件
stride/skip
```

输出包括：

```text
summary.json
tracking_quality.csv
tracking_quality.png
keyframes/keyframes.json
keyframes/images/*.png
keyframes/features/*.npz
relocalization_results.json
增强可视化视频和截图
```

### 10.1 轨迹样本结构

增强模块把每条 TUM 轨迹转成 `PoseSample`：

```text
sample_index
timestamp
position
quaternion_xyzw
source_frame
source_time_sec
translation_step
rotation_step_deg
blur_laplacian
mask_ratio
stale_count
quality_score
quality_state
```

其中 `source_frame` 根据 DPVO 的 `stride` 和 `skip` 反推：

```text
source_frame = skip + (sample_index + 1) * stride - 1
```

这样就能把 DPVO 的某个轨迹点对应回原视频帧。

### 10.2 质量评分

增强模块综合四类信号：

1. 模糊度：Laplacian variance。
2. 运动跳变：相邻位姿平移和旋转是否异常。
3. mask 比例：画面中被彩色遮挡/动态区域覆盖的比例。
4. stale count：位姿长时间几乎不变的累计次数。

质量分数大致是：

```text
quality =
    0.35 * blur_score
  + 0.25 * motion_score
  + 0.20 * mask_score
  + 0.20 * stale_score
```

然后分成：

```text
good: quality >= 0.65
weak: 0.35 <= quality < 0.65
lost: quality < 0.35
```

这个评分不是官方 DPVO 的内部置信度，而是工程层的质量诊断，目的是帮助后续模块避开低质量轨迹段。

### 10.3 关键帧选择

关键帧插入依据：

1. 第一帧必须插入。
2. 质量低于阈值的帧不插入。
3. 与上一个关键帧的平移超过阈值则插入。
4. 与上一个关键帧的旋转超过阈值则插入。
5. 距离上一个关键帧时间太久则强制插入。

自动阈值估计逻辑：

```text
translation_threshold = max(median_translation * 12, total_path / 80, 1e-5)
rotation_threshold    = max(median_rotation * 8, 3 deg)
```

关键帧用于：

1. 后续局部地图构建。
2. 深度估计抽帧。
3. 行人 BEV 投影的稳定位姿锚点。
4. 重定位候选库。

### 10.4 ORB 特征和几何验证

增强模块会为每个关键帧保存 ORB 特征。对弱跟踪片段，会寻找可能相关的历史关键帧，并做 Essential Matrix RANSAC 验证：

```text
当前弱帧 ORB 特征
  -> 与候选关键帧 ORB 描述子匹配
  -> cv2.findEssentialMat
  -> 统计 RANSAC 内点数和内点比例
  -> 输出 relocalization_results.json
```

这一步不是完整闭环，也不是主动 PnP 重定位。它的工程意义是：

> 判断某个弱跟踪片段是否还能和历史关键帧建立稳定几何关系，为后续真正的 2D-3D PnP 重定位提供候选。

完整 PnP 还需要把关键帧 ORB 特征与 DPVO 或稠密重建得到的三维点绑定起来，形成：

```text
2D 当前图像特征点 <-> 3D 地图点
```

然后通过 `solvePnPRansac` 估计当前相机位姿。

## 11. DPVO 与 ORB-SLAM3 的区别

本项目中同时接触过 DPVO 和 ORB-SLAM3，两者的定位思想不同。

| 维度 | DPVO | ORB-SLAM3 |
|---|---|---|
| 前端观测 | 深度网络驱动的 patch residual | ORB 角点和描述子匹配 |
| 后端优化 | patch 位姿和逆深度 BA | 关键帧、地图点 BA 和 pose graph |
| 动态鲁棒性 | 网络和权重对动态有一定容忍 | 依赖特征筛选，动态场景易污染地图点 |
| 初始化 | 需要足够视差和运动 | 单目同样需要初始化视差 |
| 回环能力 | 可选 loop closure，视代码配置 | 完整回环和重定位机制成熟 |
| 工程优点 | 商场视频轨迹更平滑、鲁棒 | SLAM 系统完整，可复用地图点和回环 |
| 工程短板 | 绝对尺度和地图语义需外部补充 | 单目动态行人、弱纹理、反光环境可能更脆弱 |

在当前商场室内项目中，DPVO 的相机轨迹表现优于 KV-Track3r 和部分 ORB-SLAM3 实验结果，因此采用 DPVO 作为主定位前端。

## 12. DPVO 与后续地图模块的数据关系

DPVO 输出的位姿是后续模块的坐标主线：

```text
视频帧
  -> DPVO
      -> camera trajectory
          -> BEV 行人投影
          -> 深度图反投影
          -> 稠密/半稠密点云融合
          -> 2D occupancy grid
          -> 商场导航路线图
```

其中最关键的坐标变换是：

```text
像素坐标 -> 相机坐标 -> 世界坐标 -> 地图坐标
```

像素到相机坐标：

```text
X_c = z * inv(K) * [u, v, 1]^T
```

相机坐标到世界坐标：

```text
X_w = T_wc * X_c
```

世界坐标到二维地图：

```text
map_x = round((X_w.x - origin_x) / resolution)
map_y = round((X_w.y - origin_y) / resolution)
```

如果出现俯视图方向反了、左右镜像、转向和真实世界相反，通常不是 DPVO 本身错误，而是坐标系展开时用了错误的轴。例如需要明确：

```text
相机系: x 右, y 下, z 前
机器人/地图系: x 前, y 左, z 上
图像显示系: x 右, y 下
```

项目中后续已经引入过 `mirror_y`、轨迹 flatten、alignment_selected 等校正步骤，本质都是在解决世界坐标到二维地图坐标的轴向约定问题。

## 13. 单目 DPVO 的尺度问题

DPVO 在单目输入下无法直接恢复真实米制尺度。这是物理限制，不是某个代码缺陷。单目相机只根据图像投影估计结构和运动，满足下面的尺度不变性：

```text
如果 T 和 X 是一个可行解，
那么 s*T.translation 和 s*X 也是一个投影等价解。
```

因此单目轨迹的形状可能是对的，但长度单位不是米。要用于商场导航，必须引入尺度来源：

1. 相机固定安装高度。
2. 平坦地面假设。
3. 已知地砖尺寸、门宽、通道宽度。
4. 与 CAD/真实地图配准。
5. 与轮速计、IMU、LiDAR、RGB-D 的尺度融合。

当前项目 Route A 中曾使用：

```text
Depth Anything V2 Metric Indoor
+ 地面尺度估计
+ 全局 scale
+ DPVO 位姿
```

其目的就是把 DPVO 相对轨迹和深度点云拉回近似米制。

## 14. DPVO 在商场场景中的优势

商场环境对视觉 SLAM 很难，主要难点包括：

1. 大量动态行人。
2. 玻璃、反光地砖和橱窗。
3. 弱纹理墙面。
4. 重复纹理，如货架、灯带、地砖。
5. 相机运动可能包含纯旋转、急转、遮挡。

DPVO 在本项目中表现较好的原因：

1. patch 级别跟踪比单纯稀疏角点更充分利用局部图像块信息。
2. 神经网络 update operator 对光照、模糊、非理想纹理有更强鲁棒性。
3. 每条观测带有 weight，弱观测可以在 BA 中被降低影响。
4. 滑动窗口 BA 能持续修正最近一段轨迹。
5. 在动态区域被 mask 后，DPVO 更容易锁定静态背景。

但它仍然不能自动解决：

1. 绝对尺度。
2. 完整可导航地图。
3. 行人身份跟踪。
4. 玻璃物理属性判断。
5. 语义可通行区域识别。
6. 长时间全局一致回环。

这些需要后续模块补齐。

## 15. 本项目已经完成的 DPVO 工程工作

### 15.1 ROS2 包封装

完成 `dpvo_localization` 包：

```text
check_dpvo_env
prepare_dpvo_calib
run_dpvo_video
```

这样用户可以通过 ROS2 标准方式运行 DPVO，而不需要每次手动进入 DPVO 目录执行复杂命令。

### 15.2 launch 文件

完成 `launch/dpvo_offline.launch.py`，把常用参数固化：

```text
输入视频: resources/input_video.mp4_bev.mp4
标定文件: custom_mall.txt
运行名:   mall_dpvo
stride:   2
输出:     trajectory + plot
```

### 15.3 环境隔离

在 `run_dpvo_video.py` 中隔离：

```text
DPVO_PYTHON
PYTHONPATH
LD_LIBRARY_PATH
CUDA_HOME
HOME
MPLBACKEND
MPLCONFIGDIR
DISPLAY
WAYLAND_DISPLAY
```

这解决了 ROS2 Python、conda Python、CUDA、Pangolin、matplotlib 后端之间的冲突。

### 15.4 标定转换

通过 `prepare_dpvo_calib.py` 实现 ORB-SLAM3 YAML 到 DPVO 文本标定的转换，保证两个定位系统使用同一组相机内参。

### 15.5 轨迹保存和增强

在 `run_dpvo_video.py` 中，如果启用增强模块，会自动强制保存轨迹：

```python
if enhance_enabled:
    args.save_trajectory = True
```

这样 `dpvo_enhancement.py` 可以读取：

```text
project code/DPVO/saved_trajectories/<name>.txt
```

### 15.6 关键帧记忆与弱跟踪诊断

`dpvo_enhancement.py` 已经实现：

1. 轨迹读取。
2. 视频帧反查。
3. 模糊、遮挡、运动跳变评估。
4. 关键帧自动选取。
5. ORB 特征保存。
6. 弱跟踪片段分组。
7. Essential Matrix RANSAC 几何验证。
8. 可视化视频和截图。

这部分是本项目相对官方 DPVO 的重要工程增量。

## 16. 当前代码和官方代码的边界

需要明确区分：

### 16.1 官方 DPVO 负责

```text
CNN patch 特征提取
patch graph 维护
SE(3) 位姿状态
patch 逆深度状态
相关性体计算
网络 update operator
CUDA BA 优化
关键帧剔除
轨迹输出
可选 viewer 和 loop closure
```

### 16.2 本项目负责

```text
ROS2 运行入口
DPVO 环境配置
相机标定转换
商场视频路径参数
SAM/BEV 彩色遮挡区域弱化
DPVO 输出轨迹读取
轨迹质量评分
关键帧记忆
ORB 特征缓存
重定位候选几何验证
BEV/地图模块接口
公司公开仓库文档整理
```

> 我们没有把 DPVO 当成黑盒脚本直接跑，而是把它工程化成当前系统的视觉定位模块：固定输入输出协议，解决环境和标定问题，增加质量评估和关键帧记忆，并把轨迹作为后续商场导航 BEV 地图、行人定位和可通行区域重建的坐标基准。

## 17. 常见问题解释

### 17.1 为什么 DPVO 比某些 SLAM 结果更好？

因为当前商场视频有动态行人、反光、弱纹理和遮挡。DPVO 使用学习到的 patch 特征和置信权重，比传统特征匹配对局部异常更有容忍度。同时它用滑动窗口 BA 持续优化最近位姿，不只是逐帧累积光流。

### 17.2 为什么 DPVO 轨迹不是米制？

因为单目视觉没有绝对尺度。DPVO 可以恢复轨迹形状，但平移长度需要通过地面高度、已知尺寸或地图配准校正。

### 17.3 为什么需要 stride？

stride 控制抽帧。商场视频帧率较高时，相邻帧运动太小会导致视差不足，同时计算量也大。`stride=2` 可以降低计算量，并让相邻输入帧之间有更明显运动。但 stride 太大又会导致匹配困难，因此需要折中。

### 17.4 为什么一开始几帧轨迹不稳定？

DPVO 需要足够运动和至少若干帧构成初始化窗口。代码中默认到约 8 帧后才进入稳定初始化，并执行多轮 update。

### 17.5 为什么动态行人会影响定位？

DPVO 本身不知道“人”这个语义。它只看到图像 patch。如果大量 patch 落在人身上，而人相对环境运动，这些 patch 的运动不满足静态世界假设，会污染位姿估计。因此本项目在输入层和后续地图层都使用 mask 或过滤策略降低动态物体影响。

### 17.6 为什么 DPVO 稀疏点云不能直接做导航地图？

DPVO 的点云来自稀疏 patch，不是为稠密地图设计的。它可以辅助可视化和局部结构理解，但不足以直接生成完整墙体、障碍物和可通行区域。导航地图需要额外的深度估计、稠密融合、语义分割、occupancy grid 或 ScaRF/NeRF 类重建方法。

### 17.7 为什么要做 `dpvo_enhancement.py`？

官方 DPVO 输出轨迹，但工程系统需要知道轨迹哪里可靠、哪里可能漂移、哪些帧适合做地图关键帧。增强模块补齐了这些工程诊断信息。

## 18. 后续改进方向

### 18.1 接入实时 ROS2 图像流

当前 `run_dpvo_video.py` 主要是离线视频入口。下一步可以封装在线节点：

```text
/camera/image_raw -> DPVO node -> /dpvo/odom, /tf, /dpvo/keyframes
```

实现方式：

1. 把官方 `DPVO` 对象常驻在 ROS2 node 内。
2. 每收到一帧图像，执行一次 `slam(t, image, intrinsics)`。
3. 按固定频率发布 `nav_msgs/Odometry`。
4. 按关键帧策略发布关键帧图像和位姿。

### 18.2 真正的 PnP 重定位

当前几何验证只做到 2D-2D Essential Matrix。下一步需要：

1. 建立关键帧 ORB 特征与三维地图点关系。
2. 当前帧 ORB 匹配历史关键帧。
3. 通过 2D-3D `solvePnPRansac` 求当前位姿。
4. 在 DPVO 弱跟踪或丢失时，把 PnP 位姿作为重初始化参考。

### 18.3 尺度约束

可以引入：

1. 相机高度约束。
2. 地面平面约束。
3. 已知商场通道宽度。
4. CAD 地图配准。
5. IMU/VIO 融合。

目标是让 DPVO 轨迹从相对尺度变成米制尺度。

### 18.4 与二维栅格地图联动

DPVO 负责相机轨迹，地图模块负责：

1. 单目深度估计。
2. 动态行人 mask。
3. 点云融合。
4. 高度过滤。
5. occupancy grid 更新。
6. 行人位置合法性过滤。

尤其对于行人显示，需要加入物理约束：

```text
只显示在 free space 或 unknown-free 边界附近的行人
过滤落入 wall/occupied 栅格深处的行人
过滤离相机过远、深度不稳定、轨迹 ID 跳变的行人
```

### 18.5 与 ScaRF-SLAM/NeRF 类方法对比

DPVO 更适合实时位姿估计，ScaRF-SLAM/NeRF 类方法更适合稠密或半稠密场景重建。合理架构是：

```text
DPVO: 高频定位前端
Depth/ScaRF/NeRF: 低频地图重建后端
YOLO/ByteTrack: 动态行人感知
Occupancy Grid: 导航可通行表达
```

## 19. 面向评审的工作归纳

如果需要总结个人/团队在 DPVO 方向完成的工作，可以表述为：

1. 调研并复现官方 DPVO，在本地 WSL/ROS2/CUDA 环境中跑通商场视频轨迹输出。
2. 将官方 DPVO 封装为 ROS2 包 `dpvo_localization`，提供环境检查、标定转换和离线视频运行入口。
3. 解决 DPVO 与 ROS2、conda、CUDA、Pangolin、matplotlib 之间的环境冲突，使其能够稳定离线运行。
4. 对商场视频中的语义覆盖区域做 mask 弱化，减少动态行人与可视化遮罩对 DPVO patch 跟踪的干扰。
5. 基于 DPVO 输出轨迹开发增强分析模块，提供轨迹质量评估、关键帧记忆、ORB 特征缓存和重定位候选几何验证。
6. 将 DPVO 轨迹作为后续 BEV 行人定位、二维栅格地图、三维俯视场景重建和商场导航系统的全局位姿输入。
7. 明确 DPVO 的工程边界：它负责相机位姿，不负责完整语义地图、行人 ID 追踪和米制尺度恢复，这些由后续模块补齐。

## 20. 快速运行命令

### 20.1 检查 DPVO 环境

```bash
cd /home/ros/ros2_orbslam3
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run dpvo_localization check_dpvo_env
```

### 20.2 生成 DPVO 标定文件

```bash
ros2 run dpvo_localization prepare_dpvo_calib \
  --input config/KannalaBrandt8_960x540.yaml \
  --output "project code/DPVO/calib/custom_mall.txt" \
  --scale 1.0
```

### 20.3 离线运行 DPVO

```bash
ros2 launch ./launch/dpvo_offline.launch.py
```

### 20.4 直接运行 DPVO 包装入口

```bash
ros2 run dpvo_localization run_dpvo_video \
  --dpvo-root "project code/DPVO" \
  --imagedir "resources/input_video.mp4_bev.mp4" \
  --calib "project code/DPVO/calib/custom_mall.txt" \
  --name mall_dpvo \
  --stride 2 \
  --save_trajectory \
  --plot
```

### 20.5 只运行增强分析

```bash
ros2 run dpvo_localization run_dpvo_video \
  --dpvo-root "project code/DPVO" \
  --imagedir "resources/input_video.mp4_bev.mp4" \
  --calib "project code/DPVO/calib/custom_mall.txt" \
  --name mall_dpvo \
  --stride 2 \
  --only_enhance
```

## 21. 交付物说明

本说明文档建议在 GitHub 主目录 `docs/` 下保留两种格式：

```text
docs/DPVO视觉定位项目深度说明.md
docs/DPVO视觉定位项目深度说明.docx
```

其中：

1. `.md` 适合 GitHub 在线阅读和版本管理。
2. `.docx` 适合公司内部汇报、发送给非研发负责人或直接打印。

## 22. 结论

DPVO 在本项目中的角色是高频、鲁棒的单目视觉定位前端。官方 DPVO 提供了深度 patch 特征、相关性更新、SE(3) 位姿优化和 CUDA BA；本项目则把它封装成可运行、可诊断、可输出、可接入后续商场导航系统的工程模块。

当前项目真正的工程价值不在于改写 DPVO 神经网络本身，而在于：

1. 让官方 DPVO 能在本地 ROS2 工程中稳定运行。
2. 让相机标定、视频输入、轨迹输出形成固定协议。
3. 让 DPVO 输出能被地图、行人定位和导航模块使用。
4. 为弱跟踪、重定位、尺度对齐和地图构建留下可扩展接口。

因此，面对项目评审时可以明确说明：

> 我们采用 DPVO 作为商场纯视觉导航的定位前端，并围绕它完成了工程化封装、运行环境适配、相机标定转换、动态遮挡处理、轨迹质量诊断、关键帧记忆和地图接口设计。后续二维栅格地图和论文Reinforced Cross-Modal Matching and Self-Supervised Imitation Learning for Vision-Language Navigation Fig.1 类俯视场景效果，应继续在 DPVO 稳定位姿之上，引入更可靠的稠密重建、语义分割和 occupancy grid 后端。
