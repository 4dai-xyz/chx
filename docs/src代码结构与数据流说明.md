# src 代码结构、运行原理与数据流说明

本文档专门解释 `/home/ros/ros2_orbslam3/src` 目录中的代码：每个模块负责什么、如何调用外部开源代码、数据如何流通、launch 文件如何把模块串起来，以及它们和 DPVO / ORB-SLAM3 / KV-Tracker / people BEV 的关系。

工程根目录：

```text
/home/ros/ros2_orbslam3
```

当前 `src/` 下的主要目录：

```text
src/
├── dpvo_localization/       # ROS2 Python 包：封装 DPVO 视频运行和后处理
├── video_publisher/         # ROS2 Python 包：把本地视频发布成 ROS Image topic
├── orbslam3_wrapper/        # ROS2 C++ 包：调用官方 ORB-SLAM3 动态库
├── people_bev_tracker/      # 离线 Python 工程：行人检测/跟踪 + BEV 投影
└── KV-tracker/              # 离线 Python 工程：官方 KV-Tracker 外部封装与论文复现
```

注意：

```text
VGGT 目前主要不在 src/ 下封装。
VGGT 的封装脚本在根目录 scripts/ 中，例如 scripts/run_vggt_video.py、scripts/run_vggt_scene.py、scripts/open_vggt_viser.py。
```

---

## 1. 总体分层

可以把当前工程理解成三层。

### 1.1 开源算法核心层

这些是外部开源项目，负责真正的算法核心：

```text
project code/DPVO
  -> DPVO 官方/开源视觉里程计算法

project code/ORB_SLAM3-master
  -> ORB-SLAM3 官方/开源 SLAM 算法

project code/KV-tracker/kv_tracker-main
  -> KV-Tracker 官方/开源代码

project code/VGGT/vggt-main
  -> VGGT 官方/开源代码

Ultralytics YOLO
  -> people_bev_tracker 中的行人检测、分割和跟踪
```

### 1.2 本工程封装层

这些是 `src/` 中你自己工程里的包装代码：

```text
src/dpvo_localization
  -> 用 ROS2 console script 调用 project code/DPVO/demo.py

src/orbslam3_wrapper
  -> 用 C++ ROS2 节点链接 project code/ORB_SLAM3-master/lib/libORB_SLAM3.so

src/video_publisher
  -> 读取 resources/input_video.mp4，发布 /camera/image_raw

src/people_bev_tracker
  -> 读取视频 + DPVO 轨迹 + YOLO 跟踪结果，输出 BEV 视频和 JSON

src/KV-tracker
  -> 不改官方 KV-Tracker，通过 subprocess/sys.path 调用官方 main.py，并导出 JSON/TUM/PLY
```

### 1.3 可视化与结果层

```text
RViz2
  -> 显示 /camera/image_raw、/slam/debug_image、/slam/map_points、/slam/odom

Pangolin
  -> ORB-SLAM3 官方 viewer

Rerun
  -> KV-Tracker 论文风格可视化

Viser
  -> VGGT 点云和相机位姿网页可视化

OpenCV VideoWriter
  -> people_bev_tracker 输出 bev_tracking.mp4 / debug_overlay.mp4
```

---

## 2. 一张总数据流图

```text
resources/input_video.mp4
        │
        ├──────────────────────────────────────────────┐
        │                                              │
        ▼                                              ▼
src/video_publisher                          src/dpvo_localization
        │                                      run_dpvo_video.py
        │                                              │
        │ /camera/image_raw                            │ 调用
        │ /yolo/dynamic_mask                           ▼
        ▼                                      project code/DPVO/demo.py
src/orbslam3_wrapper                                  │
  mono_node.cpp                                       ▼
        │                                      output/dpvo/trajectory_tum.txt
        │ 调用官方 ORB_SLAM3::System                   │
        ▼                                              │
project code/ORB_SLAM3-master/lib/libORB_SLAM3.so      │
        │                                              │
        ├── /slam/pose                                 │
        ├── /slam/odom                                 │
        ├── /slam/map_points                           │
        └── /slam/debug_image                          │
                                                       ▼
                                             src/people_bev_tracker
                                             offline_pipeline.py
                                                       │
                                                       │ 调用 Ultralytics YOLO track
                                                       ▼
                                             output/people_bev/
                                               ├── bev_tracking.mp4
                                               ├── debug_overlay.mp4
                                               ├── people_tracks.json
                                               └── camera_trajectory.json


resources/input_video.mp4
        │
        ▼
src/KV-tracker/scripts/run_official_kv_tracker.py
        │
        │ subprocess 调用，不改官方源码
        ▼
project code/KV-tracker/kv_tracker-main/main.py
        │
        ▼
output/kv_track3r_repro/
  ├── trajectory_tum.txt
  ├── trajectory.json
  ├── keyframes.json
  ├── confidence.json
  └── summary.md
```

---

## 3. `src/video_publisher`

### 3.1 模块职责

`src/video_publisher` 是 ROS2 Python 包，负责把本地视频读成 ROS2 图像消息。

主要代码：

```text
src/video_publisher/video_publisher/video_publisher_node.py
```

ROS2 入口：

```text
video_publisher_node = video_publisher.video_publisher_node:main
```

定义在：

```text
src/video_publisher/setup.py
```

### 3.2 输入

默认输入：

```text
resources/input_video.mp4
```

可选 mask 来源：

```text
resources/input_video.mp4_bev.mp4
```

`input_video.mp4_bev.mp4` 中黄色/绿色 overlay 会被 HSV 阈值抠成 mask，用于屏蔽动态区域。

### 3.3 输出 topic

```text
/camera/image_raw
  -> 主图像，默认 bgr8

/camera/image_raw_full
  -> 兼容旧节点，通常和 /camera/image_raw 内容相同

/yolo/dynamic_mask
  -> mono8 mask，255 表示需要屏蔽的动态/覆盖区域
```

### 3.4 被谁使用

```text
/camera/image_raw
  -> src/orbslam3_wrapper/src/mono_node.cpp 订阅
  -> RViz2 可直接显示

/yolo/dynamic_mask
  -> src/orbslam3_wrapper/src/mono_node.cpp 订阅，用于动态区域屏蔽
```

### 3.5 launch 关系

只启动视频：

```text
launch/video_only.launch.py
```

同时启动视频和 ORB-SLAM3：

```text
launch/orbslam3_video.launch.py
```

### 3.6 数据流

```text
cv2.VideoCapture(resources/input_video.mp4)
  -> frame_bgr
  -> cv_bridge.cv2_to_imgmsg(frame_bgr, "bgr8")
  -> publish /camera/image_raw

cv2.VideoCapture(resources/input_video.mp4_bev.mp4)
  -> HSV 阈值提取黄色/绿色
  -> mono8 mask
  -> publish /yolo/dynamic_mask
```

### 3.7 环境注意

该节点依赖 ROS 的 `cv_bridge`。必须在干净 ROS Python 环境运行：

```bash
unset PYTHONPATH
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source install/setup.bash
```

不要让它加载 conda `dpvo` 环境里的 NumPy 2.x。

---

## 4. `src/orbslam3_wrapper`

### 4.1 模块职责

`src/orbslam3_wrapper` 是 ROS2 C++ 包。它不是重写 ORB-SLAM3，而是：

```text
ROS Image topic
  -> OpenCV Mat
  -> 调用官方 ORB_SLAM3::System::TrackMonocular()
  -> 发布 ROS 位姿、状态、点云和调试图像
```

主要代码：

```text
src/orbslam3_wrapper/src/mono_node.cpp
```

编译配置：

```text
src/orbslam3_wrapper/CMakeLists.txt
```

### 4.2 如何调用官方 ORB-SLAM3

官方代码路径：

```text
project code/ORB_SLAM3-master
```

官方动态库：

```text
project code/ORB_SLAM3-master/lib/libORB_SLAM3.so
```

官方头文件：

```text
project code/ORB_SLAM3-master/include/System.h
project code/ORB_SLAM3-master/include/CameraModels/
```

CMake 中做了三件关键事。

第一，读取官方路径：

```cmake
if(DEFINED ENV{ORB_SLAM3_DIR})
  set(ORB_SLAM3_DIR "$ENV{ORB_SLAM3_DIR}" ...)
endif()

if(DEFINED ENV{ORB_SLAM3_LIBRARY})
  set(ORB_SLAM3_LIBRARY "$ENV{ORB_SLAM3_LIBRARY}" ...)
endif()
```

第二，把官方 include 加进来：

```cmake
target_include_directories(mono_node PRIVATE
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2
  ${ORB_SLAM3_DIR}/Thirdparty/g2o
)
```

第三，链接官方动态库：

```cmake
target_link_libraries(mono_node
  ${ORB_SLAM3_LIBRARY}
  ${Pangolin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  pthread
)
```

所以 `mono_node` 运行时真正调用的是官方 `libORB_SLAM3.so`。

### 4.3 `mono_node.cpp` 内部运行逻辑

#### 4.3.1 创建官方 ORB-SLAM3 系统

代码中包含官方头文件：

```cpp
#include "System.h"
```

初始化官方系统：

```cpp
SLAM_ = std::make_shared<ORB_SLAM3::System>(
    vocab_path,
    settings_path,
    ORB_SLAM3::System::MONOCULAR,
    enable_viewer
);
```

参数含义：

```text
vocab_path:
  ORBvoc.txt 词袋文件

settings_path:
  ORB-SLAM3 相机标定 YAML，例如 config/KannalaBrandt8_960x540.yaml

ORB_SLAM3::System::MONOCULAR:
  单目模式

enable_viewer:
  是否启动官方 Pangolin viewer
```

#### 4.3.2 订阅 ROS 图像和 mask

```cpp
sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw", 10,
    std::bind(&MonoNode::ImageCallback, this, std::placeholders::_1));

sub_mask_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/yolo/dynamic_mask", 10,
    std::bind(&MonoNode::MaskCallback, this, std::placeholders::_1));
```

#### 4.3.3 图像进入官方 ORB-SLAM3

```text
ROS Image
  -> cv_bridge::toCvShare(msg, "bgr8")
  -> cv::cvtColor(BGR, GRAY)
  -> 可选 applyDynamicMask()
  -> SLAM_->TrackMonocular(im_gray, timestamp)
```

核心调用：

```cpp
Sophus::SE3f pose = SLAM_->TrackMonocular(im_gray, timestamp);
```

这句进入官方 ORB-SLAM3 内部，完成特征提取、跟踪、局部建图、关键帧维护、地图点维护等。

#### 4.3.4 wrapper 发布什么

```text
/slam/pose
  -> geometry_msgs/PoseStamped

/slam/odom
  -> nav_msgs/Odometry

/slam/tracking_state
  -> std_msgs/Int32

/slam/map_points
  -> sensor_msgs/PointCloud2

/slam/debug_image
  -> sensor_msgs/Image，原图上画当前跟踪 keypoints
```

地图点来自官方：

```cpp
SLAM_->GetTrackedMapPoints()
```

调试特征点来自官方：

```cpp
SLAM_->GetTrackedKeyPointsUn()
```

跟踪状态来自官方：

```cpp
SLAM_->GetTrackingState()
```

### 4.4 launch 文件关系

`launch/orbslam3_only.launch.py`：

```text
只启动 orbslam3_wrapper/mono_node
不会启动视频发布节点
需要另一个终端提供 /camera/image_raw
```

`launch/orbslam3_video.launch.py`：

```text
同时启动 video_publisher_node 和 mono_node
最适合一键运行 ORB-SLAM3 视频 demo
```

### 4.5 ORB-SLAM3 完整数据流

```text
resources/input_video.mp4
  -> video_publisher_node
  -> /camera/image_raw
  -> mono_node::ImageCallback()
  -> cv_bridge
  -> OpenCV grayscale
  -> SLAM_->TrackMonocular()
  -> project code/ORB_SLAM3-master/lib/libORB_SLAM3.so
  -> pose / state / map points / keypoints
  -> /slam/pose / /slam/odom / /slam/map_points / /slam/debug_image
```

### 4.6 和官方代码的边界

`src/orbslam3_wrapper` 负责：

```text
ROS2 节点生命周期
topic 订阅和发布
图像格式转换
动态 mask 应用
调用官方 TrackMonocular
把官方结果转成 ROS 消息
```

官方 ORB-SLAM3 负责：

```text
ORB 特征提取
单目初始化
Tracking
Local Mapping
Loop Closing
Relocalization
Bundle Adjustment
地图点与关键帧维护
Pangolin viewer
```

---

## 5. `src/dpvo_localization`

### 5.1 模块职责

`src/dpvo_localization` 是 ROS2 Python 包，负责封装 DPVO 的视频运行、标定准备、环境检查和增强后处理。

它并不是一个订阅 `/camera/image_raw` 的在线 ROS 节点。当前主入口 `run_dpvo_video` 是读取视频文件或图片目录，调用外部 DPVO 的 `demo.py`。

### 5.2 文件职责

```text
src/dpvo_localization/dpvo_localization/check_dpvo_env.py
  -> 检查 DPVO Python、PyTorch、CUDA 等环境

src/dpvo_localization/dpvo_localization/prepare_dpvo_calib.py
  -> 从工程标定生成/准备 DPVO 需要的 calib txt

src/dpvo_localization/dpvo_localization/run_dpvo_video.py
  -> 主入口，调用 project code/DPVO/demo.py

src/dpvo_localization/dpvo_localization/dpvo_enhancement.py
  -> 读取 DPVO 轨迹和视频，生成关键帧记忆、质量评估、可视化视频等增强结果
```

ROS2 console scripts 定义在：

```text
src/dpvo_localization/setup.py
```

入口：

```text
check_dpvo_env
prepare_dpvo_calib
run_dpvo_video
```

### 5.3 如何调用外部 DPVO

外部 DPVO 路径：

```text
project code/DPVO
```

核心脚本：

```text
project code/DPVO/demo.py
```

权重：

```text
project code/DPVO/dpvo.pth
```

标定：

```text
project code/DPVO/calib/custom_mall.txt
```

`run_dpvo_video.py` 做的事情：

```text
解析参数
  -> 解析 dpvo-root / imagedir / calib / network
  -> 构造 DPVO conda 环境变量
  -> 检查 CUDA 可用
  -> subprocess.run([dpvo_python, demo.py, ...])
  -> 可选运行 dpvo_enhancement.py
```

核心命令等价于：

```bash
/home/ros/miniconda3/envs/dpvo/bin/python \
  "project code/DPVO/demo.py" \
  --imagedir resources/input_video.mp4 \
  --calib "project code/DPVO/calib/custom_mall.txt" \
  --name input_video_clean \
  --stride 2 \
  --network "project code/DPVO/dpvo.pth" \
  --save_trajectory
```

### 5.4 DPVO 输出

外部 DPVO 原始输出：

```text
project code/DPVO/saved_trajectories/<name>.txt
```

增强模块输出：

```text
output/dpvo_enhanced/<name>/
```

people BEV 默认使用：

```text
output/dpvo/trajectory_tum.txt
```

### 5.5 数据流

```text
resources/input_video.mp4
  -> src/dpvo_localization/run_dpvo_video.py
  -> subprocess 调用 project code/DPVO/demo.py
  -> project code/DPVO/saved_trajectories/input_video_clean.txt
  -> output/dpvo/trajectory_tum.txt
  -> src/people_bev_tracker/scripts/offline_pipeline.py
```

### 5.6 和 ROS2 的关系

`dpvo_localization` 是 ROS2 Python package，方便用：

```bash
ros2 run dpvo_localization run_dpvo_video ...
```

但它当前不发布 ROS topic，也不订阅 ROS Image。它主要是“ROS2 包形式的命令行封装”。

---

## 6. `src/people_bev_tracker`

### 6.1 模块职责

`people_bev_tracker` 是离线 Python 流水线，用来把相机轨迹和行人动态位置投影到二维 BEV 平面。

它的核心目标：

```text
原始视频
  -> 行人检测/分割/跟踪
  -> 行人脚底点
  -> DPVO 相机位姿 + 相机内参 + 地面平面
  -> 行人世界坐标
  -> 二维 BEV 视频和 JSON
```

### 6.2 外部开源调用

主要调用：

```text
Ultralytics YOLO
  -> YOLO-seg 检测行人 mask/bbox
  -> model.track(..., tracker="botsort.yaml" 或 "bytetrack.yaml")
  -> 生成 track_id
```

轨迹来自：

```text
output/dpvo/trajectory_tum.txt
```

也就是 DPVO 输出，而不是实时 ROS topic。

### 6.3 文件职责

```text
src/people_bev_tracker/scripts/offline_pipeline.py
  -> 主入口，串起视频、pose、YOLO tracker、地面投影、BEV 绘制

src/people_bev_tracker/scripts/inspect_video.py
  -> 查看视频宽高、fps、帧数

src/people_bev_tracker/scripts/render_bev_from_json.py
  -> 不重跑 YOLO，只用已有 JSON 重新渲染 BEV 视频

src/people_bev_tracker/people_bev_tracker/camera_model.py
  -> 读取标定 YAML，像素反投影为相机射线

src/people_bev_tracker/people_bev_tracker/pose_io.py
  -> 读取 TUM 轨迹，按时间戳匹配最近位姿

src/people_bev_tracker/people_bev_tracker/person_yolo_tracker.py
  -> 包装 Ultralytics YOLO + BoT-SORT/ByteTrack

src/people_bev_tracker/people_bev_tracker/footpoint.py
  -> 从 mask/bbox 计算行人脚底像素点

src/people_bev_tracker/people_bev_tracker/ground_projection.py
  -> 脚底像素射线和地面相交，得到世界坐标

src/people_bev_tracker/people_bev_tracker/state_filter.py
  -> 每个 track_id 的 EMA 平滑和速度门限

src/people_bev_tracker/people_bev_tracker/bev_canvas.py
  -> 绘制二维 BEV 画布

src/people_bev_tracker/people_bev_tracker/io_utils.py
  -> JSON、路径、视频写出等工具

src/people_bev_tracker/people_bev_tracker/types.py
  -> CameraPose、TrackedPerson、PersonWorldState 等数据结构
```

### 6.4 核心几何数据流

```text
YOLO mask/bbox
  -> footpoint.py
  -> foot_uv = [u, v]

foot_uv + K
  -> camera_model.pixel_to_ray()
  -> ray_c

ray_c + T_wc + ground plane
  -> ground_projection
  -> X_w = [x, y, z]

X_w
  -> select BEV axes, usually [x, z]
  -> bev_xy

bev_xy + track_id
  -> state_filter EMA
  -> bev_canvas.draw()
```

### 6.5 输入输出

输入：

```text
resources/input_video.mp4
config/KannalaBrandt8_1280x720.yaml
output/dpvo/trajectory_tum.txt
```

输出：

```text
output/people_bev/bev_tracking.mp4
output/people_bev/debug_overlay.mp4
output/people_bev/people_tracks.json
output/people_bev/camera_trajectory.json
```

### 6.6 和其他模块的关系

```text
依赖 DPVO:
  读取 output/dpvo/trajectory_tum.txt

依赖外部 YOLO:
  Ultralytics 检测/跟踪行人

不依赖 ORB-SLAM3:
  目前没有读取 /slam/pose

不依赖 KV-Tracker:
  目前没有读取 output/kv_track3r_repro/trajectory_tum.txt，但后续可替换 pose source
```

---

## 7. `src/KV-tracker`

### 7.1 模块职责

`src/KV-tracker` 是官方 KV-Tracker 的外部封装。设计原则：

```text
官方代码只读，不修改 project code/KV-tracker/kv_tracker-main
所有新增代码放在 src/KV-tracker
通过 subprocess/sys.path 调用官方 main.py
把官方输出转换成工程可用的 TUM/JSON/PLY/CSV
```

官方代码：

```text
project code/KV-tracker/kv_tracker-main
```

本工程封装：

```text
src/KV-tracker
```

### 7.2 外部开源调用

`src/KV-tracker` 通过 wrapper 调用：

```text
project code/KV-tracker/kv_tracker-main/main.py
```

官方 KV-Tracker 内部还依赖：

```text
Pi3 / π³ 多视图几何网络
SAM2 实时版
Rerun
PyTorch
```

本工程中这些依赖优先放在：

```text
src/KV-tracker/thirdparty/
```

而不是写入官方目录。

### 7.3 文件职责

```text
src/KV-tracker/scripts/run_official_kv_tracker.py
  -> 主入口：生成/读取配置，调用官方 main.py，结束后导出结果

src/KV-tracker/scripts/export_repro_outputs.py
  -> 只做离线转换，不重新运行官方 KV-Tracker

src/KV-tracker/scripts/run_kv_tracker_rerun.py
  -> Rerun 记录版本

src/KV-tracker/scripts/run_kv_tracker_rerun_slim.py
  -> Rerun slim 版本，过滤大图像，减小 .rrd 体积

src/KV-tracker/scripts/run_kv_tracker_rerun_live.py
  -> 实时推送给 Rerun viewer

src/KV-tracker/kv_track3r_app/official_bridge.py
  -> 处理官方路径、sys.path、subprocess、必要资源可见性

src/KV-tracker/kv_track3r_app/export_tools.py
  -> 保存 TUM、JSON、PLY、CSV 的工具函数

src/KV-tracker/kv_track3r_app/output_converter.py
  -> 把官方 traj.npy / pcd.npy / kf_poses.npy 转成工程格式

src/KV-tracker/config/mall_video.yaml
  -> 本仓库视频输入配置

src/KV-tracker/docs/
  -> 论文解析和商场导航应用方案
```

### 7.4 调用官方代码的数据流

```text
src/KV-tracker/scripts/run_official_kv_tracker.py
  -> official_bridge.run_official_subprocess()
  -> subprocess.run([
       python,
       project code/KV-tracker/kv_tracker-main/main.py,
       src/KV-tracker/config/mall_video.yaml,
       --cam_only,
       --resize_dim,
       ...
     ], cwd=official_root)
  -> 官方 main.py 运行 KV-Tracker
  -> output/kv_track3r_repro/traj.npy 等官方结果
  -> output_converter.convert_official_outputs()
  -> trajectory_tum.txt / trajectory.json / keyframes.json / confidence.json
```

### 7.5 输出

```text
output/kv_track3r_repro/traj.npy
output/kv_track3r_repro/trajectory.npy
output/kv_track3r_repro/trajectory_tum.txt
output/kv_track3r_repro/trajectory.json
output/kv_track3r_repro/keyframe_poses.npy
output/kv_track3r_repro/keyframes.json
output/kv_track3r_repro/confidence.json
output/kv_track3r_repro/runtime.csv
output/kv_track3r_repro/summary.md
```

### 7.6 和其他模块的关系

```text
可替代/对比 DPVO:
  output/kv_track3r_repro/trajectory_tum.txt 可作为另一条相机轨迹来源

可服务 people BEV:
  后续可让 people_bev_tracker 读取 KV-Tracker 的 trajectory_tum.txt

可服务商场导航:
  输出相机轨迹、关键帧、局部结构、confidence
```

---

## 8. VGGT 在本工程中的位置

VGGT 不在 `src/` 下作为 ROS2 package 存在。它主要由根目录 `scripts/` 封装：

```text
scripts/run_vggt_video.py
scripts/run_vggt_scene.py
scripts/open_vggt_viser.py
scripts/run_vggt_official_colmap.py
```

官方代码：

```text
project code/VGGT/vggt-main
```

调用方式类似：

```text
scripts/run_vggt_video.py
  -> sys.path 加入 project code/VGGT/vggt-main
  -> import vggt.models.vggt.VGGT
  -> 加载 VGGT-1B 权重
  -> 对视频抽帧/窗口推理
  -> 输出点云、深度、相机中心、preview
```

所以 VGGT 和 `src/` 的关系是：

```text
不是 src 下模块
但和 src/KV-tracker、src/people_bev_tracker 一样属于离线几何/建图工具链
```

---

## 9. launch 文件和 src 的对应关系

### 9.1 `launch/video_only.launch.py`

启动：

```text
src/video_publisher
```

具体节点：

```text
package='video_publisher'
executable='video_publisher_node'
```

发布：

```text
/camera/image_raw
/camera/image_raw_full
/yolo/dynamic_mask
```

### 9.2 `launch/orbslam3_only.launch.py`

启动：

```text
src/orbslam3_wrapper
```

具体节点：

```text
package='orbslam3_wrapper'
executable='mono_node'
```

要求外部已有：

```text
/camera/image_raw
/yolo/dynamic_mask
```

### 9.3 `launch/orbslam3_video.launch.py`

同时启动：

```text
src/video_publisher
src/orbslam3_wrapper
```

这是 ORB-SLAM3 在线演示最推荐的 launch。

### 9.4 `launch/dpvo_offline.launch.py`

启动：

```text
src/dpvo_localization
```

具体入口：

```text
package='dpvo_localization'
executable='run_dpvo_video'
```

注意：该 launch 里可能仍写着旧路径 `Opensource code/DPVO-main`。当前更推荐直接用 `ros2 run dpvo_localization run_dpvo_video ...` 显式传 `project code/DPVO`。

---

## 10. 编译和运行关系

### 10.1 ROS2 包

以下是 ROS2 包，需要 `colcon build`：

```text
src/video_publisher      # ament_python
src/dpvo_localization    # ament_python
src/orbslam3_wrapper     # ament_cmake
```

构建：

```bash
cd /home/ros/ros2_orbslam3
source /opt/ros/humble/setup.bash

export ORB_SLAM3_DIR="/home/ros/ros2_orbslam3/project code/ORB_SLAM3-master"
export ORB_SLAM3_LIBRARY="$ORB_SLAM3_DIR/lib/libORB_SLAM3.so"

colcon build \
  --packages-select video_publisher dpvo_localization orbslam3_wrapper \
  --symlink-install \
  --cmake-args \
    -DORB_SLAM3_DIR="$ORB_SLAM3_DIR" \
    -DORB_SLAM3_LIBRARY="$ORB_SLAM3_LIBRARY"
```

运行前：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 10.2 非 ROS2 离线 Python 工程

以下不需要 `colcon build`：

```text
src/people_bev_tracker
src/KV-tracker
```

它们直接用 Python 脚本运行：

```bash
conda activate dpvo
python src/people_bev_tracker/scripts/offline_pipeline.py ...
python src/KV-tracker/scripts/run_official_kv_tracker.py ...
```

---

## 11. 环境关系

### 11.1 ROS2 / ORB-SLAM3 / video_publisher

使用干净系统 ROS 环境：

```bash
unset PYTHONPATH
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash
source install/setup.bash
```

原因：`cv_bridge` 不能加载 conda `dpvo` 里的 NumPy 2.x。

### 11.2 DPVO / people BEV / KV-Tracker / VGGT

使用 conda `dpvo`：

```bash
conda activate dpvo
```

原因：这些模块依赖 PyTorch/CUDA/深度学习库。

### 11.3 同一机器上混合运行时的原则

```text
ROS2 图像/SLAM 在线节点:
  干净系统 ROS 终端

深度学习离线脚本:
  conda dpvo 终端
```

不要在带有：

```text
PYTHONPATH=/home/ros/miniconda3/envs/dpvo/lib/python3.10/site-packages
```

的终端里跑 `video_publisher_node`。

---

## 12. 从输入到最终 BEV 的完整工程链路

### 12.1 在线 SLAM 查看链路

```text
resources/input_video.mp4
  -> src/video_publisher
  -> /camera/image_raw
  -> src/orbslam3_wrapper
  -> 官方 ORB-SLAM3
  -> /slam/pose / /slam/map_points / /slam/debug_image
  -> RViz2
```

### 12.2 离线 DPVO + 行人 BEV 链路

```text
resources/input_video.mp4
  -> src/dpvo_localization/run_dpvo_video.py
  -> project code/DPVO/demo.py
  -> output/dpvo/trajectory_tum.txt
  -> src/people_bev_tracker/scripts/offline_pipeline.py
  -> Ultralytics YOLO + BoT-SORT
  -> footpoint + ground projection
  -> output/people_bev/bev_tracking.mp4
```

### 12.3 KV-Tracker 复现链路

```text
resources/input_video.mp4
  -> src/KV-tracker/scripts/run_official_kv_tracker.py
  -> project code/KV-tracker/kv_tracker-main/main.py
  -> Pi3/KV-cache tracking
  -> output/kv_track3r_repro/trajectory_tum.txt
  -> 可用于后续 BEV / 导航 / 对比 DPVO
```

### 12.4 VGGT 局部结构链路

```text
resources/input_video.mp4
  -> scripts/run_vggt_video.py
  -> project code/VGGT/vggt-main
  -> depth / confidence / point cloud / camera centers
  -> scripts/open_vggt_viser.py
  -> 浏览器 3D 查看
```

---

## 13. 文件关系速查

### 13.1 视频与 mask

```text
resources/input_video.mp4
  -> video_publisher
  -> DPVO
  -> VGGT
  -> KV-Tracker
  -> people_bev_tracker

resources/input_video.mp4_bev.mp4
  -> video_publisher 抠 mask
  -> 主要作为参考可视化，不建议作为主算法输入
```

### 13.2 标定

```text
config/KannalaBrandt8_960x540.yaml
  -> ORB-SLAM3 在线节点，匹配 video_publisher 输出 960x540

config/KannalaBrandt8_1280x720.yaml
  -> people_bev_tracker 默认相机内参来源，会按实际视频尺寸缩放

config/KannalaBrandt8.yaml
  -> 原始 1920x1080 标定

project code/DPVO/calib/custom_mall.txt
  -> DPVO 输入标定
```

### 13.3 轨迹

```text
project code/DPVO/saved_trajectories/input_video_clean.txt
  -> DPVO 原始输出

output/dpvo/trajectory_tum.txt
  -> people_bev_tracker 默认读取

output/kv_track3r_repro/trajectory_tum.txt
  -> KV-Tracker 输出，可作为另一条相机轨迹来源

/slam/pose
  -> ORB-SLAM3 在线 ROS 位姿 topic
```

### 13.4 BEV 输出

```text
output/people_bev/bev_tracking.mp4
output/people_bev/debug_overlay.mp4
output/people_bev/people_tracks.json
output/people_bev/camera_trajectory.json
```

---

## 14. 当前模块边界和后续扩展点

### 14.1 当前已经打通的边界

```text
video_publisher -> orbslam3_wrapper
DPVO trajectory -> people_bev_tracker
KV-Tracker official output -> engineering JSON/TUM
VGGT video frames -> point cloud / camera centers
```

### 14.2 当前还没有打通的边界

```text
ORB-SLAM3 /slam/pose -> people_bev_tracker
KV-Tracker trajectory_tum.txt -> people_bev_tracker 自动切换
VGGT depth/point cloud -> BEV 静态结构地图
people_bev_tracker -> ROS2 实时节点
DPVO -> 真正订阅 /camera/image_raw 的在线 ROS2 节点
```

### 14.3 推荐后续扩展

1. 给 `people_bev_tracker` 增加 pose source 选项：

```text
--pose-source dpvo_tum
--pose-source kv_tum
--pose-source orbslam_rosbag
```

2. 把 `/slam/pose` 录成 TUM：

```text
/slam/pose -> output/orbslam3/trajectory_tum.txt
```

3. 把 people BEV 做成 ROS2 节点：

```text
/camera/image_raw
/slam/pose 或 /tf
YOLO tracker
-> /people_bev/map_image
-> /people_bev/markers
```

4. 用 VGGT/KV-Tracker 的局部结构点云辅助估计地面平面和静态障碍物。

5. 做真实商场平面图配准，把 BEV 轨迹从算法坐标系映射到真实地图坐标系。

---

## 15. 最重要的理解

这套工程不是一个单体程序，而是多个开源算法和本地 wrapper 的组合：

```text
ORB-SLAM3:
  官方 C++ SLAM 核心 + src/orbslam3_wrapper 的 ROS2 适配

DPVO:
  官方 Python/CUDA VO 核心 + src/dpvo_localization 的命令行封装

KV-Tracker:
  官方论文代码 + src/KV-tracker 的只读外部调用和结果导出

VGGT:
  官方多视图几何模型 + scripts/ 下的视频/场景封装

people BEV:
  本工程离线应用层，融合视频、YOLO、相机轨迹、地面投影，输出二维地图
```

其中 `src/` 的核心价值是：

```text
把外部开源算法变成当前商场视频工程可用的数据流。
```

也就是把“算法 demo”接成“可运行、可观察、可导出、可继续接二维地图和行人定位”的工程系统。
