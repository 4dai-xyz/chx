# 室内视觉导航与 Go2 机器人控制仿真工具集

本仓库用于整理和沉淀以下工程模块：

```text
1. 面向商场等室内场景的纯视觉 SLAM / 视觉里程计 / BEV 地图实验。
2. 相机轨迹、静态地图、动态行人的二维俯视图可视化。
3. Unitree Go2 的底层运动控制、ROS2 导航实验、MuJoCo 仿真和 Isaac Lab 仿真流程。
```

本项目是一个工程集成仓库，不直接内置 DPVO、ORB-SLAM3、KV-Track3r、VGGT、ScaRF-SLAM、Unitree SDK、Isaac Lab 等第三方官方源码和模型权重。相关第三方项目应通过官方仓库、公司镜像、Git submodule 或单独安装脚本获取。


## 第三方源码与上游项目地址

本仓库只保留自研 wrapper、配置、脚本、接口和文档，不直接上传第三方官方源码大包。需要复现实验或部署环境时，请从以下上游地址、公司内部 fork 或公司镜像获取源码。若公司内部已经建立固定 fork，以公司内部 fork 和指定 commit hash 为准。

### 视觉定位、三维重建与地图相关

| 模块 | 用途 | 上游地址 |
|---|---|---|
| DPVO | 单目视觉里程计主定位前端 | https://github.com/princeton-vl/DPVO |
| ORB-SLAM3 | 传统特征法 SLAM 对照与 ROS2 wrapper 调用对象 | https://github.com/UZ-SLAMLab/ORB_SLAM3 |
| Pangolin | ORB-SLAM3 / DPVO viewer 相关依赖 | https://github.com/stevenlovegrove/Pangolin |
| VGGT | feed-forward 视觉几何、点云/深度/相机估计实验 | https://github.com/facebookresearch/vggt |
| Depth Anything V2 | 单目深度估计，Route A 稠密点云/栅格地图实验 | https://github.com/DepthAnything/Depth-Anything-V2 |
| ZoeDepth | 单目伪公制深度备用路线 | https://github.com/isl-org/ZoeDepth |
| ScaRF-SLAM | 稠密/半稠密重建与全局结构对照路线 | https://github.com/ori-drs/ScaRF-SLAM |
| KV-Tracker / KV-Track3r | Transformer pose tracking 对照路线 | https://github.com/Marwan99/kv_tracker |
| SAM 2 | 语义/实例分割与动态区域 mask 参考 | https://github.com/facebookresearch/sam2 |
| Ultralytics YOLO | 行人检测与跟踪基础接口 | https://github.com/ultralytics/ultralytics |
| VINS-Fusion | VIO / 多传感器融合方法参考 | https://github.com/HKUST-Aerial-Robotics/VINS-Fusion |

### Go2 控制、仿真与强化学习相关

| 模块 | 用途 | 上游地址 |
|---|---|---|
| Unitree SDK2 | Go2 官方底层通信 SDK | https://github.com/unitreerobotics/unitree_sdk2 |
| Unitree SDK2 Python | Go2 Python 控制接口 | https://github.com/unitreerobotics/unitree_sdk2_python |
| Unitree ROS2 | Go2 ROS2 通信与消息接口 | https://github.com/unitreerobotics/unitree_ros2 |
| Unitree MuJoCo | Go2 MuJoCo 仿真参考 | https://github.com/unitreerobotics/unitree_mujoco |
| MuJoCo | 物理仿真引擎 | https://github.com/google-deepmind/mujoco |
| Isaac Lab | Isaac Sim / Isaac Lab 强化学习与仿真框架 | https://github.com/isaac-sim/IsaacLab |
---

## 1. 仓库定位

本仓库主要保存：

```text
源码接口
ROS2 wrapper
配置文件
启动脚本
实验流程文档
后续研发计划
```

本仓库不保存：

```text
大模型权重
原始视频数据
运行结果
点云结果
Route A 历史输出
第三方官方源码大包
conda / venv 环境
build / install / log 目录
```

这样做的目的是保持公司公共 GitHub 仓库轻量、清晰、可维护，同时避免把隐私视频、模型权重、第三方许可证不明确的源码或大量实验产物上传到公共仓库。

---

## 2. 推荐目录结构

建议把当前 WSL 中的多个实验目录整理成如下结构：

```text
indoor-navigation-go2/
├── README.md
├── .gitignore
├── docs/
├── mall_visual_slam/
│   ├── config/
│   ├── launch/
│   ├── scripts/
│   └── src/
│       ├── dpvo_localization/
│       ├── orbslam3_wrapper/
│       ├── video_publisher/
│       ├── people_bev_tracker/
│       └── KV-tracker/
├── go2_control/
│   ├── docs/
│   ├── notes/
│   ├── scripts/
│   └── projects/
│       ├── base/
│       ├── go2_control_demos/
│       └── go2_nav_sim/
└── isaac_go2_sim/
    ├── docs/
    ├── scripts/
    └── source_extensions/
```

如果保持原始 ROS2 workspace 结构，则主要源码包位于：

```text
src/dpvo_localization
src/orbslam3_wrapper
src/video_publisher
src/people_bev_tracker
src/KV-tracker
```

---

## 3. 商场室内视觉导航模块

### 3.1 DPVO 定位接口

路径：

```text
src/dpvo_localization/
```

作用：

```text
1. 调用 DPVO 作为主要单目视觉里程计前端。
2. 从视频帧中估计相机 3D 位姿轨迹。
3. 准备相机标定文件。
4. 检查 DPVO 运行环境。
5. 为后续 BEV 地图和动态行人投影提供主轨迹。
```

重要文件：

```text
src/dpvo_localization/dpvo_localization/run_dpvo_video.py
src/dpvo_localization/dpvo_localization/prepare_dpvo_calib.py
src/dpvo_localization/dpvo_localization/check_dpvo_env.py
src/dpvo_localization/dpvo_localization/dpvo_enhancement.py
```

说明：

```text
DPVO 官方源码和 dpvo.pth 权重不应提交到本仓库。
请在本地单独安装 DPVO。
```

---

### 3.2 ORB-SLAM3 ROS2 Wrapper

路径：

```text
src/orbslam3_wrapper/
```

作用：

```text
1. 将官方 ORB-SLAM3 封装成 ROS2 C++ 节点。
2. 订阅 ROS2 图像 topic。
3. 调用 ORB-SLAM3 monocular tracking。
4. 发布相机位姿、轨迹和可视化信息。
5. 可与 video_publisher、RViz2 一起运行。
```

重要文件：

```text
src/orbslam3_wrapper/src/mono_node.cpp
config/KannalaBrandt8.yaml
config/KannalaBrandt8_960x540.yaml
launch/orbslam3_only.launch.py
```

说明：

```text
运行时通过 ORB_SLAM3_DIR、ORB_SLAM3_LIBRARY、LD_LIBRARY_PATH 指向本地安装位置。
```

---

### 3.3 视频发布节点

路径：

```text
src/video_publisher/
```

作用：

```text
1. 将本地视频文件发布为 ROS2 image topic。
2. 支持重采样和分辨率缩放。
3. 可选使用 mask 视频剔除动态区域。
4. 为 ORB-SLAM3、RViz2 和其他 ROS2 节点提供统一图像输入。
```

重要文件：

```text
src/video_publisher/video_publisher/video_publisher_node.py
launch/video_only.launch.py
```

---

### 3.4 People BEV Tracker

路径：

```text
src/people_bev_tracker/
```

作用：

```text
1. 使用 YOLO / BoT-SORT 等方法检测和跟踪行人。
2. 将相机轨迹从 3D 位姿压平到 BEV 平面。
3. 将行人脚点投影到 BEV 坐标系。
4. 构建二维静态地图和导航栅格接口。
5. 将动态行人作为 overlay 显示，不写入静态地图。
6. 为 Route A 的稠密重建、ScaRF-SLAM、Figure 1 风格俯视展示保留接口。
```

重要文件：

```text
src/people_bev_tracker/people_bev_tracker/person_yolo_tracker.py
src/people_bev_tracker/people_bev_tracker/footpoint.py
src/people_bev_tracker/people_bev_tracker/trajectory_flatten.py
src/people_bev_tracker/people_bev_tracker/static_map.py
src/people_bev_tracker/people_bev_tracker/map_quality.py
src/people_bev_tracker/people_bev_tracker/person_map_filter.py
src/people_bev_tracker/scripts/offline_pipeline_A.py
src/people_bev_tracker/scripts/run_route_A_v3_pipeline.py
src/people_bev_tracker/scripts/filter_people_tracks_on_map.py
```

长期目标：

```text
DPVO 相机轨迹
  + Depth Anything / ScaRF-SLAM 稠密重建
  + 二维 occupancy grid
  + 动态行人物理过滤
  + 类似论文 Figure 1 的俯视路线图展示
```

---

### 3.5 KV-Track3r 调用接口

路径：

```text
src/KV-tracker/
```

作用：

```text
1. 提供官方 KV-Track3r 的调用桥接。
2. 不修改官方开源代码，只通过 wrapper 调用。
3. 将官方输出转换为本项目可读取的相机位姿、局部结构、全局结构等格式。
4. 保留论文理解、商场导航应用和 BEV 地图后续接口。
```

重要文件：

```text
src/KV-tracker/kv_track3r_app/official_bridge.py
src/KV-tracker/kv_track3r_app/output_converter.py
src/KV-tracker/kv_track3r_app/export_tools.py
src/KV-tracker/scripts/run_official_kv_tracker.py
```

---

## 4. Route A 后续计划

Route A 是当前商场室内导航地图重建的主要研发路线。

目标流程：

```text
单目视频
  -> DPVO 输出相机轨迹
  -> 行人检测和多目标跟踪
  -> 稠密深度 / 稠密重建后端
  -> 二维 occupancy grid
  -> 行人位置物理过滤
  -> BEV 视频和俯视路线图
```

最新计划文档：

```text
src/people_bev_tracker/docs/06_RouteA_V3_ScaRF_SLAM稠密重建与导航栅格执行方案_最新版.md
src/people_bev_tracker/docs/07_RouteA_V3_2官方ScaRF与地图诊断执行方案_最新版.md
```

Route A 的运行结果不提交到 Git：

```text
output/route_A/
output/route_A_v2/
output/route_A_v3_scarf/
output/route_A_v3_2_scarf_official/
```

后续工作：

```text
1. 复现官方 ScaRF-SLAM，作为稠密重建基线。
2. 判断地图失败来自深度、尺度、融合、occupancy 投影还是渲染。
3. 生成更清晰的黑白 / 三值二维导航栅格。
4. 对动态行人做物理约束过滤：
   - 只显示相机附近行人；
   - 只显示视野内行人；
   - 只显示 free space 内行人；
   - 禁止将行人画到墙体、障碍物或 unknown 深处。
5. 渲染类似论文 Reinforced Cross-Modal Matching and Self-Supervised Imitation Learning for Vision-Language Navigation Figure 1 的俯视展示：
   - 全局场景 / 地图；
   - 起点和终点；
   - 相机轨迹；
   - 动态行人 overlay；
   - 可选局部图像缩略图。
```

---

## 5. Go2 底层控制与仿真

Go2 部分建议与商场视觉 SLAM 模块保持相对独立。

推荐保留路径：

```text
go2_control/docs/
go2_control/notes/
go2_control/scripts/
go2_control/projects/base/
go2_control/projects/go2_control_demos/
go2_control/projects/go2_nav_sim/
```

主要能力：

```text
1. Unitree Go2 ROS2 环境准备。
2. MuJoCo 中的底层运动控制实验。
3. cmd_vel bridge 和基础导航接口。
4. Nav2 / SLAM Toolbox 冒烟测试。
5. RL policy runner 接口。
```

不要提交：

```text
.venv-unitree/
build/
install/
log/
logs/
downloads/
MUJOCO_LOG.TXT
projects/go2_control_demos/policies/*.pt
```

真机安全提示：

```text
所有真机控制脚本在上机前必须人工 review。
优先在仿真环境中测试。
限制速度和力矩。
准备急停。
不要直接运行未经验证的策略模型。
```

---

## 6. Isaac Lab / Isaac Sim

Isaac Lab 作为外部依赖处理，不建议将官方完整源码放进本仓库。

推荐策略：

```text
1. README 或 docs 中记录 Isaac Lab 版本和安装方法。
2. 保留自定义 Go2 task / env / wrapper / extension。
3. 保留 Isaac 相关启动脚本。
4. 不提交 IsaacLab 官方源码、asset cache 和日志。
```

推荐路径：

```text
isaac_go2_sim/docs/
isaac_go2_sim/scripts/
isaac_go2_sim/source_extensions/
```

---

## 7. 外部依赖

本仓库依赖以下外部项目或运行环境：

```text
ROS2 Humble
OpenCV
cv_bridge
ORB-SLAM3
DPVO
YOLO / Ultralytics
Depth Anything V2 / V3
ScaRF-SLAM
KV-Track3r
Unitree SDK2 / unitree_ros2
MuJoCo
Isaac Lab
```

依赖管理建议：

```text
1. 官方开源项目放在仓库外部，或使用 Git submodule / 公司 fork。
2. 模型权重放到公司模型仓库、对象存储或 GitHub Release，不直接进 Git。
3. 数据集、视频、运行结果放到外部数据目录。
4. README 中写清楚第三方项目的安装路径和环境变量。
```

---

## 8. 编译与运行

### 8.1 ROS2 workspace 编译

```bash
cd /path/to/workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 8.2 启动视频发布节点

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ./launch/video_only.launch.py
```

### 8.3 启动 ORB-SLAM3 wrapper

示例环境变量：

```bash
export ORB_SLAM3_DIR=/path/to/ORB_SLAM3
export ORB_SLAM3_LIBRARY=$ORB_SLAM3_DIR/lib/libORB_SLAM3.so
export LD_LIBRARY_PATH=$ORB_SLAM3_DIR/lib:$ORB_SLAM3_DIR/Thirdparty/DBoW2/lib:$ORB_SLAM3_DIR/Thirdparty/g2o/lib:$LD_LIBRARY_PATH
```

启动：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ./launch/orbslam3_only.launch.py
```

如需查看图像、轨迹或 topic，可单独打开：

```bash
rviz2
```

### 8.4 运行 DPVO 视频轨迹接口

DPVO 建议使用独立 Python / conda 环境。

示例：

```bash
conda activate dpvo
python -m dpvo_localization.run_dpvo_video --help
```

实际命令需要根据本地 DPVO 安装路径、模型权重和相机标定文件调整。

---

## 9. 不纳入 Git 的内容

以下内容应由 `.gitignore` 排除：

```text
build/
install/
log/
output/
resources/*.mp4
*.pt
*.pth
*.onnx
*.ply
*.pcd
*.npy
*.npz
*.pdf
*.zip
*.tar.gz
project code/
thirdparty/
src/KV-tracker/thirdparty/
assets_cache/
downloads/
.cache/
.local/
.runtime/
.venv*/
```

原因：

```text
仓库应保存源码、配置、wrapper、接口和文档。
大数据、运行结果、模型权重、第三方源码和本地环境应通过安装脚本或外部存储重新获取。
```

---

## 10. 当前状态

已完成：

```text
1. ROS2 视频发布节点。
2. ORB-SLAM3 monocular ROS2 wrapper。
3. DPVO 视频轨迹接口。
4. 行人检测、跟踪和 BEV 投影工具。
5. Route A 文档和后续执行接口。
6. KV-Track3r 官方代码调用桥接。
7. Go2 控制和仿真辅助脚本。
```

进行中 / 计划中：

```text
1. 官方 ScaRF-SLAM 稠密重建基线。
2. 更可靠的二维 occupancy grid。
3. 基于地图物理约束的动态行人过滤。
4. 类似论文 Figure 1 的俯视路线图展示。
5. 将视觉导航结果进一步接入 Go2 导航和运动控制。
```

---

## 11. 许可证与第三方声明

本仓库包含自研集成代码和工程文档，同时引用多个第三方开源项目。每个第三方项目均保留其原始许可证、引用方式和使用限制。

公开发布前请确认：

```text
1. 公司是否允许公开发布该项目。
2. 仓库中是否包含第三方源码。
3. 仓库中是否包含模型权重或数据集。
4. 演示视频是否包含商场隐私画面或可识别人员。
5. 所有第三方许可证是否满足公司合规要求。
```
