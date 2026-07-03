# 公司公共 GitHub 上传整理方案

本文档用于把当前 WSL 中的实验目录整理成适合公司公共 GitHub 的工程仓库。当前涉及三个主要目录：

```text
/home/ros/ros2_orbslam3     商场室内纯视觉 SLAM / BEV 导航主项目
/home/ros/unitree_dev       Unitree Go2 底层控制、ROS2、MuJoCo、Nav2 相关项目
/home/ros/isaac_go2         Isaac Lab / Isaac Sim 中的 Go2 仿真项目
```

建议不要把当前实验目录原样上传。当前目录包含大量构建产物、模型权重、视频、论文 PDF、第三方官方源码、缓存、Route A 运行结果和历史调参输出。更适合的做法是新建一个干净发布目录，只复制真正要维护的源码、配置、启动脚本、接口和文档。

---

## 1. 推荐仓库组织方式

### 方案 A：一个公司总仓库

适合公司内部希望统一展示“视觉导航 + 机器人控制 + 仿真”的情况：

```text
indoor-navigation-go2/
├── README.md
├── .gitignore
├── docs/
├── mall_visual_slam/
│   ├── config/
│   ├── launch/
│   ├── scripts/
│   ├── src/
│   └── docs/
├── go2_control/
│   ├── docs/
│   ├── scripts/
│   └── projects/
└── isaac_go2_sim/
    ├── docs/
    ├── scripts/
    └── source_extensions/
```

优点：

```text
1. 对外展示完整系统闭环。
2. README 可以讲清楚从视觉定位到 Go2 执行的整体路线。
3. 适合公司公共 GitHub 做项目合集。
```

缺点：

```text
1. 三个方向依赖差异很大。
2. 后续维护时 CI / 环境会比较复杂。
```

### 方案 B：拆成两个仓库

推荐实际落地使用：

```text
mall-visual-slam-navigation/
go2-control-and-simulation/
```

其中：

```text
mall-visual-slam-navigation
  = ros2_orbslam3 中的商场视觉 SLAM、DPVO、ORB-SLAM3 wrapper、people BEV tracker、KV/ScaRF 接口。

go2-control-and-simulation
  = unitree_dev + isaac_go2 中的 Go2 控制、MuJoCo、Nav2、Isaac Lab 仿真脚本。
```

优点：

```text
1. 项目边界清楚。
2. 商场导航算法和 Go2 控制仿真不会互相污染环境。
3. 更适合后续单独迭代。
```

建议：

```text
如果公司只要求一个公共 GitHub 项目，用方案 A。
如果你能决定仓库结构，用方案 B。
```

---

## 2. 商场室内导航项目上传清单

源目录：

```text
/home/ros/ros2_orbslam3
```

### 2.1 必须上传

这些是你自己写的 ROS2 wrapper、定位、BEV、行人检测和地图接口代码：

```text
src/dpvo_localization/
src/orbslam3_wrapper/
src/video_publisher/
src/people_bev_tracker/
src/KV-tracker/
config/
launch/
scripts/
```

其中：

```text
src/dpvo_localization
  DPVO 视频运行、标定准备、环境检查、轨迹增强接口。

src/orbslam3_wrapper
  ROS2 C++ wrapper，调用 ORB-SLAM3 官方库，订阅图像并发布位姿/轨迹。

src/video_publisher
  视频转 ROS2 image topic，可叠加动态 mask。

src/people_bev_tracker
  行人检测跟踪、相机轨迹投影、BEV 地图、Route A 接口、Depth/ScaRF 后续接口。

src/KV-tracker
  KV-Track3r 官方库调用桥接、输出转换、论文理解和商场导航应用接口。
```

建议保留的顶层文档：

```text
系统运行总说明.md
src代码结构与数据流说明.md
纯视觉SLAM商场导航项目完整知识库.md
输出结果保留清单.md
```

建议保留的 Route A 文档：

```text
src/people_bev_tracker/docs/00_文档索引_阅读顺序.md
src/people_bev_tracker/docs/06_RouteA_V3_ScaRF_SLAM稠密重建与导航栅格执行方案_最新版.md
src/people_bev_tracker/docs/07_RouteA_V3_2官方ScaRF与地图诊断执行方案_最新版.md
```

说明：

```text
Route A 的运行结果不要上传，但保留 06/07 两份执行方案。
这样公司仓库里既不会塞 4.7G 输出，又能保留后续继续做 ScaRF-SLAM、二维栅格地图、Figure 1 风格俯视展示、行人物理过滤的接口。
```

### 2.2 建议选择性上传

```text
src/people_bev_tracker/docs/01_*.md 到 05_*.md
```

建议：

```text
如果公司公共 GitHub 希望简洁，只保留 00、06、07。
如果需要展示完整研究迭代过程，可以保留 01 到 07，但 README 中要说明 01-05 是历史方案。
```

```text
Claude_Code_*.md
代码逐句说明.md
代码逐文件逐块说明.md
```

建议：

```text
这些更像个人开发辅助文档，不建议放公共仓库根目录。
如果确实要保留，可以移动到 docs/archive/。
```

### 2.3 不要上传

运行输出和调参结果：

```text
output/
output/route_A/
output/route_A_v2/
output/route_A_v3_scarf/
output/route_A_v3_2_scarf_official/
output/vggt_aligned_full_run/
output/people_bev/
output/people_bev_test/
output/kv_track3r_repro/
```

ROS2 构建产物：

```text
build/
install/
log/
```

缓存和本地环境：

```text
.cache/
.local/
.runtime/
.codex/
.agents/
__pycache__/
*.pyc
```

视频、权重、PDF 和大文件：

```text
resources/*.mp4
yolo11n-seg.pt
project code/DPVO/dpvo.pth
*.pdf
*.zip
*.tar.gz
```

个人学习目录：

```text
C++ learn/
Python Learn/
```

第三方下载包和本地依赖：

```text
thirdparty/
thirdparty/python_packages/
thirdparty/gsplat/
thirdparty/cuda-cccl-12-9/
thirdparty/cuda_cccl_debs/
```

特殊提醒：

```text
src/KV-tracker/thirdparty/
```

这个目录当前约 318M，包含 Pi3、SAM2 实时分割、checkpoint、示例视频等第三方内容。公司公共 GitHub 不建议上传。保留 `src/KV-tracker/kv_track3r_app/`、`scripts/`、`config/`、`docs/` 即可。

### 2.4 第三方官方代码处理方式

当前目录里有：

```text
project code/DPVO
project code/ORB_SLAM3-master
project code/VGGT
project code/KV-tracker
project code/Pangolin
```

不建议直接复制到公司公共 GitHub 主仓库。建议在 README 中写成外部依赖：

```text
third-party/DPVO              请按官方仓库安装
third-party/ORB_SLAM3         请按官方仓库编译
third-party/VGGT              可选，用于历史点云实验
third-party/KV-Track3r        可选，用于稠密重建基线
third-party/ScaRF-SLAM        后续计划接口
```

如果公司要求一键复现，可以用以下方式之一：

```text
1. Git submodule 指向官方仓库或公司 fork。
2. scripts/setup_thirdparty.sh 自动 clone。
3. docs/third_party_setup.md 写安装步骤。
```

不要把官方代码、模型权重、视频数据和运行结果混在主仓库里。

---

## 3. Go2 底层控制项目上传清单

源目录：

```text
/home/ros/unitree_dev
```

### 3.1 必须上传

建议保留你自己写的流程文档：

```text
docs/
notes/
README.md
```

建议保留脚本：

```text
scripts/build_all_possible.sh
scripts/build_mujoco_cpp.sh
scripts/build_sdk2.sh
scripts/build_unitree_ros2.sh
scripts/check_go2_ros_graph.sh
scripts/check_network.sh
scripts/go2_rl_policy_check.sh
scripts/go2_ros2_sdk_check.sh
scripts/go2_ros_env.sh
scripts/install_mujoco.sh
scripts/install_system_deps.sh
scripts/setup_python_env.sh
scripts/unitree_env.sh
scripts/run_go2_cmd_vel_smoke_test.sh
scripts/run_go2_nav2.sh
scripts/run_go2_nav2_goal.sh
scripts/run_go2_nav2_slam_map.sh
scripts/run_go2_nav_bridge.sh
scripts/run_go2_nav_bridge_headless.sh
scripts/run_go2_nav_rviz.sh
scripts/run_go2_slam_mapping_drive.sh
scripts/run_go2_slam_toolbox.sh
scripts/save_go2_slam_map.sh
```

建议保留项目代码：

```text
projects/base/
projects/go2_control_demos/
projects/go2_nav_sim/
```

其中 `go2_control_demos/policies/*.pt` 不建议上传。如果需要保留接口：

```text
projects/go2_control_demos/policies/.gitkeep
projects/go2_control_demos/policies/README.md
```

说明模型权重需要从公司模型仓库或 release 下载。

### 3.2 不要上传

虚拟环境、构建产物、下载包、日志：

```text
.venv-unitree/
build/
install/
log/
logs/
downloads/
MUJOCO_LOG.TXT
```

官方源码建议不要直接塞主仓库：

```text
src/unitree_mujoco/
src/unitree_ros2/
src/unitree_sdk2/
src/unitree_sdk2_python/
opt/unitree_robotics/
```

建议改成：

```text
docs/third_party_unitree_setup.md
scripts/setup_unitree_thirdparty.sh
```

如果公司已经允许重新分发 Unitree 官方仓库，可以作为 submodule 或公司 fork 管理。

### 3.3 open source 项目目录

当前存在：

```text
projects/open_source_deploy_simtoreal_rl_go2/
projects/open_source_unitree_rl_gym/
```

这些是第三方开源项目，不建议原样上传到公司公共仓库。建议：

```text
1. README 中列为参考项目。
2. 使用 submodule 或 fork。
3. 只保留你自己的 wrapper / adapter / notes。
```

---

## 4. Isaac Go2 仿真项目上传清单

源目录：

```text
/home/ros/isaac_go2
```

当前主要内容是：

```text
IsaacLab/
IsaacLab-release_3.0.0-beta2.tar.gz
assets_cache/
```

### 4.1 不建议上传 IsaacLab 官方完整源码

`IsaacLab/` 是官方大项目，不建议原样放入公司公共仓库。更推荐：

```text
1. README 中说明 Isaac Lab 版本。
2. docs/isaaclab_setup.md 写安装步骤。
3. scripts/isaaclab_*.sh 放在 go2_control/scripts/ 或 isaac_go2_sim/scripts/。
4. 你自己的 task/env/asset extension 单独放到 isaac_go2_sim/source_extensions/。
```

### 4.2 不要上传

```text
IsaacLab-release_3.0.0-beta2.tar.gz
assets_cache/
IsaacLab/logs/
IsaacLab/.git/
```

### 4.3 可以上传

如果你在 IsaacLab 里写了自己的 Go2 task、env、asset 或训练脚本，需要单独挑出来，例如：

```text
isaac_go2_sim/scripts/
isaac_go2_sim/source_extensions/
isaac_go2_sim/docs/
```

如果目前还没有自定义 Isaac 扩展，只上传：

```text
unitree_dev/scripts/isaaclab_*.sh
unitree_dev/docs/IsaacSimLab_Go2复现指南.md
unitree_dev/docs/Isaac云服务器迁移指南.md
```

---

## 5. 推荐发布目录结构

建议新建目录：

```text
/home/ros/company_github_publish/indoor-navigation-go2
```

然后组织为：

```text
indoor-navigation-go2/
├── README.md
├── .gitignore
├── docs/
│   ├── architecture.md
│   ├── third_party_setup.md
│   ├── route_a_future_work.md
│   └── go2_simulation_setup.md
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

---

## 6. Route A 结果处理原则

用户明确不希望上传后来 Route A 产生的大量结果，因此：

```text
不上传 output/route_A*
不上传 static_map.png / nav_binary_map.png / dense_global_static.ply
不上传 route_A_v2 / route_A_v3_scarf / route_A_v3_2_scarf_official
不上传 depth_cache / keyframes / submaps / videos
```

保留接口和计划：

```text
src/people_bev_tracker/scripts/run_route_A_v3_pipeline.py
src/people_bev_tracker/scripts/build_route_A_v3_scarf_like.py
src/people_bev_tracker/scripts/diagnose_route_A_v3_map.py
src/people_bev_tracker/scripts/filter_people_tracks_on_map.py
src/people_bev_tracker/people_bev_tracker/person_map_filter.py
src/people_bev_tracker/docs/06_RouteA_V3_ScaRF_SLAM稠密重建与导航栅格执行方案_最新版.md
src/people_bev_tracker/docs/07_RouteA_V3_2官方ScaRF与地图诊断执行方案_最新版.md
```

这样仓库对外表达的是：

```text
本仓库已经实现 DPVO/ORB-SLAM3/YOLO/BEV 的基础接口；
Route A 的稠密地图、官方 ScaRF-SLAM、Figure 1 风格展示是后续计划；
运行结果由使用者自行生成，不进入 Git。
```

---

## 7. 建议 .gitignore

```gitignore
# Build outputs
build/
install/
log/
logs/
*.log

# Python caches
__pycache__/
*.py[cod]
.pytest_cache/
.mypy_cache/
.ruff_cache/

# Local environments
.venv/
.venv-*/
venv/
env/
.conda/
.cache/
.local/
.runtime/

# ROS / runtime
.ros/
.rviz2/
*.bag
*.db3

# Generated outputs
output/
outputs/
results/
runs/
wandb/
tensorboard/

# Large media / datasets
resources/*.mp4
resources/*.avi
resources/*.mov
*.mp4
*.avi
*.mov
*.mkv
*.jpg
*.jpeg
*.png
*.gif
*.pgm

# Models / point clouds / arrays
*.pt
*.pth
*.onnx
*.engine
*.ckpt
*.bin
*.ply
*.pcd
*.npy
*.npz

# Archives and papers
*.zip
*.tar
*.tar.gz
*.7z
*.pdf

# Third-party source dumps
project code/
thirdparty/
src/KV-tracker/thirdparty/

# Isaac / MuJoCo / Unitree local assets
assets_cache/
downloads/
opt/
MUJOCO_LOG.TXT

# IDE / agent local state
.vscode/
.idea/
.codex/
.agents/
.claude/
```

注意：

```text
如果 README 需要展示图片，建议单独建立 docs/assets/，并在 .gitignore 中为 docs/assets/ 开白名单。
```

---

## 8. 上传前检查清单

上传前必须检查：

```text
1. 仓库里没有 output/、build/、install/、log/。
2. 仓库里没有 .pt/.pth/.mp4/.pdf/.zip/.tar.gz。
3. 仓库里没有第三方官方源码大包，除非公司明确允许。
4. 仓库里没有个人缓存、conda/venv、.cache、.codex、.claude。
5. README 说明清楚第三方依赖如何安装。
6. README 说明 Route A 后续计划，但不包含大结果。
7. README 说明 Go2 真机控制需要安全条件，不建议直接上电运行未验证策略。
8. 如果仓库是 public，要确认视频数据、商场画面、人员图像没有隐私风险。
```

---

## 9. 一句话建议

```text
上传源码、接口、配置、启动脚本、核心文档；
不上传模型权重、视频数据、PDF、官方源码、Route A 结果、build/install/log/cache。
```
