# Unitree Go2 开发工作区

当前工作区路径：

```text
/home/ros/unitree_dev
```

本工作区现在只保留两条清晰主线：

```text
1. Unitree MuJoCo / SDK2 / ROS2 环境
   用于 Go2 官方 MuJoCo 仿真、SDK2 开发、ROS2 通信和后续实机连接。

2. Isaac Sim / Isaac Lab 环境
   用于 Go2 GPU 仿真、强化学习训练、策略播放和后续 sim-to-real 学习。
```

两套环境相互独立。MuJoCo 主工作区在 `/home/ros/unitree_dev`，Isaac 主工作区在 `/home/ros/isaac_go2`，Isaac 的 Python 环境只保留 `/home/ros/miniconda3/envs/env_isaaclab312`。

## 当前结论

已经确认：

```text
机器人型号：Unitree Go2
系统：WSL2 Ubuntu 22.04
ROS2：Humble，路径 /opt/ros/humble
MuJoCo 工作区：/home/ros/unitree_dev
Isaac 工作区：/home/ros/isaac_go2
Isaac conda 环境：/home/ros/miniconda3/envs/env_isaaclab312
```

已经安装并确认的关键系统依赖：

```text
libglfw3-dev
ros-humble-rmw-cyclonedds-cpp
ros-humble-rosidl-generator-dds-idl
```

已经清理：

```text
旧 Isaac 环境：/home/ros/miniconda3/envs/env_isaaclab
IsaacLab 不完整下载残留目录
IsaacLab 不完整压缩包
```

未清理、未影响：

```text
/home/ros/ros2_orbslam3
/home/ros/ros2_orbslam3/Opensource code/ORB_SLAM3-master
/home/ros/ros2_orbslam3/src/orbslam3_wrapper
/home/ros/miniconda3/envs/dpvo
```

## 推荐阅读顺序

零基础学习主线：

```text
docs/Go2从0开始教程.md
```

仿真、运动控制、SLAM、具身导航总路线：

```text
docs/仿真运动控制与具身导航最快入门路线.md
```

Isaac Sim/Lab 专项：

```text
docs/IsaacSimLab_Go2复现指南.md
```

只查命令：

```text
docs/命令速查.md
```

当前状态：

```text
notes/current_status.md
```

## 最终文件架构

```text
/home/ros
├── unitree_dev/                         # Go2 MuJoCo / SDK2 / ROS2 主工作区
│   ├── src/
│   │   ├── unitree_mujoco/              # Unitree 官方 MuJoCo 仿真器和 Go2 模型
│   │   ├── unitree_sdk2/                # C++ SDK2，实机/仿真 DDS 通信
│   │   ├── unitree_sdk2_python/         # Python SDK2
│   │   └── unitree_ros2/                # Go2 ROS2 消息和示例
│   ├── scripts/                         # 一键安装、构建、运行脚本
│   ├── docs/                            # 中文教程
│   ├── notes/                           # 当前状态和快速指南
│   ├── .venv-unitree/                   # MuJoCo / SDK2 Python 虚拟环境
│   ├── build/                           # CMake/colcon 构建输出
│   ├── install/                         # ROS2 构建安装空间
│   └── opt/unitree_robotics/            # SDK2 本地安装前缀
│
├── isaac_go2/                           # Isaac Sim / Isaac Lab 主工作区
│   ├── IsaacLab/                        # 当前唯一保留的 Isaac Lab 源码
│   ├── IsaacLab-release_3.0.0-beta2.tar.gz
│   └── assets_cache/                    # Isaac/Go2 USD 资产缓存
│
├── miniconda3/envs/env_isaaclab312/     # 当前唯一 Isaac Sim/Lab Python 环境
├── miniconda3/envs/dpvo/                # 其他项目环境，和 Go2/Isaac 独立
└── ros2_orbslam3/                       # ORB-SLAM3 工作区，未被本次清理影响
```

## 两套仿真环境的区别

### Unitree MuJoCo

适合：

```text
快速看到 Go2 动起来
学习 Unitree SDK2 通信
学习 ROS2 topic/message
写低层控制或简单运动控制实验
后续把仿真控制程序迁移到真实 Go2
```

运行时使用：

```text
/home/ros/unitree_dev/.venv-unitree
/home/ros/unitree_dev/src/unitree_mujoco
/home/ros/unitree_dev/src/unitree_sdk2_python
/home/ros/unitree_dev/scripts/unitree_env.sh
```

### Isaac Sim / Isaac Lab

适合：

```text
GPU 物理仿真
强化学习训练
粗糙地形速度跟踪
策略 checkpoint 播放
传感器仿真和后续 sim-to-real 学习
```

运行时使用：

```text
/home/ros/miniconda3/envs/env_isaaclab312
/home/ros/isaac_go2/IsaacLab
/home/ros/isaac_go2/assets_cache
/home/ros/unitree_dev/scripts/isaaclab_*.sh
```

## 为什么不会互相干扰

MuJoCo 脚本会加载 `scripts/unitree_env.sh`，主要面向 ROS2 Humble、CycloneDDS、SDK2 和 `.venv-unitree`。

Isaac 脚本会直接指定：

```text
/home/ros/miniconda3/envs/env_isaaclab312/bin
```

并且运行前清理：

```text
PYTHONPATH
LD_LIBRARY_PATH
CONDA_PREFIX
CONDA_DEFAULT_ENV
CONDA_SHLVL
CONDA_PROMPT_MODIFIER
```

所以日常建议：

```text
跑 MuJoCo 时使用 scripts/run_mujoco_*.sh
跑 Isaac 时使用 scripts/isaaclab_*.sh
不要手动混用 conda activate 和 source /opt/ros/humble/setup.bash
```

## 最短运行命令

运行 MuJoCo Python 仿真：

```bash
cd /home/ros/unitree_dev
bash scripts/run_mujoco_python.sh
```

另开终端跑 MuJoCo 通信测试：

```bash
cd /home/ros/unitree_dev
bash scripts/run_mujoco_python_test.sh
```

检查 Isaac 环境：

```bash
cd /home/ros/unitree_dev
bash scripts/isaaclab_check.sh
```

Isaac 平台限制：

```text
当前工作区运行在 WSL2。NVIDIA 官方不支持在 WSL2 中运行 Isaac Sim
Python/Kit 图形程序，因此本机 WSL 中不能把 Isaac 可视化窗口作为可用功能。
实时画面请使用当前 MuJoCo，或把 Isaac 迁移到原生 Windows/Ubuntu/远程 Linux。
```

下面的小规模训练命令仅作为原生受支持系统上的参考；WSL2 中属于非官方实验路径：

```bash
cd /home/ros/unitree_dev
NUM_ENVS=16 MAX_ITERATIONS=50 TASK=Isaac-Velocity-Flat-Unitree-Go2-v0 bash scripts/isaaclab_go2_train_small.sh
```

## 后续学习路线

建议顺序：

```text
1. Linux/WSL 基础：路径、终端、环境变量、source、bash 脚本
2. Go2 MuJoCo：先跑仿真窗口，再理解 scene.xml/go2.xml/config.py
3. SDK2 Python：学习 ChannelFactoryInitialize、订阅状态、发布命令
4. ROS2：学习 topic、message、service、colcon、CycloneDDS
5. 实机只读连接：先读状态，不发运动命令
6. MuJoCo 控制程序：在仿真里写自己的运动控制 demo
7. Isaac Lab：理解 task、env、observation、action、reward、PPO
8. Isaac 训练：从小 num_envs 开始训练 Go2 velocity task
9. SLAM/导航：再把 /home/ros/ros2_orbslam3、ROS2 navigation、Go2 里程计和传感器接起来
10. sim-to-real：确认急停和安全空间后，再把仿真程序迁移到实机
```

## 安全提醒

第一次连接真实 Go2 时只做只读：

```bash
cd /home/ros/unitree_dev
export UNITREE_NET_IF=eth0
source scripts/unitree_env.sh
ros2 topic list
ros2 topic echo /lf/sportmodestate
ros2 topic echo /lf/lowstate
ros2 topic echo /wirelesscontroller
```

不要一开始运行底层控制或站立示例：

```text
go2_sport_client
go2_stand_example
go2_low_level
stand_go2.py eth0
```

这些程序可能让真实机器狗运动。
