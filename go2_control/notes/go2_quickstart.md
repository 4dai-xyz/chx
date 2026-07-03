# Go2 快速指南

本指南只给最短路线。详细解释请看：

```text
docs/Go2从0开始教程.md
docs/IsaacSimLab_Go2复现指南.md
docs/命令速查.md
```

## 1. 当前最终环境

你的实机已经确认是：

```text
Unitree Go2
```

当前只保留两套 Go2 相关环境：

```text
MuJoCo / SDK2 / ROS2：/home/ros/unitree_dev
Isaac Sim / Isaac Lab：/home/ros/isaac_go2 + env_isaaclab312
```

旧 Isaac 环境 `env_isaaclab` 已删除。

ORB-SLAM3 工作区仍在：

```text
/home/ros/ros2_orbslam3
```

## 2. MuJoCo 快速运行

进入工作区：

```bash
cd /home/ros/unitree_dev
```

运行 Python 版 MuJoCo：

```bash
bash scripts/run_mujoco_python.sh
```

另开终端跑通信测试：

```bash
cd /home/ros/unitree_dev
bash scripts/run_mujoco_python_test.sh
```

C++ 版 MuJoCo：

```bash
cd /home/ros/unitree_dev
bash scripts/run_mujoco_cpp.sh
```

## 3. Isaac 快速运行

检查 Isaac 环境：

```bash
cd /home/ros/unitree_dev
bash scripts/isaaclab_check.sh
```

运行 Go2 平地任务：

```bash
cd /home/ros/unitree_dev
NUM_ENVS=4 TASK=Isaac-Velocity-Flat-Unitree-Go2-v0 bash scripts/isaaclab_go2_play.sh
```

小规模训练：

```bash
cd /home/ros/unitree_dev
NUM_ENVS=16 MAX_ITERATIONS=50 TASK=Isaac-Velocity-Flat-Unitree-Go2-v0 bash scripts/isaaclab_go2_train_small.sh
```

## 4. Go2 实机网络

Windows 物理以太网卡设置：

```text
IP 地址：192.168.123.99
子网掩码：255.255.255.0
网关：留空
DNS：留空
```

WSL 检查：

```bash
cd /home/ros/unitree_dev
bash scripts/check_network.sh
```

## 5. 第一次实机只读

```bash
cd /home/ros/unitree_dev
export UNITREE_NET_IF=eth0
source scripts/unitree_env.sh
ros2 topic list
ros2 topic echo /lf/sportmodestate
ros2 topic echo /lf/lowstate
ros2 topic echo /wirelesscontroller
```

如果 `ros2 topic list` 显示的 topic 没有前导 `/`，就按实际显示名称输入。

## 6. 安全提醒

第一次不要直接运行：

```text
go2_sport_client
go2_stand_example
go2_low_level
stand_go2.py eth0
```

这些程序可能让真实 Go2 运动。
