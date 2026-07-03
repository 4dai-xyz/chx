# 当前状态记录

日期：2026-06-06

## 1. 总结

当前只保留两套 Go2 相关仿真/开发环境：

```text
1. Unitree MuJoCo / SDK2 / ROS2
   路径：/home/ros/unitree_dev

2. Isaac Sim / Isaac Lab
   路径：/home/ros/isaac_go2
   conda 环境：/home/ros/miniconda3/envs/env_isaaclab312
```

旧 Isaac 环境已经删除：

```text
/home/ros/miniconda3/envs/env_isaaclab
```

不完整 IsaacLab 下载残留已经删除。

ORB-SLAM3 工作区未被删除、未被修改：

```text
/home/ros/ros2_orbslam3
/home/ros/ros2_orbslam3/Opensource code/ORB_SLAM3-master
/home/ros/ros2_orbslam3/src/orbslam3_wrapper
```

## 2. 机器人型号

已确认：

```text
Unitree Go2
```

## 3. MuJoCo / SDK2 / ROS2 环境

主工作区：

```text
/home/ros/unitree_dev
```

源码仓库：

```text
/home/ros/unitree_dev/src/unitree_sdk2
/home/ros/unitree_dev/src/unitree_ros2
/home/ros/unitree_dev/src/unitree_mujoco
/home/ros/unitree_dev/src/unitree_sdk2_python
```

本地依赖：

```text
/home/ros/.mujoco/mujoco-3.3.6
/home/ros/unitree_dev/src/unitree_mujoco/simulate/mujoco -> /home/ros/.mujoco/mujoco-3.3.6
/home/ros/unitree_dev/.venv-unitree
/home/ros/unitree_dev/opt/unitree_robotics
```

Python MuJoCo 包已固定为：

```text
mujoco 3.3.6
```

原因：`mujoco 3.9.0` 在当前 WSLg viewer 创建阶段会卡住，降回官方仓库对应的 3.3.6 后，Python viewer 可以进入仿真线程。

关键系统包已安装：

```text
libglfw3-dev
ros-humble-rmw-cyclonedds-cpp
ros-humble-rosidl-generator-dds-idl
```

常用命令：

```bash
cd /home/ros/unitree_dev
bash scripts/run_mujoco_python.sh
```

C++ MuJoCo 运行脚本已修复：

```text
脚本入口：/home/ros/unitree_dev/scripts/run_mujoco_cpp.sh
实际启动：/home/ros/unitree_dev/src/unitree_mujoco/simulate/build/unitree_mujoco
配置文件：/home/ros/unitree_dev/src/unitree_mujoco/simulate/config.yaml
模型目录：/home/ros/unitree_dev/src/unitree_mujoco/unitree_robots
```

并额外保留兼容软链接：

```text
/home/ros/unitree_dev/build/config.yaml -> /home/ros/unitree_dev/src/unitree_mujoco/simulate/config.yaml
/home/ros/unitree_dev/unitree_robots -> /home/ros/unitree_dev/src/unitree_mujoco/unitree_robots
```

## 4. Isaac Sim / Isaac Lab 环境

当前唯一 Isaac 环境：

```text
工作区：/home/ros/isaac_go2
源码：/home/ros/isaac_go2/IsaacLab
conda 环境：/home/ros/miniconda3/envs/env_isaaclab312
资产缓存：/home/ros/isaac_go2/assets_cache
```

当前已确认：

```text
Python 3.12.13
Isaac Sim 6.0.0.0
PyTorch 2.10.0+cu128
Go2 task 已注册
```

常用命令：

```bash
cd /home/ros/unitree_dev
bash scripts/isaaclab_check.sh
NUM_ENVS=4 TASK=Isaac-Velocity-Flat-Unitree-Go2-v0 bash scripts/isaaclab_go2_play.sh
```

## 5. conda 环境

当前 conda 环境列表只保留：

```text
base
dpvo
env_isaaclab312
```

说明：

```text
dpvo 是其他项目环境，本次没有处理。
env_isaaclab312 是当前唯一 Isaac Sim/Lab 环境。
env_isaaclab 已删除。
```

## 6. 磁盘占用概况

最近一次检查：

```text
/home/ros/unitree_dev                  约 1.3GB
/home/ros/isaac_go2                    约 146MB，不含 conda 环境和全局缓存
/home/ros/miniconda3/envs/env_isaaclab312 约 11GB
/home/ros/.cache/pip                   约 21GB
/home/ros/.local/share/ov              约 2.3GB
```

`/home/ros/.cache/pip` 和 `/home/ros/.local/share/ov` 是下载/扩展缓存。为了后续少重复下载，当前保留。

## 7. 环境污染提示

当前 shell 仍可能出现：

```text
/home/ros/miniconda3/envs/dpvo/lib/libtinfo.so.6: no version information available
```

这说明你的默认 shell 里还有 DPVO/miniconda 的库路径残留。当前 Go2 脚本已尽量规避：

```text
MuJoCo/ROS2：source scripts/unitree_env.sh
Isaac：scripts/isaaclab_*.sh 自动清理相关变量
```

如果手动运行命令时出错，可以使用：

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV <你的命令>
```

## 8. 推荐下一步

```text
1. 跑 MuJoCo Python 仿真，熟悉 Go2 模型和控制接口。
2. 跑 Isaac check/play，确认 Isaac Go2 可视化任务。
3. 用 Isaac 小规模训练理解强化学习流程。
4. 只读连接真实 Go2，确认 topic 和状态。
5. 后续再把 ORB-SLAM3、导航、Go2 状态和传感器接起来。
```

## 9. 2026-06-20 Go2 Nav/SLAM 闭环状态

当前已经从单纯底层控制推进到 ROS2 导航/SLAM 仿真闭环阶段。

新增主线：

```text
Nav2 / slam_toolbox
  -> /cmd_vel
  -> Go2 MuJoCo ROS2 bridge
  -> Go2 RL policy runner
  -> /odom /scan /tf /clock
  -> Nav2 / slam_toolbox
```

当前优先使用成熟开源组件和现成开源地图：

```text
导航：Nav2
2D SLAM：slam_toolbox
静态地图：第一轮使用 ROS2/Nav2 TurtleBot3 示例地图 turtlebot3_world.yaml
底层控制：当前已验证稳定的 Go2 MuJoCo RL policy runner
```

已确认的关键通信问题和解决方案：

```text
问题：
  go2_mujoco_ros2_bridge.py 进程存在，Go2 能进入 RL_CONTROL，
  但 run_go2_cmd_vel_smoke_test.sh 提示找不到 /cmd_vel subscriber。

诊断：
  发布/订阅代码没有写错，都是 geometry_msgs/msg/Twist + /cmd_vel。
  问题出在 ROS2 DDS graph discovery。
  rmw_fastrtps_cpp 或 ROS_LOCALHOST_ONLY=1 在当前 WSL/本机环境中发现异常。

解决：
  使用 scripts/go2_ros_env.sh 统一所有 Go2 Nav/SLAM 脚本的 ROS2 环境。
  当前本机仿真固定为：
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ROS_LOCALHOST_ONLY=0
    GO2_CYCLONEDDS_INTERFACE=lo
    GO2_CYCLONEDDS_MAX_AUTO_INDEX=120
```

已验证的 ROS2 图关系：

```text
/go2_mujoco_ros2_bridge 可被 ros2 node list 发现。
/cmd_vel 有 go2_mujoco_ros2_bridge 订阅者。
/odom 有 go2_mujoco_ros2_bridge 发布者。
/scan 有 go2_mujoco_ros2_bridge 发布者。
```

Nav2 planner 调试结论：

```text
TurtleBot3 示例地图中 (0,0) 不是明确 free cell。
如果 AMCL 初始位姿使用 (0,0)，planner 可能报：
  GridBased: failed to create plan
  Planning algorithm GridBased failed to generate a valid path

此时 Go2 仍然运动，多半是 Nav2 recovery behavior，例如 spin/backup，
不代表正常路径规划成功。

当前修复：
  TurtleBot3 地图测试阶段使用 map-offset 为 (0.5, -0.5)，start 为 (0.5, -0.5)。
  当前复杂地图 go2_office_maze 默认 map-offset 为 (0.0, 0.0)，start 为 (0.0, 0.0)。
  nav2_send_goal.py 发送目标前会检查 start/goal 是否在地图 free cell。
  nav2_go2_params.yaml 将 local/global costmap inflation_radius 从 0.55 降到 0.25。
  nav2_go2_params.yaml 将 expected_planner_frequency 从 20.0 降到 5.0。
```

Nav2 近目标速度调试结论：

```text
当桥接日志出现类似：
  pose=(-2.12,-0.04) cmd=(-0.05,+0.00,+0.00)

说明机器人已经接近目标，但 Nav2 仍在用很小速度修正。
Go2 RL policy 对 0.05 m/s 级别的命令可能没有明显动作。

当前处理：
  nav2_go2_params.yaml 放宽第一轮验收目标：
    xy_goal_tolerance: 0.35
    yaw_goal_tolerance: 1.57

  nav2_go2_params.yaml 提高 DWB 最低速度采样：
    min_speed_xy: 0.08
    min_speed_theta: 0.15

  go2_mujoco_ros2_bridge.py 增加命令整形：
    abs(vx) < 0.03 -> 0
    0.03 <= abs(vx) < 0.10 -> sign(vx) * 0.10
    abs(vy) < 0.03 -> 0
    0.03 <= abs(vy) < 0.08 -> sign(vy) * 0.08
    abs(yaw) < 0.05 -> 0
    0.05 <= abs(yaw) < 0.18 -> sign(yaw) * 0.18
```

Nav2 goal 脚本曾出现：

```text
Failed to find a free participant index for domain 0
```

原因是 Nav2 多节点会占用较多 CycloneDDS participant index。当前处理：

```text
1. scripts/go2_ros_env.sh 设置 MaxAutoParticipantIndex=120。
2. scripts/run_go2_nav2.sh 使用 use_composition:=True。
3. scripts/run_go2_nav2_slam_map.sh 使用 use_composition:=True。
```

常用诊断命令：

```bash
cd /home/ros/unitree_dev
bash scripts/check_go2_ros_graph.sh
```

下一步执行顺序：

```text
1. /cmd_vel 冒烟测试：
   bash scripts/run_go2_cmd_vel_smoke_test.sh --wait 1

2. Nav2 + go2_office_maze 复杂地图目标点导航：
   bash scripts/run_go2_nav2.sh
   bash scripts/run_go2_nav2_goal.sh

3. slam_toolbox 在线建图：
   bash scripts/run_go2_slam_toolbox.sh
   bash scripts/run_go2_slam_mapping_drive.sh --duration 30

4. 保存地图并用自建地图重新跑 Nav2：
   bash scripts/save_go2_slam_map.sh
   bash scripts/run_go2_nav2_slam_map.sh
```

注意：

```text
selected interface "lo" is not multicast-capable: disabling multicast
```

这条 CycloneDDS 提示不视为失败。只要 ROS2 graph 能发现桥接节点和话题关系，本机仿真通信就是可用的。

后续真机连接时不要继续使用 `lo`，需要根据 `ip -br addr` 查到的真实网卡名设置：

```bash
GO2_CYCLONEDDS_INTERFACE=<真实网卡名>
```

## 10. 2026-06-20 复杂地图切换

当前默认 Nav2 静态地图已经从 TurtleBot3 示例地图切换为项目内复杂办公室/走廊地图：

```text
projects/go2_nav_sim/maps/go2_office_maze.yaml
projects/go2_nav_sim/maps/go2_office_maze.pgm
```

生成脚本：

```text
projects/go2_nav_sim/tools/generate_go2_office_maze_map.py
```

默认起点/目标：

```text
start: (0.0, 0.0)
goal:  (-4.0, -2.5)
```

相关脚本现在默认使用同一张复杂地图：

```text
scripts/run_go2_nav_bridge.sh
scripts/run_go2_nav_bridge_headless.sh
scripts/run_go2_nav2.sh
scripts/run_go2_nav2_goal.sh
scripts/run_go2_nav_rviz.sh
```

二维地图运动可视化：

```text
RViz 配置：projects/go2_nav_sim/config/go2_nav2_2d_view.rviz
启动命令：bash scripts/run_go2_nav_rviz.sh
```

可用环境变量切换地图：

```bash
GO2_NAV_MAP_YAML=/path/to/map.yaml
GO2_MAP_OFFSET_X=<x>
GO2_MAP_OFFSET_Y=<y>
```

重要边界：

```text
复杂地图当前用于 RViz /map、Nav2 costmap 和 bridge 根据地图生成的模拟 /scan。
MuJoCo viewer 里暂时仍是平地，不显示物理墙体，也不会和墙体真实碰撞。
后续如果需要物理墙体，再把 go2_office_maze 转成 MuJoCo MJCF box 障碍物。
```

Nav2 对旧默认目标 `(5.0, 3.5)` 曾出现：

```text
GridBased: failed to create plan with tolerance 0.50
Planning algorithm GridBased failed to generate a valid path to (5.00, 3.50)
```

当前修复：

```text
1. go2_office_maze 地图生成器加宽 `(0.0,0.0) -> (5.0,3.5)` 和 `(0.0,0.0) -> (-4.0,-2.5)` 主测试通道到约 1.6-2.0m。
2. 已重新生成 projects/go2_nav_sim/maps/go2_office_maze.pgm/yaml。
3. global_costmap 插件改为 ["static_layer", "inflation_layer"]。
4. local_costmap 继续使用 /scan 做局部障碍感知。
5. 本地连通性检查确认新默认目标 `(-4.0,-2.5)` 在 0.70m clearance 下仍可达。
```

修改地图或 Nav2 参数后必须重启 bridge/Nav2/RViz/goal，正在运行的 map_server 不会自动重新读取磁盘上的 PGM。

当前导航感知来源：

```text
不是视觉导航。
不是 MuJoCo 真实物理激光雷达。
当前是 bridge 根据二维地图 raycast 生成 /scan，Nav2/AMCL 使用 /map + /scan + /odom + /tf 完成导航。
视觉 SLAM / DPVO / ORB-SLAM3 仍是后续阶段。
```
