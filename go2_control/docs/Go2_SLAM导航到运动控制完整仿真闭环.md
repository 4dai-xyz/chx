# Go2 从 SLAM/导航大脑到 MuJoCo RL 底层控制的完整仿真闭环

本文档记录当前项目从“Go2 底层强化学习控制已经稳定”继续往上走的下一步：把 ROS2 导航/SLAM 大脑接到 MuJoCo 中的 Go2 RL 底层控制器，先跑通完整闭环，再逐步替换成视觉 SLAM、Isaac Sim 场景和真机接口。

## 1. 当前最推荐的开源路线

现阶段最稳的路线不是一上来就把 ORB-SLAM3、Isaac Sim、MuJoCo、Nav2 全部硬接起来，而是先做一个最小闭环：

```text
Nav2 / SLAM 大脑
  -> 发布 /cmd_vel
  -> Go2 MuJoCo ROS2 桥接节点
  -> Go2 RL policy 底层控制器
  -> MuJoCo 里真实运动
  -> 发布 /odom、/scan、/tf、/clock
  -> 回到 Nav2 / SLAM
```

这样做的原因：

1. 你的 Go2 RL 底层控制已经验证过稳定，不应该重新发明底层控制。
2. ROS2 导航生态默认使用 `/cmd_vel`、`/odom`、`/scan`、`/tf`，只要把这些标准接口接好，后续换算法成本很低。
3. Nav2 和 slam_toolbox 已经安装在系统 ROS2 Humble 中，不需要重新下载大依赖。
4. Nav2 自带 TurtleBot3 示例地图可以作为回退对照；当前默认改用项目内生成的 `go2_office_maze` 复杂办公室/走廊地图，给规划器和局部避障更大压力。
5. ORB-SLAM3、DPVO、VGGT 后续可以先作为“定位模块”替换 `/odom` 或 `map->odom`，而不是一开始就接管所有系统。

推荐开源算法/模块：

| 模块 | 推荐方案 | 当前作用 |
|---|---|---|
| 全局/局部导航 | Nav2 | 负责从目标点规划路径，并输出 `/cmd_vel` |
| 2D SLAM | slam_toolbox | 后续用 `/scan + /odom` 建图或定位 |
| 视觉 SLAM | ORB-SLAM3 / DPVO | 后续替换或校正定位来源 |
| 底层运动控制 | 当前已跑通的 Go2 RL policy runner | 接收速度命令，输出 12 关节目标/力矩 |
| 动力学仿真 | MuJoCo | 验证 Go2 真实运动稳定性 |
| 高真实感场景 | Isaac Sim | 后续用于相机、IMU、视觉导航验证 |

官方/开源参考：

- Nav2: `ros-navigation/navigation2`
- slam_toolbox: `SteveMacenski/slam_toolbox`
- 当前默认测试地图：`projects/go2_nav_sim/maps/go2_office_maze.yaml`
- 回退对照地图：`/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml`

### 1.1 开源算法和开源地图原则

当前阶段优先采用成熟开源算法和标准 Nav2 地图格式，不自研导航或 SLAM 算法：

1. 导航使用 Nav2，负责全局规划、局部控制、目标点导航和 `/cmd_vel` 输出。
2. 2D SLAM 使用 slam_toolbox，负责 `/scan + /odom -> /map` 在线建图。
3. 静态地图第一轮用 ROS2/Nav2 已安装的 TurtleBot3 示例地图 `turtlebot3_world.yaml` 排除接口误差；当前进入复杂路径规划测试后，默认使用项目内确定性生成的 `go2_office_maze.yaml`。
4. 后续如果下载到更合适的开源室内地图，只需要设置 `GO2_NAV_MAP_YAML=/path/to/map.yaml`，不需要改导航算法。
5. 底层运动继续使用当前已验证稳定的 Go2 MuJoCo RL policy runner，不重新写底层步态控制。
6. ORB-SLAM3、DPVO、Isaac Sim 相机传感器放到 Nav2 + slam_toolbox 闭环跑通之后，再作为视觉定位来源接入。

## 2. 本次新增的文件

新增目录：

```text
projects/go2_nav_sim/
```

文件说明：

```text
projects/go2_nav_sim/go2_mujoco_ros2_bridge.py
```

核心桥接节点。它会：

1. 加载你已经验证稳定的 `go2_mujoco_rl_policy_runner.py`。
2. 在 MuJoCo 中启动 Go2 RL 底层控制。
3. 订阅 ROS2 `/cmd_vel`。
4. 把 `/cmd_vel` 转成 RL policy 速度命令。
5. 发布 `/odom`。
6. 根据地图模拟 `/scan`。
7. 发布 `/tf`：`odom -> base_link`，以及 `base_link -> base_scan`。
8. 发布 `/clock`，供 Nav2 的 `use_sim_time:=True` 使用。

```text
projects/go2_nav_sim/cmd_vel_smoke_test.py
```

最小冒烟测试脚本，不经过 Nav2，直接发布 `/cmd_vel`。它用于先验证：

```text
ROS2 /cmd_vel -> MuJoCo Go2 RL 控制器 -> Go2 运动
```

如果这个测试不通，不要急着跑 Nav2。

```text
projects/go2_nav_sim/nav2_send_goal.py
```

Nav2 目标点发送脚本。它会调用 Nav2 的 NavigateToPose action，给机器人发送一个地图坐标系下的目标点。

```text
projects/go2_nav_sim/config/turtlebot3_world.yaml
projects/go2_nav_sim/config/turtlebot3_world.pgm
```

从 Nav2 官方示例中复制出来的开源室内地图。当前作为回退对照地图保留。

```text
projects/go2_nav_sim/maps/go2_office_maze.yaml
projects/go2_nav_sim/maps/go2_office_maze.pgm
projects/go2_nav_sim/tools/generate_go2_office_maze_map.py
```

项目内生成的复杂办公室/走廊测试地图。它包含外墙、内部墙、门洞、家具状障碍物，以及两条约 1.6-2.0m 宽的主测试通道。默认起点 `(0.0, 0.0)`，默认目标点 `(-4.0, -2.5)`，用于避开当前 MuJoCo 场景中 Go2 前方台阶区域。`(5.0, 3.5)` 仍作为可选远目标保留。

```text
projects/go2_nav_sim/config/go2_nav2_2d_view.rviz
```

二维顶视图 RViz 配置，用于观察 Go2 在复杂地图里的运动、激光、costmap 和 Nav2 路径。

```text
projects/go2_nav_sim/config/nav2_go2_params.yaml
```

基于 Nav2 官方参数改出来的 Go2 版本：

1. 速度上限更保守，避免 Nav2 输出过猛。
2. AMCL 改为 `OmniMotionModel`，更适合 Go2 这种可横移底盘。
3. AMCL 的 `base_frame_id` 使用 `base_link`，和桥接节点发布的 TF 保持一致。
4. 保留 `/scan` 作为局部/全局代价地图输入。

```text
projects/go2_nav_sim/config/slam_toolbox_go2_mapper.yaml
```

给 `slam_toolbox` 建图阶段使用的参数文件，坐标和话题匹配当前桥接节点：

```text
/scan
odom
base_link
map
```

```text
projects/go2_nav_sim/slam_mapping_drive.py
```

建图阶段用的慢速脚本化 `/cmd_vel` 发布器。它会让 Go2 做保守的前进、转向和横移，让 `slam_toolbox` 能收到连续的 `/scan + /odom` 变化。

新增启动脚本：

```text
scripts/go2_ros_env.sh
scripts/check_go2_ros_graph.sh
scripts/run_go2_nav_bridge.sh
scripts/run_go2_nav_bridge_headless.sh
scripts/run_go2_cmd_vel_smoke_test.sh
scripts/run_go2_nav2.sh
scripts/run_go2_nav2_goal.sh
scripts/run_go2_slam_toolbox.sh
scripts/run_go2_slam_mapping_drive.sh
scripts/save_go2_slam_map.sh
scripts/run_go2_nav2_slam_map.sh
```

其中：

```text
scripts/go2_ros_env.sh
```

所有 Go2 Nav/SLAM 脚本共用的 ROS2 通信环境入口。当前本机仿真默认：

```text
ROS_DOMAIN_ID=0
ROS_LOCALHOST_ONLY=0
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
GO2_CYCLONEDDS_INTERFACE=lo
GO2_CYCLONEDDS_MAX_AUTO_INDEX=120
```

```text
scripts/check_go2_ros_graph.sh
```

通信诊断脚本，用来确认 ROS2 图里能看到桥接节点、`/cmd_vel` 订阅关系、`/odom` 和 `/scan` 发布关系。

### 2.1 ROS2/DDS 通信问题复盘

本次调试中出现过一个容易误判的问题：桥接进程已经存在，Go2 也能进入 `RL_CONTROL`，但 `run_go2_cmd_vel_smoke_test.sh` 一直提示：

```text
waiting for /cmd_vel subscriber
```

排查过程：

1. 检查代码后确认发布/订阅没有写错：桥接节点订阅绝对话题 `/cmd_vel`，烟测脚本发布绝对话题 `/cmd_vel`，消息类型都是 `geometry_msgs/msg/Twist`。
2. `check_go2_ros_graph.sh` 起初只能在进程层看到 `go2_mujoco_ros2_bridge.py`，但 ROS2 图里看不到 `/go2_mujoco_ros2_bridge`、`/cmd_vel`、`/odom`、`/scan`。
3. 这说明问题在 DDS 发现层，不在 Go2 控制逻辑，也不在 `/cmd_vel` topic 名称。
4. `rmw_fastrtps_cpp` 或 `ROS_LOCALHOST_ONLY=1` 在当前 WSL/本机环境中会导致节点发现异常。
5. 切换为 `rmw_cyclonedds_cpp`，并显式使用 loopback 接口 `lo` 后，ROS2 图发现恢复正常。
6. 启动 Nav2 后再运行 goal 脚本时，曾出现 `Failed to find a free participant index for domain 0`。这是 Nav2 多节点占用 CycloneDDS participant index，默认自动 index 范围不够导致的。当前通过 `MaxAutoParticipantIndex=120` 和 Nav2 composition 模式解决。

当前已验证成功的本机仿真配置：

```text
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ROS_LOCALHOST_ONLY=0
GO2_CYCLONEDDS_INTERFACE=lo
GO2_CYCLONEDDS_MAX_AUTO_INDEX=120
```

成功诊断输出应包含：

```text
== ROS2 nodes ==
/go2_mujoco_ros2_bridge

== /cmd_vel graph ==
Type: geometry_msgs/msg/Twist
Publisher count: 0
Subscription count: 1
Node name: go2_mujoco_ros2_bridge

== /odom graph ==
Publisher count: 1

== /scan graph ==
Publisher count: 1
```

看到下面提示时不视为失败：

```text
selected interface "lo" is not multicast-capable: disabling multicast
```

只要 ROS2 图能发现桥接节点和话题关系，就说明同机仿真通信可用。后续真机联网时不要使用 `lo`，需要根据 `ip -br addr` 查到的真实网卡名设置 `GO2_CYCLONEDDS_INTERFACE`。

## 3. 第一步：只跑桥接节点

终端 1：

```bash
cd /home/ros/unitree_dev
bash scripts/run_go2_nav_bridge.sh
```

启动日志最前面应出现：

```text
Go2 ROS2 env: ROS_DOMAIN_ID=0, ROS_LOCALHOST_ONLY=0, RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
Go2 ROS2 env: CYCLONEDDS interface=lo
Go2 ROS2 env: CYCLONEDDS max_auto_participant_index=120
```

预期现象：

1. 打开 MuJoCo viewer。
2. Go2 先站起。
3. 日志出现：

```text
state=PD_STAND_UP
state=STABILIZE
state=RL_CONTROL
```

进入 `RL_CONTROL` 后说明 RL policy 已经接管。

如果你不想打开窗口：

```bash
cd /home/ros/unitree_dev
bash scripts/run_go2_nav_bridge_headless.sh --duration 12
```

另开终端确认 ROS2 图：

```bash
cd /home/ros/unitree_dev
bash scripts/check_go2_ros_graph.sh
```

需要能看到 `/go2_mujoco_ros2_bridge`，并且 `/cmd_vel` 的 `Subscription count` 为 1。

## 4. 第二步：不用 Nav2，先验证 /cmd_vel 能驱动 Go2

终端 1 保持桥接节点运行。

终端 2：

```bash
cd /home/ros/unitree_dev
bash scripts/run_go2_cmd_vel_smoke_test.sh
```

如果桥接节点已经被发现，烟测脚本会输出：

```text
/cmd_vel subscriber count=1, starting scripted motion.
```

默认动作：

```text
等待 Go2 站稳
前进
原地转向
向左横移
向右横移
停止
```

如果终端 1 的桥接日志里出现类似：

```text
cmd=(+0.25,+0.00,+0.00) cmd_publishers=1
pose=(...)
```

并且 MuJoCo 画面中 Go2 真的移动，说明：

```text
ROS2 大脑接口 -> Go2 RL 底层控制
```

已经打通。

可以改成更保守的测试：

```bash
cd /home/ros/unitree_dev
bash scripts/run_go2_cmd_vel_smoke_test.sh \
  --forward-vx 0.15 \
  --turn-yaw 0.25 \
  --strafe-vy 0.10
```

可以改成更快的测试：

```bash
cd /home/ros/unitree_dev
bash scripts/run_go2_cmd_vel_smoke_test.sh \
  --forward-vx 0.35 \
  --turn-yaw 0.50 \
  --strafe-vy 0.25
```

## 5. 第三步：启动 Nav2

终端 1：先启动桥接节点。

```bash
cd /home/ros/unitree_dev
bash scripts/run_go2_nav_bridge.sh
```

默认会使用复杂地图：

```text
projects/go2_nav_sim/maps/go2_office_maze.yaml
```

桥接节点会把 MuJoCo 世界原点映射到地图 `(0.0, 0.0)`，这个点在复杂地图里是 free cell。

终端 2：启动 Nav2。

```bash
cd /home/ros/unitree_dev
bash scripts/run_go2_nav2.sh
```

`run_go2_nav2.sh` 默认使用 Nav2 composition 模式，减少独立 DDS participant 数量，避免 CycloneDDS 在本机仿真中出现：

```text
Failed to find a free participant index for domain 0
```

这会启动：

```text
map_server
amcl
planner_server
controller_server
bt_navigator
velocity_smoother
lifecycle_manager
```

其中：

```text
Nav2 controller 输出 /cmd_vel
桥接节点订阅 /cmd_vel
桥接节点发布 /odom /scan /tf /clock
```

## 6. 第四步：发送 Nav2 目标点

终端 3：

```bash
cd /home/ros/unitree_dev
bash scripts/run_go2_nav2_goal.sh
```

默认等价于：

```bash
cd /home/ros/unitree_dev
bash scripts/run_go2_nav2_goal.sh \
  --start-x 0.0 \
  --start-y 0.0 \
  --goal-x -4.0 \
  --goal-y -2.5 \
  --goal-yaw 0.0
```

目标脚本会在真正发送 Nav2 goal 前检查起点和目标点是否是 free cell。复杂地图默认应该看到：

```text
[检查] start=(0.00,0.00) map_status=free
[检查] goal=(-4.00,-2.50) map_status=free
```

注意：`(-4.0,-2.5)` 是 ROS `map` 坐标系下的目标。RViz 如果旋转了顶视图，屏幕里的“右下角/左下角”可能和坐标正负方向看起来不一致，以坐标值为准。

如果成功，Nav2 会规划路径并持续输出 `/cmd_vel`，Go2 会在 MuJoCo 中跟着运动。

可以尝试其他目标：

```bash
cd /home/ros/unitree_dev
bash scripts/run_go2_nav2_goal.sh \
  --goal-x -3.5 \
  --goal-y -2.5 \
  --goal-yaw 0.0
```

或者：

```bash
cd /home/ros/unitree_dev
bash scripts/run_go2_nav2_goal.sh \
  --goal-x 5.0 \
  --goal-y 3.5 \
  --goal-yaw 0.0
```

如果要切回 Nav2 官方 TurtleBot3 示例地图做对照测试，三个终端都使用同一个环境变量。

桥接终端：

```bash
cd /home/ros/unitree_dev
export GO2_NAV_MAP_YAML=/home/ros/unitree_dev/projects/go2_nav_sim/config/turtlebot3_world.yaml
export GO2_MAP_OFFSET_X=0.5
export GO2_MAP_OFFSET_Y=-0.5
bash scripts/run_go2_nav_bridge.sh
```

Nav2 终端：

```bash
cd /home/ros/unitree_dev
export GO2_NAV_MAP_YAML=/home/ros/unitree_dev/projects/go2_nav_sim/config/turtlebot3_world.yaml
bash scripts/run_go2_nav2.sh
```

目标点终端：

```bash
cd /home/ros/unitree_dev
export GO2_NAV_MAP_YAML=/home/ros/unitree_dev/projects/go2_nav_sim/config/turtlebot3_world.yaml
bash scripts/run_go2_nav2_goal.sh \
  --start-x 0.5 \
  --start-y -0.5 \
  --goal-x -2.0 \
  --goal-y 0.0 \
  --goal-yaw 0.0
```

## 6.5 第五步：slam_toolbox 建图并保存地图

这一阶段不要启动 `scripts/run_go2_nav2.sh`，因为 AMCL 和 `slam_toolbox` 都会发布 `map -> odom`，同时运行会造成 TF 冲突。

终端 1：启动桥接节点。建图时不要加 `--publish-map`，让 `/map` 由 `slam_toolbox` 发布。

```bash
cd /home/ros/unitree_dev
bash scripts/run_go2_nav_bridge.sh
```

终端 2：启动 `slam_toolbox` 在线建图。

```bash
cd /home/ros/unitree_dev
bash scripts/run_go2_slam_toolbox.sh
```

终端 3：让 Go2 做一段保守巡航，用 `/scan + /odom` 生成地图。

```bash
cd /home/ros/unitree_dev
bash scripts/run_go2_slam_mapping_drive.sh
```

默认巡航 90 秒。想先短测可以用：

```bash
cd /home/ros/unitree_dev
bash scripts/run_go2_slam_mapping_drive.sh --duration 30
```

终端 4：地图稳定后保存。

```bash
cd /home/ros/unitree_dev
bash scripts/save_go2_slam_map.sh
```

默认会生成：

```text
projects/go2_nav_sim/maps/go2_slam_map.yaml
projects/go2_nav_sim/maps/go2_slam_map.pgm
```

保存后，关闭 `slam_toolbox`，保留或重新启动桥接节点，再用刚建好的地图启动 Nav2：

```bash
cd /home/ros/unitree_dev
bash scripts/run_go2_nav2_slam_map.sh
```

然后继续用目标点脚本测试路径规划：

```bash
cd /home/ros/unitree_dev
export GO2_NAV_MAP_YAML=/home/ros/unitree_dev/projects/go2_nav_sim/maps/go2_slam_map.yaml
bash scripts/run_go2_nav2_goal.sh \
  --goal-x 1.0 \
  --goal-y 0.0 \
  --goal-yaw 0.0
```

## 7. RViz 观察

如果想看 Go2 在二维复杂地图里怎么运动，使用专用 RViz 顶视图。

终端 3：启动 RViz。

```bash
cd /home/ros/unitree_dev
bash scripts/run_go2_nav_rviz.sh
```

终端 4：发送目标点。

```bash
cd /home/ros/unitree_dev
bash scripts/run_go2_nav2_goal.sh
```

RViz 里重点看这些显示项：

```text
Map                /map                          复杂二维地图
Go2 Odom Arrow     /odom                         Go2 在二维地图中的当前位置和朝向
LaserScan          /scan                         桥接节点根据二维地图 raycast 出来的模拟激光
Global Costmap     /global_costmap/costmap       Nav2 全局代价地图
Local Costmap      /local_costmap/costmap        Go2 周围局部代价地图
Global Plan        /plan                         Nav2 全局规划路径
Local Plan         /local_plan                   DWB 局部控制器短期轨迹
Go2 Footprint      /local_costmap/published_footprint  Nav2 认为的机器人占用轮廓
```

Fixed Frame 选：

```text
map
```

如果只想看桥接节点，不启动 Nav2，可以把 Fixed Frame 改成：

```text
odom
```

更推荐的四终端观察顺序：

```text
终端 1：bash scripts/run_go2_nav_bridge.sh
终端 2：bash scripts/run_go2_nav2.sh
终端 3：bash scripts/run_go2_nav_rviz.sh
终端 4：bash scripts/run_go2_nav2_goal.sh
```

这时 MuJoCo viewer 看 Go2 的真实步态和稳定性，RViz 看 Go2 在复杂二维地图里的位置、路径和避障过程。

## 8. 当前闭环的工作原理

### 8.1 Nav2 做什么

Nav2 是 ROS2 中最成熟的移动机器人导航栈。它主要负责：

1. 读取地图 `/map`。
2. 读取当前定位 `map -> odom -> base_link`。
3. 读取障碍物传感器 `/scan`。
4. 给定目标点后规划路径。
5. 局部控制器跟踪路径。
6. 输出速度命令 `/cmd_vel`。

当前目标点导航的实际执行链路是：

```text
scripts/run_go2_nav2_goal.sh
  -> projects/go2_nav_sim/nav2_send_goal.py
  -> BasicNavigator.goToPose(goal)
  -> Nav2 NavigateToPose action
  -> bt_navigator 行为树
  -> planner_server / GridBased NavfnPlanner 生成全局路径
  -> controller_server / DWBLocalPlanner 跟踪路径
  -> /cmd_vel
  -> go2_mujoco_ros2_bridge.py::_on_cmd_vel()
  -> Go2MujocoRlPolicyRunner.step(sim_time, cmd)
  -> Go2MujocoRlPolicyRunner._build_obs(cmd_vel)
  -> policy_rough.pt
  -> 12 个关节目标
  -> MuJoCo PD 内环
```

其中真正发送目标的是 `nav2_send_goal.py` 里的 `BasicNavigator.goToPose(goal)`；真正接收速度并喂给 Go2 RL policy 的是 `go2_mujoco_ros2_bridge.py` 里的 `_on_cmd_vel()` 和 `_timer_step()`。

当前导航感知来源是：

```text
定位/运动反馈：MuJoCo root state -> /odom 和 odom->base_link
障碍物感知：二维地图 raycast -> /scan
全局地图：go2_office_maze.yaml -> /map
定位融合：Nav2 AMCL 使用 /map + /scan + /odom
路径规划：Nav2 planner/controller 使用 /map、costmap、/odom、/tf
```

所以当前不是视觉导航，也不是 ORB-SLAM3/DPVO 在提供位姿；视觉模块还没有进入这轮闭环。它更接近“理想二维激光雷达 + 轮式/里程计导航”的 Nav2 验证流程，只是底层执行机器人换成了 MuJoCo 中的 Go2 RL policy。

### 8.2 Go2 桥接节点做什么

`go2_mujoco_ros2_bridge.py` 把 Go2 RL runner 包成 ROS2 机器人。

对外它像一个标准 ROS2 移动机器人：

```text
输入：/cmd_vel
输出：/odom /scan /tf /clock
```

内部它调用：

```text
Go2MujocoRlPolicyRunner.step(sim_time, cmd_vel)
```

底层仍然是你已经验证稳定的：

```text
52 维 obs -> policy_rough.pt -> 12 维 action -> 目标关节角 -> MuJoCo PD 内环
```

注意：Go2 RL policy 对很小的速度命令不一定有明显动作。当前桥接层会对 Nav2 输出做轻量整形：

```text
abs(vx) < 0.03            -> 0
0.03 <= abs(vx) < 0.10   -> sign(vx) * 0.10

abs(vy) < 0.03            -> 0
0.03 <= abs(vy) < 0.08   -> sign(vy) * 0.08

abs(yaw) < 0.05           -> 0
0.05 <= abs(yaw) < 0.18  -> sign(yaw) * 0.18
```

这样可以避免 Nav2 长时间输出 `cmd=(-0.05,0,0)`，但 Go2 policy 几乎不动的状态。

### 8.3 /scan 怎么来的

当前 `/scan` 不是物理激光雷达模型，而是桥接节点根据二维地图做 ray casting 得到的模拟激光。

因此当前复杂地图会体现在：

```text
RViz /map
Nav2 global_costmap
Nav2 local_costmap
/scan 障碍物距离
planner/controller 输出
```

但 MuJoCo viewer 里暂时仍然是 Go2 在平地上运动，不会显示这些墙体，也不会和墙体发生真实物理碰撞。这个取舍是为了先验证 Nav2 复杂地图规划闭环，不干扰已经稳定的 Go2 RL 底层控制。后续如果需要“画面里也有墙、物理上也会撞墙”，再把 `go2_office_maze` 转成 MuJoCo MJCF box 障碍物。

这适合第一阶段验证 Nav2 闭环，因为 Nav2 只需要：

```text
当前位姿 + 激光障碍物距离 + 地图
```

后续如果使用 Isaac Sim，就可以把这里的 `/scan` 换成 Isaac 里的真实激光/深度相机。

### 8.4 /odom 怎么来的

当前 `/odom` 来自 MuJoCo 中 Go2 root body 的位置和速度：

```text
data.qpos[0:3] -> position
data.qpos[3:7] -> yaw
data.qvel -> velocity
```

后续接 ORB-SLAM3/DPVO 时，可以让视觉 SLAM 发布：

```text
map -> odom
```

或者直接发布视觉里程计，再与 MuJoCo/IMU 里程计融合。

## 9. 后续怎么接 ORB-SLAM3 / DPVO / VGGT

建议按这个顺序来：

### 阶段 A：当前最小闭环

```text
Nav2 + AMCL + 2D map + 模拟 scan + MuJoCo Go2 RL
```

目标：证明导航目标能驱动 Go2 走起来。

### 阶段 B：slam_toolbox 建图

```text
slam_toolbox + /scan + /odom -> /map
Nav2 使用在线地图导航
```

目标：证明建图、定位、导航能在 ROS2 框架里跑通。

### 阶段 C：视觉定位接入

```text
Isaac Sim / MuJoCo 相机图像
  -> ORB-SLAM3 / DPVO
  -> visual odom / trajectory
  -> map->odom 或 odom
  -> Nav2
```

目标：把“定位来源”从 2D AMCL 换成视觉。

### 阶段 D：具身导航大脑

```text
自然语言目标 / 视觉理解 / 可通行区域
  -> 目标点或局部路径
  -> Nav2 / MPC / PID
  -> /cmd_vel
  -> Go2 RL 底层控制
```

目标：把“我要去哪里”从手动目标点升级成大模型/视觉大脑给出的目标。

## 10. 常见问题

### 10.1 看到 `libtinfo.so.6: no version information`

这是你当前 shell 里 DPVO conda 环境污染造成的提示，不一定影响运行。脚本里已经尽量清理了：

```bash
unset LD_LIBRARY_PATH
unset PYTHONPATH
unset CONDA_PREFIX
unset CONDA_DEFAULT_ENV
```

### 10.2 看到 `getifaddrs: Operation not permitted`

如果这是 Codex 工具里跑出来的，多半是工具沙盒限制网络接口枚举导致。你在普通终端里运行通常不会阻塞。

真正判断是否跑通，看这两个现象：

1. 桥接节点日志里 `state=RL_CONTROL`。
2. 收到 `/cmd_vel` 后，日志里 `cmd=(...)` 非零，并且 `pose=(...)` 变化。

### 10.3 Nav2 不动

按顺序排查：

```bash
cd /home/ros/unitree_dev
bash scripts/check_go2_ros_graph.sh
source scripts/go2_ros_env.sh
ros2 topic echo /cmd_vel
ros2 topic echo /odom
ros2 topic echo /scan
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link base_scan
```

如果 `/cmd_vel` 有数据但 Go2 不动，检查桥接节点是否收到命令。

如果 `/cmd_vel` 没数据，检查 Nav2 是否 active、目标点是否在可通行区域、RViz 中 costmap 是否正常。

如果 Nav2 日志出现：

```text
GridBased: failed to create plan
Planning algorithm GridBased failed to generate a valid path
```

说明 planner 没有找到从当前位姿到目标点的有效路径。此时如果 MuJoCo 中 Go2 仍然动了，通常是 Nav2 的恢复行为，例如 `spin`、`backup`、`wait`，不是正常路径跟踪。

如果原始地图中目标是 free，但 Nav2 仍然反复规划失败，优先检查 costmap inflation。当前 TurtleBot3 示例地图通道不宽，`inflation_radius=0.55` 会让可通行区域变得很窄，甚至把起点和目标分到不同的可通行连通区。当前第一轮仿真已把 local/global costmap 调为：

```text
inflation_radius: 0.25
cost_scaling_factor: 5.0
```

同时把 `expected_planner_frequency` 调为 `5.0`，避免仿真和 composition 模式下频繁出现 planner rate warning。

复杂地图测试中，如果出现：

```text
GridBased: failed to create plan with tolerance 0.50
Planning algorithm GridBased failed to generate a valid path to (-4.00, -2.50)
```

说明 Nav2 的全局 costmap 认为起点到目标点不可达。当前处理：

```text
1. go2_office_maze 主测试通道已加宽，避免 costmap 膨胀后断路。
2. global_costmap 只使用 static_layer + inflation_layer。
3. local_costmap 继续使用 /scan 做局部障碍感知。
```

当前通向默认目标 `(-4.0,-2.5)` 的主通道已经拓宽到约 1.6-2.0m。本地检查确认 `(0.0,0.0) -> (-4.0,-2.5)` 在 0.70m clearance 下仍可达，这比当前 `robot_radius=0.22 + inflation_radius=0.25` 更保守。

修改地图或 Nav2 参数后必须完整重启桥接和 Nav2，`map_server` 不会自动重新读取磁盘上的 PGM：

```bash
Ctrl+C  # 关闭 bridge、Nav2、RViz、goal

cd /home/ros/unitree_dev
bash scripts/run_go2_nav_bridge.sh

cd /home/ros/unitree_dev
bash scripts/run_go2_nav2.sh

cd /home/ros/unitree_dev
bash scripts/run_go2_nav_rviz.sh

cd /home/ros/unitree_dev
bash scripts/run_go2_nav2_goal.sh
```

如果 Go2 已经接近目标，例如桥接日志显示：

```text
pose=(-2.12,-0.04) cmd=(-0.05,+0.00,+0.00)
```

说明位置基本已经接近目标，但 Nav2 可能还在纠结目标朝向或恢复行为。第一轮仿真优先验证“能到目标点附近”，当前参数已经放宽：

```text
xy_goal_tolerance: 0.35
yaw_goal_tolerance: 1.57
```

同时 DWB 最低速度和桥接层最低可执行速度已经调高，避免微小命令拖慢闭环。

当前默认复杂地图 `go2_office_maze.yaml` 里 `(0,0)` 是明确 free cell，所以桥接节点默认使用：

```text
--map-offset-x 0.0
--map-offset-y 0.0
```

`run_go2_nav2_goal.sh` 默认 AMCL 初始位姿也使用 `(0.0,0.0)`，并会在发送目标前检查起点和目标点是否位于地图 free cell。复杂地图推荐目标点命令：

```bash
cd /home/ros/unitree_dev
bash scripts/run_go2_nav2_goal.sh
```

如果切回 TurtleBot3 示例地图，因为该地图里 `(0,0)` 不是明确 free cell，需要重新设置：

```text
GO2_MAP_OFFSET_X=0.5
GO2_MAP_OFFSET_Y=-0.5
--start-x 0.5 --start-y -0.5
```

### 10.4 ROS2 图里看不到桥接节点

如果 `go2_mujoco_ros2_bridge.py` 进程存在，但 `ros2 node list` 看不到 `/go2_mujoco_ros2_bridge`，优先检查 DDS 环境，而不是改控制代码。

推荐本机仿真配置：

```bash
cd /home/ros/unitree_dev
bash scripts/check_go2_ros_graph.sh
```

输出中应包含：

```text
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ROS_LOCALHOST_ONLY=0
CYCLONEDDS_URI=...NetworkInterface name="lo"...
```

如果需要临时指定其他接口，例如 `eth0`：

```bash
cd /home/ros/unitree_dev
GO2_CYCLONEDDS_INTERFACE=eth0 bash scripts/run_go2_nav_bridge.sh
GO2_CYCLONEDDS_INTERFACE=eth0 bash scripts/check_go2_ros_graph.sh
GO2_CYCLONEDDS_INTERFACE=eth0 bash scripts/run_go2_cmd_vel_smoke_test.sh --wait 1
```

真机连接阶段应把 `GO2_CYCLONEDDS_INTERFACE` 切到真实网卡，而不是 `lo`。

### 10.5 CycloneDDS participant index 不够

如果启动 Nav2 后运行目标点脚本时报错：

```text
Failed to find a free participant index for domain 0
rmw_create_node: failed to create domain
```

说明当前 DDS domain 里已经有较多 ROS2 participants。Nav2 非 composition 模式会启动多个进程，每个进程都会占用 participant index。当前脚本已经做了两层规避：

1. `scripts/go2_ros_env.sh` 设置 `MaxAutoParticipantIndex=120`。
2. `scripts/run_go2_nav2.sh` 和 `scripts/run_go2_nav2_slam_map.sh` 默认使用 `use_composition:=True`。

修复脚本后，需要重启所有相关进程：

```bash
Ctrl+C  # 关闭桥接、Nav2、goal 脚本

cd /home/ros/unitree_dev
bash scripts/run_go2_nav_bridge.sh

cd /home/ros/unitree_dev
bash scripts/run_go2_nav2.sh

cd /home/ros/unitree_dev
bash scripts/run_go2_nav2_goal.sh
```

如果仍然不够，可以临时继续增大：

```bash
GO2_CYCLONEDDS_MAX_AUTO_INDEX=200 bash scripts/run_go2_nav2.sh
GO2_CYCLONEDDS_MAX_AUTO_INDEX=200 bash scripts/run_go2_nav2_goal.sh
```

### 10.6 Go2 走得太猛或容易不稳

先改桥接层限幅：

```bash
bash scripts/run_go2_nav_bridge.sh \
  --max-vx 0.25 \
  --max-vy 0.15 \
  --max-yaw 0.5
```

再改 Nav2 参数：

```text
projects/go2_nav_sim/config/nav2_go2_params.yaml
```

重点看：

```text
max_vel_x
max_vel_y
max_vel_theta
max_velocity
acc_lim_x
acc_lim_y
acc_lim_theta
```

## 11. 当前验证状态

已经验证：

1. `.venv-unitree` 在 source ROS2 后可以同时 import `rclpy`、`tf2_ros`、`mujoco`、`torch`、`yaml`。
2. `go2_mujoco_ros2_bridge.py` 语法检查通过。
3. 桥接节点可以启动 Go2 MuJoCo RL runner。
4. Go2 可以从 `PD_STAND_UP` 进入 `STABILIZE`，再进入 `RL_CONTROL`。
5. `--duration` 可以让 headless 桥接测试自动退出。
6. 使用 `rmw_cyclonedds_cpp + GO2_CYCLONEDDS_INTERFACE=lo` 后，ROS2 图可以发现 `/go2_mujoco_ros2_bridge`。
7. `/cmd_vel` 已确认存在桥接节点订阅关系，`/odom` 和 `/scan` 已确认存在桥接节点发布关系。

当前 Codex 工具沙盒中跨进程 ROS2 通信受到网络权限限制，真实判断以你在普通终端中运行 `check_go2_ros_graph.sh`、桥接节点日志和 MuJoCo 画面为准。

下一步按顺序执行：

```text
1. /cmd_vel 冒烟测试：run_go2_cmd_vel_smoke_test.sh --wait 1
2. Nav2 + go2_office_maze 复杂地图目标点导航：run_go2_nav2.sh + run_go2_nav2_goal.sh
3. slam_toolbox 在线建图：run_go2_slam_toolbox.sh + run_go2_slam_mapping_drive.sh
4. 保存地图并用自建地图重新跑 Nav2：save_go2_slam_map.sh + run_go2_nav2_slam_map.sh
```
