# Go2 高层控制与 SLAM / 视觉导航路线

这份文档是新的主线。

你现在不再把重点放在底层步态、关节 PD、RL policy、LowCmd 上，而是改成：

```text
SLAM / 视觉定位 / 地图 / 路径规划 / Nav2
  -> 输出安全限幅后的 /cmd_vel
  -> 交给宇树官方高层 Sport 控制器
  -> Go2 自己完成稳定行走、平衡和步态控制
```

这条路线更适合你的目标：

```text
先完成仿真和导航闭环
再接真实 Go2
最后把 ORB-SLAM3、DPVO、VGGT 等视觉定位效果搭载到机器狗上
```

## 1. 为什么现在切换到高层控制

你已经验证过：

```text
1. 低层 PD 站立可以做。
2. 手写步态很容易因为接触、相位、重心、足端轨迹不完整而不稳定。
3. Isaac policy 到 MuJoCo 还存在 sim-to-sim gap。
```

这说明继续纠结底层运动控制，会消耗大量时间，而且不直接服务于 SLAM/导航主线。

Go2 本身已经有稳定的高层运动控制接口，例如：

```python
sport_client.StandUp()
sport_client.BalanceStand()
sport_client.Move(vx, vy, yaw_rate)
sport_client.StopMove()
sport_client.StandDown()
```

本地参考文件：

```text
/home/ros/unitree_dev/src/unitree_sdk2_python/example/go2/high_level/go2_sport_client.py
/home/ros/unitree_dev/src/unitree_sdk2_python/unitree_sdk2py/go2/sport/sport_client.py
```

所以新的分工应该是：

| 模块 | 负责什么 |
|---|---|
| 宇树高层 Sport 控制器 | 稳定站立、行走、转向、避障模式、急停 |
| ROS2 / Nav2 | 地图、定位、规划、输出 `/cmd_vel` |
| ORB-SLAM3 / DPVO / VGGT | 视觉定位、轨迹估计、关键帧/地图 |
| go2_ros2_sdk | Go2 相机、雷达、里程计、IMU、点云、ROS2 topic、WebRTC 高层控制 |
| 你写的导航逻辑 | 把定位和目标点变成安全速度命令 |

## 2. 新系统架构

最终闭环是：

```text
Go2 相机 / 雷达 / IMU / 里程计
  -> ROS2 topics
  -> SLAM / 视觉定位
  -> map / odom / tf / pose
  -> Nav2 或自写路径跟踪器
  -> /cmd_vel
  -> twist_mux
  -> /cmd_vel_out
  -> go2_driver_node
  -> WebRTC / Sport Move
  -> Go2 官方高层控制器
  -> 真实机器狗稳定运动
```

对应本地代码：

```text
/home/ros/unitree_dev/projects/go2_ros2_sdk_ws
  ROS2 Go2 SDK 工作区，负责实机桥接、相机、雷达、SLAM、Nav2。

/home/ros/ros2_orbslam3
  ORB-SLAM3 / DPVO / VGGT 视觉定位工作区。

/home/ros/unitree_dev/src/unitree_sdk2_python
  宇树官方 Python SDK2，高层 SportClient 示例。
```

## 3. 一个重要现实：官方高层控制不能完整本地 MuJoCo 仿真

必须明确：

```text
宇树官方高层 Sport 控制器实际运行在 Go2 机器狗本体上。
```

当前本地 `unitree_mujoco` Python 仿真主要支持：

```text
rt/lowcmd
rt/lowstate
rt/sportmodestate
```

它不是完整的 Go2 官方高层控制器仿真器。也就是说：

```text
SportClient.Move() 这种官方高层稳定运动能力，
不能在本地 MuJoCo 里 1:1 验证。
```

所以“先仿真，再上实机”的正确理解是：

```text
仿真阶段验证：
  1. SLAM / 视觉定位能输出 pose / odom / path。
  2. Nav2 或自写导航器能输出合理 /cmd_vel。
  3. /cmd_vel 已经限幅、平滑、可急停。
  4. RViz 能看到地图、机器人位姿、目标点、路径。

实机阶段验证：
  1. Go2 官方高层控制器执行 /cmd_vel。
  2. 低速、小范围、可急停。
  3. 从手动遥控到半自主，再到自主导航。
```

你不需要在 MuJoCo 里复刻宇树高层步态控制器。

## 4. go2_ros2_sdk 的高层控制链路

本地 launch：

```text
/home/ros/unitree_dev/projects/go2_ros2_sdk_ws/src/go2_robot_sdk/launch/robot.launch.py
```

关键 topic 链路：

```text
/cmd_vel       Nav2 或你的导航程序输出
/cmd_vel_joy   手柄或键盘遥控输出
  -> twist_mux
/cmd_vel_out   合成后的最终速度命令
  -> go2_driver_node
  -> WebRTC sport request
  -> Go2 高层 Move
```

`twist_mux.yaml` 里当前优先级：

```text
joy:
  topic: cmd_vel_joy
  priority: 10

navigation:
  topic: cmd_vel
  priority: 5
```

这意味着：

```text
手柄优先级高于导航。
```

这是对的：真实机器狗上，人工遥控必须能覆盖自主导航。

我已经补了一个安全修正：

```text
文件：
/home/ros/unitree_dev/projects/open source/go2_ros2_sdk-master/go2_ros2_sdk-master/go2_robot_sdk/go2_robot_sdk/application/services/robot_control_service.py

修正：
当 /cmd_vel_out 是零速度时，不再忽略，而是显式发送 StopMove。
```

原因：

```text
导航停止、急停、twist_mux 超时都会产生零速度。
零速度必须让 Go2 停住。
```

## 5. 阶段 0：只验证高层控制接口

先不要跑 SLAM，不要跑 Nav2，只验证 Go2 高层控制能否工作。

### 5.1 Python SDK2 高层测试

真实 Go2 连接好后，用官方示例测试：

```bash
cd /home/ros/unitree_dev

env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  .venv-unitree/bin/python src/unitree_sdk2_python/example/go2/high_level/go2_sport_client.py <网卡名>
```

网卡名通常可能是：

```text
eth0
enp...
```

在 WSL 里先用：

```bash
ip addr
```

确认和 Go2 网线通信的网卡。

推荐只测试这些安全动作：

```text
stand_up
balanced stand
move forward
move rotate
stop_move
stand_down
recovery
```

不要测试：

```text
flip
jump
handstand
cross step
walk upright
```

### 5.2 ROS2 高层控制测试

启动 go2_ros2_sdk：

```bash
cd /home/ros/unitree_dev/projects/go2_ros2_sdk_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

export ROBOT_IP=<Go2_IP>
export CONN_TYPE=webrtc

ros2 launch go2_robot_sdk robot.launch.py \
  rviz2:=false \
  nav2:=false \
  slam:=false \
  foxglove:=false \
  joystick:=false \
  teleop:=true
```

另开终端发一个很小的速度：

```bash
source /opt/ros/humble/setup.bash

ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

停止：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

这一阶段验收：

```text
1. Go2 能站立。
2. 0.10m/s 低速前进能执行。
3. /cmd_vel=0 能停住。
4. 遥控器或手机 App 能随时接管。
```

## 6. 阶段 1：仿真 / 离线验证 SLAM 和视觉定位

这一阶段不需要机器狗真的动。

目标：

```text
把视觉定位和 SLAM 结果跑通，输出 ROS2 标准话题。
```

你现有视觉定位工作区：

```text
/home/ros/ros2_orbslam3
```

你要优先验证：

```text
/camera/image_raw
/slam/pose
/slam/odom
/slam/path
/tf 或 /tf_static
RViz 轨迹显示
```

建议顺序：

```text
1. 用本地视频跑 ORB-SLAM3。
2. 用本地视频跑 DPVO。
3. 把两者输出统一成 nav_msgs/Odometry 或 geometry_msgs/PoseStamped。
4. 在 RViz 中显示轨迹。
5. 先不要接控制，只看定位结果是否稳定。
```

## 7. 阶段 2：仿真 / 干跑 Nav2 输出速度

这一阶段仍然不让真实 Go2 动。

目标：

```text
让 Nav2 或自写路径跟踪器输出合理 /cmd_vel。
```

验证内容：

```text
1. 给定目标点。
2. 规划出路径。
3. 输出 /cmd_vel。
4. /cmd_vel 不超过安全限幅。
5. 目标到达后输出 0。
```

建议先写一个安全限幅原则：

```text
vx:  -0.15 ~ 0.25 m/s
vy:  -0.10 ~ 0.10 m/s
yaw: -0.30 ~ 0.30 rad/s
```

实机初期建议更保守：

```text
vx <= 0.10 m/s
yaw <= 0.20 rad/s
```

## 8. 阶段 3：实机只建图，不自主运动

连接真实 Go2，但不要让它自主导航。

启动 go2_ros2_sdk，打开：

```text
camera
odom
imu
point_cloud2
scan
rviz
slam_toolbox
```

机器人运动方式：

```text
手柄或手机 App 慢慢遥控
```

目标：

```text
1. RViz 里能看到 Go2 模型。
2. 能看到点云 / scan。
3. 能看到 odom。
4. slam_toolbox 能建图。
5. ORB-SLAM3 / DPVO 能处理相机图像。
```

这一阶段不要让 Nav2 控制机器狗。

## 9. 阶段 4：实机低速闭环导航

当地图、定位、传感器都稳定后，再接控制：

```text
Nav2 / 自写导航器
  -> /cmd_vel
  -> twist_mux
  -> /cmd_vel_out
  -> go2_driver_node
  -> Go2 高层 Move
```

启动时建议：

```bash
ros2 launch go2_robot_sdk robot.launch.py \
  rviz2:=true \
  slam:=true \
  nav2:=true \
  foxglove:=true \
  joystick:=true \
  teleop:=true
```

安全要求：

```text
1. 人站在旁边，手柄/手机 App 可随时接管。
2. 第一轮只给 1 米以内目标点。
3. 速度限制到 vx <= 0.10m/s。
4. 目标到达必须发布 /cmd_vel=0。
5. 任何异常先发 /cmd_vel=0，再手动接管。
```

## 10. 你接下来最该做的 5 件事

第一件：确认高层控制接口。

```text
能 StandUp / Move / StopMove / StandDown。
```

第二件：确认 go2_ros2_sdk topic。

```text
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /camera/image_raw
ros2 topic echo /point_cloud2
ros2 topic echo /cmd_vel_out
```

第三件：用真实相机或本地视频跑 ORB-SLAM3 / DPVO。

```text
先看轨迹，不接控制。
```

第四件：让 Nav2 或自写路径跟踪器只输出 `/cmd_vel`。

```text
先 echo 看，不发给机器人。
```

第五件：低速实机闭环。

```text
只在空旷环境，速度极低，目标点很近。
```

## 11. 当前路线一句话总结

不要再让自己陷在“怎么写一个稳定四足步态控制器”里。

你现在的项目主线应该是：

```text
Go2 官方高层控制负责走路。
你负责让它知道自己在哪里、要去哪里、下一秒应该给多大的 /cmd_vel。
```

这就是更适合 SLAM、视觉导航、具身导航项目的工程路线。
