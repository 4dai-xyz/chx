# Go2 ROS2 SDK 从 0 到 SLAM 导航学习路线

本文用于跟进你新下载的 `go2_ros2_sdk-master` 路线。它和前面的 MuJoCo/Isaac 路线不是互相替代，而是各自负责不同层次：

```text
MuJoCo：底层关节、接触、动力学、运动控制学习
Isaac Lab：高真实感仿真、强化学习策略、传感器/场景验证
go2_ros2_sdk：ROS2 实机桥接、RViz、SLAM、Nav2、相机/雷达/目标检测
```

## 1. 当前源码位置

你下载的原始源码在：

```text
/home/ros/unitree_dev/projects/open source/go2_ros2_sdk-master/go2_ros2_sdk-master
```

这个路径里有空格，长期敲命令不方便。所以我已经新建了一个干净的 ROS2 workspace：

```text
/home/ros/unitree_dev/projects/go2_ros2_sdk_ws
```

其中：

```text
/home/ros/unitree_dev/projects/go2_ros2_sdk_ws/src
```

里面用软链接指向原始源码包：

```text
coco_detector
go2_interfaces
go2_robot_sdk
lidar_processor
lidar_processor_cpp
speech_processor
```

这样做的好处是：

```text
原始下载目录不动
后续 colcon 命令路径简短
出问题时容易重建 workspace
```

## 2. 这套 go2_ros2_sdk 的核心功能

根据本地 README 和源码，这套 SDK 主要提供：

```text
1. Go2 URDF / RViz 机器人模型显示
2. WebRTC Wi-Fi 连接真实 Go2
3. CycloneDDS 网线连接真实 Go2
4. joint_states / IMU / 里程计 / 状态信息发布
5. 前置相机图像发布
6. 雷达点云发布
7. PointCloud2 转 LaserScan
8. 手柄 teleop_twist_joy 控制
9. twist_mux 合并手柄和导航速度
10. slam_toolbox 建图
11. Nav2 自主导航
12. Foxglove 可视化
13. COCO 目标检测
```

注意：这条路线主要面向“真实 Go2 + ROS2 应用层”，不是 MuJoCo 那种底层物理仿真。

## 3. 当前已经完成的本地验证

### 3.1 ROS2 Humble 可用

每个新终端都要先执行：

```bash
source /opt/ros/humble/setup.bash
```

当前系统检测到：

```text
ROS_DISTRO=humble
Python 3.10.12
```

如果直接运行 `ros2` 提示找不到，通常只是因为没有 source。

### 3.2 已创建 workspace

工作区路径：

```bash
cd /home/ros/unitree_dev/projects/go2_ros2_sdk_ws
```

### 3.3 已成功编译的包

已经成功编译：

```text
go2_interfaces
go2_robot_sdk
lidar_processor
speech_processor
coco_detector
```

其中 `go2_interfaces` 是最基础的消息接口包。已经验证可以查看消息：

```bash
source /opt/ros/humble/setup.bash
source /home/ros/unitree_dev/projects/go2_ros2_sdk_ws/install/setup.bash
ros2 interface show go2_interfaces/msg/WebRtcReq
ros2 interface show go2_interfaces/msg/Go2State
```

### 3.4 暂时未成功的包

`lidar_processor_cpp` 暂时未成功，原因是系统缺少：

```text
pcl_ros
```

报错核心是：

```text
Could not find a package configuration file provided by "pcl_ros"
```

需要安装：

```bash
sudo apt install ros-humble-pcl-ros
```

但当前 `ros` 用户不在 sudo 组，所以我不能直接替你安装。需要你用有管理员权限的方式安装，或者把当前用户加入 sudoers 后再装。

## 4. 当前依赖状态

### 4.1 已有的 ROS2 包

当前已检测到：

```text
joy
teleop_twist_joy
```

### 4.2 缺少的 ROS2 系统依赖

完整运行 `robot.launch.py`、SLAM、Nav2、点云转激光和 Foxglove 需要补齐：

```text
twist_mux
nav2_bringup
nav2_amcl
nav2_map_server
slam_toolbox
pointcloud_to_laserscan
foxglove_bridge
pcl_ros
```

建议安装命令：

```bash
sudo apt update
sudo apt install \
  ros-humble-twist-mux \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-amcl \
  ros-humble-nav2-map-server \
  ros-humble-slam-toolbox \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-foxglove-bridge \
  ros-humble-pcl-ros \
  ros-humble-image-transport \
  ros-humble-compressed-image-transport \
  ros-humble-vision-msgs \
  ros-humble-image-tools \
  portaudio19-dev \
  clang
```

### 4.3 缺少的 Python 依赖

当前缺少：

```text
aiortc
aiohttp
open3d
pycryptodome
pydub
```

仓库要求在原始源码根目录执行：

```bash
cd "/home/ros/unitree_dev/projects/open source/go2_ros2_sdk-master/go2_ros2_sdk-master"
python3 -m pip install -r requirements.txt
```

说明：

```text
aiortc / aiohttp：WebRTC 连接真实 Go2 所需
open3d：点云处理/地图保存所需
pycryptodome：加密通信相关
pydub：音频处理相关
torch / torchvision：COCO 目标检测相关
```

## 5. 推荐学习顺序：从简单到复杂

### 第 0 阶段：只检查环境

运行：

```bash
cd /home/ros/unitree_dev
bash scripts/go2_ros2_sdk_check.sh
```

它只做检查，不安装、不连接机器狗、不发送控制命令。

你要看懂输出中的 3 类信息：

```text
本项目包是否已经编译
ROS2 系统依赖是否安装
Python 运行依赖是否可导入
```

### 第 1 阶段：学习 ROS2 消息接口

目标：

```text
理解 Go2 ROS2 SDK 自定义了哪些消息
知道 WebRtcReq / Go2State / LowState / SportModeState 分别是什么
```

命令：

```bash
source /opt/ros/humble/setup.bash
source /home/ros/unitree_dev/projects/go2_ros2_sdk_ws/install/setup.bash

ros2 interface show go2_interfaces/msg/WebRtcReq
ros2 interface show go2_interfaces/msg/Go2State
ros2 interface show go2_interfaces/msg/SportModeState
ros2 interface show go2_interfaces/msg/LowState
```

重点理解：

```text
WebRtcReq：直接给 Go2 WebRTC API 发命令
Go2State：Go2 高层状态
SportModeState：运动状态
LowState：底层电机/IMU 等状态
```

### 第 2 阶段：学习 URDF 和 RViz 模型

目标：

```text
先不连接真实机器狗，只看 Go2 模型、TF 和 joint_states 的基本关系
```

核心文件：

```text
go2_robot_sdk/urdf/go2.urdf
go2_robot_sdk/launch/robot.launch.py
go2_robot_sdk/config/single_robot_conf.rviz
```

后续可以单独写一个最小 launch，只启动：

```text
robot_state_publisher
rviz2
```

这样你能先看懂：

```text
base_link
各条腿的 link / joint
雷达 frame
相机 frame
TF 树
```

### 第 3 阶段：理解速度控制链路

这套 SDK 的运动控制主线是：

```text
手柄 / Nav2 / 你的程序
  -> cmd_vel 或 cmd_vel_joy
  -> twist_mux
  -> cmd_vel_out
  -> go2_driver_node
  -> WebRTC Move 命令 api_id=1008
  -> Go2 自带高层运动控制器
```

关键源码：

```text
go2_robot_sdk/config/twist_mux.yaml
go2_robot_sdk/go2_robot_sdk/presentation/go2_driver_node.py
go2_robot_sdk/go2_robot_sdk/application/services/robot_control_service.py
go2_robot_sdk/go2_robot_sdk/application/utils/command_generator.py
```

`cmd_vel_out` 回调在 `go2_driver_node.py`：

```text
Twist.linear.x  -> 前后速度
Twist.linear.y  -> 左右速度
Twist.angular.z -> 原地转向速度
```

WebRTC Move 命令在 `command_generator.py` 中生成：

```text
api_id = 1008
topic = rt/api/sport/request
parameter = {"x": vx, "y": vy, "z": yaw_rate}
```

### 第 4 阶段：连接真实 Go2，只看数据

先不要运动，只验证连接和传感器数据。

Wi-Fi WebRTC 方式：

```bash
source /opt/ros/humble/setup.bash
source /home/ros/unitree_dev/projects/go2_ros2_sdk_ws/install/setup.bash

export ROBOT_IP="你的Go2 IP"
export CONN_TYPE="webrtc"
ros2 launch go2_robot_sdk robot.launch.py joystick:=false teleop:=false slam:=false nav2:=false foxglove:=false
```

然后另开终端：

```bash
source /opt/ros/humble/setup.bash
source /home/ros/unitree_dev/projects/go2_ros2_sdk_ws/install/setup.bash
ros2 topic list
ros2 topic echo /go2_states
ros2 topic echo /imu
```

安全要求：

```text
手机 App 先断开 WebRTC 连接
不要发送 cmd_vel
不要启动 joystick / teleop
机器狗放在安全空间
遥控器和急停准备好
```

### 第 5 阶段：低速手动控制

完整依赖安装好后，再启用手柄/teleop。

启动：

```bash
source /opt/ros/humble/setup.bash
source /home/ros/unitree_dev/projects/go2_ros2_sdk_ws/install/setup.bash

export ROBOT_IP="你的Go2 IP"
export CONN_TYPE="webrtc"
ros2 launch go2_robot_sdk robot.launch.py slam:=false nav2:=false foxglove:=false
```

注意：

```text
twist_mux 订阅 cmd_vel_joy 和 cmd_vel
go2_driver_node 订阅 cmd_vel_out
真实 Go2 最终收到的是 WebRTC Move 命令
```

建议把速度先改小：

```text
go2_robot_sdk/config/twist_mux.yaml
```

当前配置里手柄速度是：

```yaml
scale_linear:
  x: 0.5
  y: 0.5
scale_angular:
  yaw: 1.0
```

初学建议降到：

```yaml
scale_linear:
  x: 0.15
  y: 0.10
scale_angular:
  yaw: 0.35
```

### 第 6 阶段：SLAM 建图

依赖：

```text
slam_toolbox
pointcloud_to_laserscan
lidar_processor_cpp 或 lidar_processor
```

启动建图模式：

```bash
source /opt/ros/humble/setup.bash
source /home/ros/unitree_dev/projects/go2_ros2_sdk_ws/install/setup.bash

export ROBOT_IP="你的Go2 IP"
export CONN_TYPE="webrtc"
export MAP_NAME="my_first_go2_map"
ros2 launch go2_robot_sdk mapping.launch.py
```

在 RViz 里观察：

```text
RobotModel
PointCloud2
LaserScan
Map
Odometry
TF
```

学习重点：

```text
点云如何变成 LaserScan
LaserScan 如何输入 slam_toolbox
slam_toolbox 如何发布 /map
地图保存时生成 yaml / pgm / posegraph / data
```

### 第 7 阶段：Nav2 自主导航

前提：

```text
已经有一张可用地图
Go2 起始位姿和地图对齐
低速控制已经验证
```

启动：

```bash
source /opt/ros/humble/setup.bash
source /home/ros/unitree_dev/projects/go2_ros2_sdk_ws/install/setup.bash

export ROBOT_IP="你的Go2 IP"
export CONN_TYPE="webrtc"
export MAP_FILE="/home/ros/unitree_dev/projects/go2_ros2_sdk_ws/my_first_go2_map.yaml"
ros2 launch go2_robot_sdk navigation.launch.py map:=$MAP_FILE
```

在 RViz 中：

```text
先确认定位正确
再用 Nav2 Goal 给一个很近的目标点
跟着机器狗旁边观察
随时准备急停
```

常见问题：

```text
地图不准 -> 规划路线穿墙
初始位姿不准 -> 机器人认为自己在错误位置
雷达频率低 -> costmap 更新慢
控制频率太高 -> 真实机器人可能乱转
```

### 第 8 阶段：目标检测和具身导航

启动 driver 后，前置相机 topic 通常是：

```text
/camera/image_raw
```

运行 COCO 检测：

```bash
source /opt/ros/humble/setup.bash
source /home/ros/unitree_dev/projects/go2_ros2_sdk_ws/install/setup.bash
ros2 run coco_detector coco_detector_node
```

查看检测结果：

```bash
ros2 topic echo /detected_objects
```

这一步可以继续扩展成：

```text
检测 person
根据 bbox 中心判断目标在左/中/右
生成 cmd_vel
通过 twist_mux -> cmd_vel_out -> Go2 Move
实现简单跟随
```

## 6. 当前发现的源码/配置注意点

### 6.1 不要使用 --symlink-install 编译 go2_robot_sdk

当前环境下：

```bash
colcon build --packages-select go2_robot_sdk --symlink-install
```

会失败：

```text
error: option --editable not recognized
```

已经验证普通构建可以成功：

```bash
colcon build --packages-select go2_robot_sdk
```

所以这套 workspace 暂时建议不用 `--symlink-install`。

### 6.2 launch 里疑似有 cyclonedds 拼写判断问题

源码里部分判断写成了：

```text
cyclonedx
```

但真实连接类型应该是：

```text
cyclonedds
```

这可能导致网线模式被误判成 multi robot。等你要走网线 CycloneDDS 路线时，建议先统一修正这个拼写。

### 6.3 WebRTC 路线需要手机 App 断开

README 提醒：

```text
If you are using WebRTC (Wi-Fi) protocol, close the connection with a mobile app before connecting to the robot.
```

也就是：

```text
用 WebRTC 连接真实 Go2 前，手机 App 先断开。
```

### 6.4 这条路线会直接控制真实 Go2

只要 `go2_driver_node` 成功连接机器狗，并且收到 `cmd_vel_out`，它会发 WebRTC Move 命令。

所以实机运动前必须满足：

```text
速度限幅足够小
有遥控器或急停
机器狗周围空旷
先不用 Nav2，先手动低速
先发短时速度，再发 0 停止
```

## 7. 建议你下一步做什么

当前最合理的顺序：

```text
1. 读本文第 1 到第 5 节
2. 跑 scripts/go2_ros2_sdk_check.sh
3. 学 go2_interfaces 的消息结构
4. 学 robot.launch.py 启动了哪些节点
5. 补齐 apt / pip 依赖
6. 只连接 Go2 看 topic，不运动
7. 低速手动控制
8. SLAM 建图
9. Nav2 导航
10. COCO 检测和具身导航
```

如果你要把它和你现在的 MuJoCo demo 接起来，统一抽象应该是：

```text
路径点控制器 / 键盘控制器 / Nav2
  -> cmd_vel
  -> 后端 A：MuJoCo 教学可视化
  -> 后端 B：Isaac Lab policy
  -> 后端 C：go2_ros2_sdk 实机 WebRTC Move
```

这样你写一次上层导航逻辑，以后可以切换不同执行后端。
