#!/usr/bin/env python3
"""
Go2 MuJoCo RL 底层控制器 <-> ROS2 导航栈桥接节点。

这个脚本的目标不是替代完整的 Isaac/Gazebo 传感器仿真，而是先把最关键的闭环跑通：

    Nav2 / SLAM 大脑 -> /cmd_vel -> Go2 RL policy + MuJoCo -> /odom, /scan, /tf -> Nav2 / SLAM

只要这条链路跑通，后面就可以逐步把这里的二维激光和里程计替换成 Isaac Sim 相机、
ORB-SLAM3/DPVO 位姿、真实 Go2 里程计等更复杂模块。

运行前需要 source ROS2，并使用 .venv-unitree 的 Python：

    cd /home/ros/unitree_dev
    source /opt/ros/humble/setup.bash
    env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
      .venv-unitree/bin/python projects/go2_nav_sim/go2_mujoco_ros2_bridge.py
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


PROJECT_ROOT = Path(__file__).resolve().parents[2]  # 项目根目录：/home/ros/unitree_dev。
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))  # 让脚本能直接 import projects/go2_control_demos 中的 runner。

from projects.go2_control_demos.go2_mujoco_rl_policy_runner import (  # noqa: E402
    GO2_SCENE_XML,
    RunnerConfig,
    RunnerState,
    Go2MujocoRlPolicyRunner,
)


DEFAULT_MAP_YAML = PROJECT_ROOT / "projects/go2_nav_sim/maps/go2_office_maze.yaml"  # 复杂办公室/走廊测试地图。


@dataclass
class MapData:
    """二维栅格地图数据，用来发布 /map 并从地图中模拟二维激光。"""

    width: int  # 地图宽度，单位：像素。
    height: int  # 地图高度，单位：像素。
    resolution: float  # 每个像素代表多少米。
    origin_x: float  # 地图左下角在 map 坐标系下的 x。
    origin_y: float  # 地图左下角在 map 坐标系下的 y。
    occupancy: np.ndarray  # OccupancyGrid 使用的 [-1, 0, 100] 栅格数据，shape=(height,width)。
    occupied_mask: np.ndarray  # 用于 ray cast 的障碍物布尔图，True 表示障碍物。


def yaw_to_quat(yaw: float) -> Quaternion:
    """把二维 yaw 角转换成 ROS 四元数。"""
    q = Quaternion()  # ROS 消息对象。
    q.z = math.sin(yaw * 0.5)  # 只绕 z 轴旋转。
    q.w = math.cos(yaw * 0.5)  # 单位四元数的 w。
    return q  # 返回 geometry_msgs/Quaternion。


def quat_wxyz_to_yaw(q_wxyz: np.ndarray) -> float:
    """从 MuJoCo free joint 的 [w,x,y,z] 四元数提取 yaw。"""
    w, x, y, z = q_wxyz  # MuJoCo 根节点四元数顺序。
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))  # ZYX yaw。


def normalize_angle(angle: float) -> float:
    """把角度归一化到 [-pi, pi]，避免导航中 yaw 无限累加。"""
    return math.atan2(math.sin(angle), math.cos(angle))  # atan2(sin,cos) 是稳定的角度归一化方式。


def ros_time_from_seconds(seconds: float) -> Time:
    """把仿真时间秒数转换成 ROS Time 消息。"""
    msg = Time()  # 创建 builtin_interfaces/Time。
    msg.sec = int(seconds)  # 整秒部分。
    msg.nanosec = int((seconds - msg.sec) * 1_000_000_000)  # 纳秒部分。
    return msg  # 返回 ROS 时间。


def shape_command_axis(value: float, deadband: float, min_abs: float) -> float:
    """把太小的速度命令变成 0 或提升到最低可执行速度。"""
    if abs(value) < deadband:
        return 0.0
    if abs(value) < min_abs:
        return math.copysign(min_abs, value)
    return value


def load_map(map_yaml: Path) -> MapData:
    """读取 Nav2 map_server 同款 yaml+pgm 地图。"""
    with map_yaml.open("r", encoding="utf-8") as f:
        meta = yaml.safe_load(f)  # 读取 yaml 元数据。

    image_path = Path(meta["image"])  # 地图图片路径可能是相对路径。
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path  # 相对路径按 yaml 所在目录解析。

    with image_path.open("rb") as f:
        magic = f.readline().strip()  # PGM 文件头，Nav2 示例地图是 P5。
        if magic != b"P5":
            raise ValueError(f"当前只支持 P5 PGM 地图，实际 magic={magic!r}")  # 简化实现，避免误读。

        header_tokens: list[bytes] = []  # 保存 width/height/maxval。
        while len(header_tokens) < 3:
            line = f.readline()  # 逐行读取头部。
            if line.startswith(b"#"):
                continue  # 跳过 PGM 注释。
            header_tokens.extend(line.split())  # 拆出数字 token。
        width, height, maxval = map(int, header_tokens[:3])  # 解析尺寸和最大灰度。
        if maxval <= 0:
            raise ValueError("PGM maxval 非法")  # 防御异常地图。
        image = np.frombuffer(f.read(width * height), dtype=np.uint8).reshape((height, width))  # PGM 像素，第一行是图像顶部。

    negate = int(meta.get("negate", 0))  # Nav2 map yaml 的反色标志。
    occupied_thresh = float(meta.get("occupied_thresh", 0.65))  # 占据阈值。
    free_thresh = float(meta.get("free_thresh", 0.196))  # 空闲阈值。
    resolution = float(meta["resolution"])  # 地图分辨率。
    origin_x, origin_y, _ = [float(v) for v in meta["origin"]]  # 地图左下角原点。

    if negate:
        occ_prob = image.astype(np.float32) / float(maxval)  # negate=1 时亮色表示占据。
    else:
        occ_prob = (float(maxval) - image.astype(np.float32)) / float(maxval)  # negate=0 时黑色表示占据。

    occupancy = np.full((height, width), -1, dtype=np.int8)  # 默认 unknown。
    occupancy[occ_prob <= free_thresh] = 0  # 低占据概率表示 free。
    occupancy[occ_prob >= occupied_thresh] = 100  # 高占据概率表示 occupied。
    occupied_mask = occupancy >= 100  # ray cast 只把明确障碍物当作命中。

    return MapData(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        occupancy=occupancy,
        occupied_mask=occupied_mask,
    )  # 返回地图数据。


class Go2MujocoRos2Bridge(Node):
    """把 MuJoCo 中的 Go2 RL 控制器包装成 ROS2 中的移动机器人。"""

    def __init__(self, args: argparse.Namespace):
        super().__init__("go2_mujoco_ros2_bridge")  # ROS2 节点名。
        self.args = args  # 保存命令行参数。
        self.map_data = load_map(args.map_yaml)  # 加载 2D 地图，用于发布 /map 和模拟 /scan。
        self.runner = Go2MujocoRlPolicyRunner(self._make_runner_config(args))  # 创建已经验证过的 Go2 RL runner。
        self.runner.reset()  # 重置 MuJoCo 仿真和控制状态机。

        self.cmd_vel = np.zeros(3, dtype=np.float32)  # 当前导航速度命令：[vx, vy, yaw_rate]。
        self.last_cmd_wall_time = time.monotonic()  # 最近一次收到 /cmd_vel 的真实时间，用于超时保护。
        self.sim_time = 0.0  # 仿真时间，从 0 开始。
        self.last_status_time = 0.0  # 上次打印状态的仿真时间。
        self.should_stop = False  # 到达 --duration 或外部关闭时置 True，让主循环退出。

        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)  # 订阅 Nav2 输出速度。
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)  # 发布里程计。
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)  # 发布二维激光。
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 1) if args.publish_map else None  # 可选发布地图，Nav2 map_server 运行时不重复发布。
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)  # 发布仿真时间；Nav2 use_sim_time=true 时需要。
        self.tf_broadcaster = TransformBroadcaster(self)  # 发布动态 TF。
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)  # 发布静态 TF。

        self._publish_static_tf()  # base_link -> base_scan 静态关系。
        if self.map_pub is not None:
            self._publish_map(self.sim_time)  # 需要时启动先发一次地图。

        self.timer = self.create_timer(self.runner.dt, self._timer_step)  # 用 MuJoCo timestep 驱动仿真循环。
        self.get_logger().info(
            "Go2 MuJoCo ROS2 bridge started: /cmd_vel -> RL runner, publishing /odom /scan /tf /map."
        )  # 启动日志。

    def _make_runner_config(self, args: argparse.Namespace) -> RunnerConfig:
        """根据命令行参数生成底层 RL runner 配置。"""
        return RunnerConfig(
            scene_xml=args.scene_xml,  # MuJoCo 场景。
            sim_duration=args.duration,  # viewer 模式最长运行时间；本桥接主要用 timer，不主动按这个退出。
            headless_duration=args.duration,  # 复用参数，便于自检。
            stand_time=args.stand_time,  # PD 缓起时间。
            stabilize_time=args.stabilize_time,  # 接管前稳定时间。
            policy_dt=args.policy_dt,  # policy 推理周期。
            kp=args.kp,  # RL 控制阶段 kp。
            kd=args.kd,  # RL 控制阶段 kd。
            stand_kp=args.stand_kp,  # 站立阶段 kp。
            stand_kd=args.stand_kd,  # 站立阶段 kd。
            action_clip=args.action_clip,  # action 限幅。
            foot_force_binary=args.foot_force_binary,  # 足端力二值化选项。
            enable_keyboard_cmd=False,  # 桥接模式只接受 /cmd_vel，不启用键盘。
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        """收到 Nav2 或测试脚本发来的速度命令。"""
        vx = float(np.clip(msg.linear.x, -self.args.max_vx, self.args.max_vx))  # 前后速度限幅。
        vy = float(np.clip(msg.linear.y, -self.args.max_vy, self.args.max_vy))  # 横向速度限幅；Nav2 默认通常为 0。
        yaw = float(np.clip(msg.angular.z, -self.args.max_yaw, self.args.max_yaw))  # 角速度限幅。
        vx = shape_command_axis(vx, self.args.cmd_deadband_vx, self.args.min_exec_vx)  # 避免 RL policy 对微小 vx 无响应。
        vy = shape_command_axis(vy, self.args.cmd_deadband_vy, self.args.min_exec_vy)  # 避免微小横移命令拖住 Nav2。
        yaw = shape_command_axis(yaw, self.args.cmd_deadband_yaw, self.args.min_exec_yaw)  # 避免微小角速度原地磨。
        self.cmd_vel[:] = [vx, vy, yaw]  # 保存给下一轮 MuJoCo 控制使用。
        self.last_cmd_wall_time = time.monotonic()  # 更新命令时间戳。

    def _timer_step(self) -> None:
        """ROS2 定时器回调：推进一次 MuJoCo 仿真并发布 ROS2 话题。"""
        if self.args.duration > 0.0 and self.sim_time >= self.args.duration:
            self._zero_cmd()  # 到达设定时长后停住。
            self.should_stop = True  # 通知主循环退出，避免自检进程一直挂着。
            return  # 本轮不再推进仿真。

        if time.monotonic() - self.last_cmd_wall_time > self.args.cmd_timeout:
            self._zero_cmd()  # 超过一定时间没收到速度命令就自动刹停。

        if self.runner.state in (RunnerState.PD_STAND_UP, RunnerState.STABILIZE):
            cmd = np.zeros(3, dtype=np.float32)  # 站起/稳定阶段禁止导航命令进入 policy。
        else:
            cmd = self.cmd_vel.copy()  # RL_CONTROL 后才接收导航大脑速度。

        self.runner.step(self.sim_time, cmd)  # 推进 Go2 RL 控制器和 MuJoCo 物理。
        self.sim_time += self.runner.dt  # 更新时间。

        if self.runner.step_count % self.args.publish_decimation == 0:
            self._publish_all()  # 降频发布 ROS2 话题，避免消息过密。

        if self.sim_time - self.last_status_time >= 1.0:
            self.last_status_time = self.sim_time  # 每秒打印一次。
            ros_x = float(self.runner.data.qpos[0]) + self.args.map_offset_x  # 带地图偏移的 ROS x。
            ros_y = float(self.runner.data.qpos[1]) + self.args.map_offset_y  # 带地图偏移的 ROS y。
            self.get_logger().info(
                f"t={self.sim_time:.1f}s state={self.runner.state.value} "
                f"pose=({ros_x:+.2f},{ros_y:+.2f}) "
                f"cmd=({self.cmd_vel[0]:+.2f},{self.cmd_vel[1]:+.2f},{self.cmd_vel[2]:+.2f}) "
                f"cmd_publishers={self.count_publishers('/cmd_vel')}"
            )  # 输出关键状态，方便判断闭环是否真的在动。

    def _zero_cmd(self) -> None:
        """清零速度命令。"""
        self.cmd_vel[:] = 0.0  # 三轴速度全部置零。

    def _publish_all(self) -> None:
        """发布 /clock、/odom、/scan、/tf。"""
        stamp = ros_time_from_seconds(self.sim_time)  # 当前仿真时间。
        clock = Clock()  # /clock 的消息类型是 rosgraph_msgs/Clock。
        clock.clock = stamp  # 将 builtin_interfaces/Time 放进 Clock.clock 字段。
        self.clock_pub.publish(clock)  # 发布仿真时钟。
        self._publish_odom_and_tf(stamp)  # 发布 odom 和动态 TF。
        self._publish_scan(stamp)  # 发布模拟二维激光。
        if self.map_pub is not None and int(self.sim_time * 10) % 50 == 0:
            self._publish_map(self.sim_time)  # 地图低频重发，避免 late subscriber 收不到。

    def _publish_odom_and_tf(self, stamp: Time) -> None:
        """从 MuJoCo 根节点状态发布 /odom 和 odom->base_link TF。"""
        x = float(self.runner.data.qpos[0]) + self.args.map_offset_x  # ROS 地图/里程计 x。
        y = float(self.runner.data.qpos[1]) + self.args.map_offset_y  # ROS 地图/里程计 y。
        z = float(self.runner.data.qpos[2])  # MuJoCo 世界 z。
        yaw = quat_wxyz_to_yaw(self.runner.data.qpos[3:7])  # 根节点 yaw。
        quat = yaw_to_quat(yaw)  # 转 ROS 四元数。

        odom = Odometry()  # 创建里程计消息。
        odom.header.stamp = stamp  # 时间戳。
        odom.header.frame_id = "odom"  # 全局里程计坐标系。
        odom.child_frame_id = "base_link"  # 机器人机身坐标系。
        odom.pose.pose.position.x = x  # 位置 x。
        odom.pose.pose.position.y = y  # 位置 y。
        odom.pose.pose.position.z = z  # 位置 z。
        odom.pose.pose.orientation = quat  # 姿态。
        odom.twist.twist.linear.x = float(self.runner.data.qvel[0])  # 世界系线速度 x，给 Nav2 够用。
        odom.twist.twist.linear.y = float(self.runner.data.qvel[1])  # 世界系线速度 y。
        odom.twist.twist.angular.z = float(self.runner.data.qvel[5])  # yaw 角速度近似。
        self.odom_pub.publish(odom)  # 发布 /odom。

        tf_msg = TransformStamped()  # 创建动态 TF。
        tf_msg.header.stamp = stamp  # 时间戳。
        tf_msg.header.frame_id = "odom"  # 父坐标系。
        tf_msg.child_frame_id = "base_link"  # 子坐标系。
        tf_msg.transform.translation.x = x  # 平移 x。
        tf_msg.transform.translation.y = y  # 平移 y。
        tf_msg.transform.translation.z = 0.0  # 导航只看平面，base_link 放到地面投影。
        tf_msg.transform.rotation = quat  # 旋转。
        self.tf_broadcaster.sendTransform(tf_msg)  # 发布 odom->base_link。

    def _publish_static_tf(self) -> None:
        """发布 base_link->base_scan 的静态 TF。"""
        tf_msg = TransformStamped()  # 静态 TF 消息。
        tf_msg.header.stamp = ros_time_from_seconds(0.0)  # 静态关系时间戳无所谓。
        tf_msg.header.frame_id = "base_link"  # 激光挂在机身坐标系下。
        tf_msg.child_frame_id = "base_scan"  # 激光坐标系。
        tf_msg.transform.translation.x = self.args.scan_x  # 激光相对机身前向偏移。
        tf_msg.transform.translation.y = 0.0  # 激光相对机身侧向偏移。
        tf_msg.transform.translation.z = self.args.scan_z  # 激光高度。
        tf_msg.transform.rotation.w = 1.0  # 无旋转。
        self.static_tf_broadcaster.sendTransform(tf_msg)  # 发布静态 TF。

    def _publish_map(self, sim_time: float) -> None:
        """发布 OccupancyGrid 地图，便于 RViz 观察和 Nav2 使用。"""
        msg = OccupancyGrid()  # 创建地图消息。
        msg.header.stamp = ros_time_from_seconds(sim_time)  # 时间戳。
        msg.header.frame_id = "map"  # 地图坐标系。
        msg.info.resolution = self.map_data.resolution  # 分辨率。
        msg.info.width = self.map_data.width  # 宽度。
        msg.info.height = self.map_data.height  # 高度。
        msg.info.origin.position.x = self.map_data.origin_x  # 原点 x。
        msg.info.origin.position.y = self.map_data.origin_y  # 原点 y。
        msg.info.origin.orientation.w = 1.0  # 地图无旋转。
        msg.data = self.map_data.occupancy[::-1, :].reshape(-1).astype(np.int8).tolist()  # ROS OccupancyGrid 从左下角开始。
        if self.map_pub is not None:
            self.map_pub.publish(msg)  # 发布 /map。

    def _publish_scan(self, stamp: Time) -> None:
        """根据地图和机器人当前位姿模拟一个二维 LaserScan。"""
        x = float(self.runner.data.qpos[0]) + self.args.map_offset_x + self.args.scan_x * math.cos(quat_wxyz_to_yaw(self.runner.data.qpos[3:7]))  # 激光地图 x。
        y = float(self.runner.data.qpos[1]) + self.args.map_offset_y + self.args.scan_x * math.sin(quat_wxyz_to_yaw(self.runner.data.qpos[3:7]))  # 激光地图 y。
        yaw = quat_wxyz_to_yaw(self.runner.data.qpos[3:7])  # 激光朝向默认与机身一致。

        msg = LaserScan()  # 创建 LaserScan。
        msg.header.stamp = stamp  # 时间戳。
        msg.header.frame_id = "base_scan"  # 激光坐标系。
        msg.angle_min = -math.pi  # 360 度扫描起点。
        msg.angle_max = math.pi  # 360 度扫描终点。
        msg.angle_increment = 2.0 * math.pi / self.args.scan_beams  # 每束角度间隔。
        msg.time_increment = 0.0  # 简化模型，不模拟单束时间差。
        msg.scan_time = self.args.publish_decimation * self.runner.dt  # 扫描周期近似。
        msg.range_min = self.args.scan_min_range  # 最小量程。
        msg.range_max = self.args.scan_max_range  # 最大量程。
        ranges = [
            self._ray_cast(x, y, normalize_angle(yaw + msg.angle_min + i * msg.angle_increment))
            for i in range(self.args.scan_beams)
        ]  # 按每个方向射线投射。
        msg.ranges = ranges  # 写入距离数组。
        self.scan_pub.publish(msg)  # 发布 /scan。

    def _ray_cast(self, x: float, y: float, theta: float) -> float:
        """在二维栅格地图上做简单 ray casting，返回最近障碍距离。"""
        step = max(self.map_data.resolution * 0.5, 0.01)  # 射线步长，越小越精细但越耗时。
        dist = self.args.scan_min_range  # 从最小量程开始。
        while dist <= self.args.scan_max_range:
            px = x + dist * math.cos(theta)  # 当前采样点 x。
            py = y + dist * math.sin(theta)  # 当前采样点 y。
            col = int((px - self.map_data.origin_x) / self.map_data.resolution)  # 转地图列。
            row_from_bottom = int((py - self.map_data.origin_y) / self.map_data.resolution)  # 转从底部数的行。
            row = self.map_data.height - 1 - row_from_bottom  # PGM 数组行从顶部开始。
            if col < 0 or col >= self.map_data.width or row < 0 or row >= self.map_data.height:
                return float(dist)  # 射出地图边界，按命中边界处理。
            if self.map_data.occupied_mask[row, col]:
                return float(dist)  # 命中障碍物。
            dist += step  # 继续向前。
        return float("inf")  # 未命中时按 LaserScan 习惯发布 inf。

    def run_viewer_loop(self) -> None:
        """在 MuJoCo viewer 中运行，同时继续处理 ROS2 回调。"""
        with mujoco.viewer.launch_passive(self.runner.model, self.runner.data) as viewer:
            while rclpy.ok() and viewer.is_running() and not self.should_stop:
                start = time.perf_counter()  # 本轮开始时间。
                rclpy.spin_once(self, timeout_sec=0.0)  # 处理一次 ROS 回调，timer 会推进仿真。
                viewer.sync()  # 刷新 MuJoCo 窗口。
                elapsed = time.perf_counter() - start  # 本轮耗时。
                if elapsed < self.runner.dt:
                    time.sleep(self.runner.dt - elapsed)  # 尽量按实时速度播放。


def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(description="Go2 MuJoCo RL runner ROS2 navigation bridge")
    parser.add_argument("--scene-xml", type=Path, default=GO2_SCENE_XML, help="MuJoCo Go2 场景 XML")
    parser.add_argument("--map-yaml", type=Path, default=DEFAULT_MAP_YAML, help="Nav2 yaml+pgm 地图")
    parser.add_argument("--headless", action="store_true", help="不打开 MuJoCo viewer，只发布 ROS2 数据")
    parser.add_argument("--publish-map", action="store_true", help="由桥接节点发布 /map；接 Nav2 map_server 时通常不要打开")
    parser.add_argument("--duration", type=float, default=0.0, help="仿真运行时长；0 表示一直运行")
    parser.add_argument("--stand-time", type=float, default=2.5, help="PD 缓起时间")
    parser.add_argument("--stabilize-time", type=float, default=1.5, help="policy 接管前稳定时间")
    parser.add_argument("--policy-dt", type=float, default=0.02, help="RL policy 推理周期")
    parser.add_argument("--kp", type=float, default=40.0, help="RL 控制阶段 kp")
    parser.add_argument("--kd", type=float, default=0.5, help="RL 控制阶段 kd")
    parser.add_argument("--stand-kp", type=float, default=60.0, help="站立阶段 kp")
    parser.add_argument("--stand-kd", type=float, default=5.0, help="站立阶段 kd")
    parser.add_argument("--action-clip", type=float, default=3.0, help="policy action 限幅")
    parser.add_argument("--foot-force-binary", action="store_true", help="足端力改为 0/1 接触指示")
    parser.add_argument("--publish-decimation", type=int, default=10, help="每多少个 MuJoCo 步发布一次 ROS2 数据")
    parser.add_argument("--cmd-timeout", type=float, default=0.5, help="/cmd_vel 超时清零时间")
    parser.add_argument("--max-vx", type=float, default=0.45, help="桥接层 vx 安全限幅")
    parser.add_argument("--max-vy", type=float, default=0.30, help="桥接层 vy 安全限幅")
    parser.add_argument("--max-yaw", type=float, default=0.8, help="桥接层 yaw_rate 安全限幅")
    parser.add_argument("--cmd-deadband-vx", type=float, default=0.03, help="abs(vx) 小于该值时清零")
    parser.add_argument("--cmd-deadband-vy", type=float, default=0.03, help="abs(vy) 小于该值时清零")
    parser.add_argument("--cmd-deadband-yaw", type=float, default=0.05, help="abs(yaw_rate) 小于该值时清零")
    parser.add_argument("--min-exec-vx", type=float, default=0.10, help="非零 vx 的最低执行速度")
    parser.add_argument("--min-exec-vy", type=float, default=0.08, help="非零 vy 的最低执行速度")
    parser.add_argument("--min-exec-yaw", type=float, default=0.18, help="非零 yaw_rate 的最低执行角速度")
    parser.add_argument("--scan-beams", type=int, default=180, help="模拟激光束数")
    parser.add_argument("--scan-min-range", type=float, default=0.05, help="模拟激光最小量程")
    parser.add_argument("--scan-max-range", type=float, default=6.0, help="模拟激光最大量程")
    parser.add_argument("--scan-x", type=float, default=0.20, help="激光相对 base_link 的前向偏移")
    parser.add_argument("--scan-z", type=float, default=0.25, help="激光相对 base_link 的高度")
    parser.add_argument("--map-offset-x", type=float, default=0.0, help="MuJoCo 世界 x 加到 ROS map/odom 的偏移")
    parser.add_argument("--map-offset-y", type=float, default=0.0, help="MuJoCo 世界 y 加到 ROS map/odom 的偏移")
    return parser.parse_args()


def main() -> None:
    """主入口。"""
    args = parse_args()  # 解析参数。
    rclpy.init()  # 初始化 ROS2。
    node = Go2MujocoRos2Bridge(args)  # 创建桥接节点。
    try:
        if args.headless:
            while rclpy.ok() and not node.should_stop:
                rclpy.spin_once(node, timeout_sec=0.1)  # 无窗口模式循环处理 ROS2 回调，并支持 --duration 自动退出。
        else:
            node.run_viewer_loop()  # viewer 模式手动 spin_once + viewer.sync。
    finally:
        node.destroy_node()  # 销毁节点。
        rclpy.shutdown()  # 关闭 ROS2。


if __name__ == "__main__":
    main()
