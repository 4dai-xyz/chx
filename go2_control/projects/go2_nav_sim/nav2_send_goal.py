#!/usr/bin/env python3
"""
给 Nav2 发送一个 NavigateToPose 目标点。

使用方式：
  1. 先启动 go2_mujoco_ros2_bridge.py。
  2. 再启动 Nav2 bringup。
  3. 最后运行本脚本发送目标点。

目标点默认选择 go2_office_maze 地图中避开 MuJoCo 前方台阶的位置，适合做复杂地图闭环测试。
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MAP_YAML = PROJECT_ROOT / "projects/go2_nav_sim/maps/go2_office_maze.yaml"


def yaw_to_quaternion_msg(yaw: float):
    """把 yaw 转成 ROS 四元数消息字段。"""
    from geometry_msgs.msg import Quaternion

    q = Quaternion()  # 创建消息。
    q.z = math.sin(yaw * 0.5)  # z 分量。
    q.w = math.cos(yaw * 0.5)  # w 分量。
    return q  # 返回四元数。


def make_pose(frame_id: str, x: float, y: float, yaw: float, stamp) -> PoseStamped:
    """构造 PoseStamped。"""
    pose = PoseStamped()  # 创建目标位姿。
    pose.header.frame_id = frame_id  # 通常是 map。
    pose.header.stamp = stamp  # 使用 Nav2 当前时钟。
    pose.pose.position.x = x  # 目标 x。
    pose.pose.position.y = y  # 目标 y。
    pose.pose.orientation = yaw_to_quaternion_msg(yaw)  # 目标朝向。
    return pose  # 返回目标。


def publish_initial_pose(navigator: BasicNavigator, x: float, y: float, yaw: float) -> None:
    """给 AMCL/Nav2 设置初始位姿。"""
    initial_pose = PoseStamped()  # BasicNavigator 接收 PoseStamped 初始位姿。
    initial_pose.header.frame_id = "map"  # 初始位姿在 map 坐标系下。
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()  # 当前 ROS 时间。
    initial_pose.pose.position.x = x  # 初始 x。
    initial_pose.pose.position.y = y  # 初始 y。
    initial_pose.pose.orientation = yaw_to_quaternion_msg(yaw)  # 初始 yaw。
    navigator.setInitialPose(initial_pose)  # 通知 Nav2 初始位姿。


def load_map_status(map_yaml: Path):
    """读取 Nav2 yaml+pgm 地图，返回坐标查询函数。"""
    with map_yaml.open("r", encoding="utf-8") as f:
        meta = yaml.safe_load(f)

    image_path = Path(meta["image"])
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path

    with image_path.open("rb") as f:
        magic = f.readline().strip()
        if magic != b"P5":
            raise ValueError(f"当前只支持 P5 PGM 地图，实际 magic={magic!r}")
        tokens: list[bytes] = []
        while len(tokens) < 3:
            line = f.readline()
            if line.startswith(b"#"):
                continue
            tokens.extend(line.split())
        width, height, maxval = map(int, tokens[:3])
        image = np.frombuffer(f.read(width * height), dtype=np.uint8).reshape((height, width))

    resolution = float(meta["resolution"])
    origin_x, origin_y, _ = [float(v) for v in meta["origin"]]
    occupied_thresh = float(meta.get("occupied_thresh", 0.65))
    free_thresh = float(meta.get("free_thresh", 0.196))
    negate = int(meta.get("negate", 0))

    if negate:
        occ_prob = image.astype(np.float32) / float(maxval)
    else:
        occ_prob = (float(maxval) - image.astype(np.float32)) / float(maxval)

    def status_at(x: float, y: float) -> str:
        col = int((x - origin_x) / resolution)
        row_from_bottom = int((y - origin_y) / resolution)
        row = height - 1 - row_from_bottom
        if col < 0 or col >= width or row < 0 or row >= height:
            return "out_of_map"
        prob = float(occ_prob[row, col])
        if prob <= free_thresh:
            return "free"
        if prob >= occupied_thresh:
            return "occupied"
        return "unknown"

    return status_at


def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(description="Send a Nav2 goal to drive Go2 MuJoCo bridge")
    parser.add_argument("--start-x", type=float, default=0.0, help="AMCL 初始 x")
    parser.add_argument("--start-y", type=float, default=0.0, help="AMCL 初始 y")
    parser.add_argument("--start-yaw", type=float, default=0.0, help="AMCL 初始 yaw")
    parser.add_argument("--goal-x", type=float, default=-4.0, help="目标点 x")
    parser.add_argument("--goal-y", type=float, default=-2.5, help="目标点 y")
    parser.add_argument("--goal-yaw", type=float, default=0.0, help="目标点 yaw")
    parser.add_argument("--map-yaml", type=Path, default=DEFAULT_MAP_YAML, help="用于检查起点/目标是否在 free cell 的地图")
    parser.add_argument("--allow-nonfree-pose", action="store_true", help="允许起点或目标不在 free cell，主要用于调试")
    parser.add_argument("--timeout", type=float, default=120.0, help="导航超时时间")
    parser.add_argument("--shutdown-nav2", action="store_true", help="任务结束后关闭 Nav2 lifecycle；默认不关闭，方便继续调试")
    return parser.parse_args()


def main() -> None:
    """主入口。"""
    args = parse_args()  # 解析参数。
    status_at = load_map_status(args.map_yaml)  # 读取地图，用来提前发现不可规划目标。
    start_status = status_at(args.start_x, args.start_y)
    goal_status = status_at(args.goal_x, args.goal_y)
    print(f"[检查] start=({args.start_x:.2f},{args.start_y:.2f}) map_status={start_status}")
    print(f"[检查] goal=({args.goal_x:.2f},{args.goal_y:.2f}) map_status={goal_status}")
    if not args.allow_nonfree_pose and (start_status != "free" or goal_status != "free"):
        raise SystemExit(
            "起点或目标点不在地图 free cell 中，Nav2 planner 很可能无法生成路径。"
            "请换用 free 坐标，或加 --allow-nonfree-pose 强制发送。"
        )

    rclpy.init()  # 初始化 ROS2。
    navigator = BasicNavigator()  # 创建 Nav2 简单控制器。

    publish_initial_pose(navigator, args.start_x, args.start_y, args.start_yaw)  # 设置初始位姿。
    navigator.waitUntilNav2Active()  # 等待 Nav2 lifecycle 节点全部 active。

    goal = make_pose("map", args.goal_x, args.goal_y, args.goal_yaw, navigator.get_clock().now().to_msg())  # 构造目标。
    navigator.goToPose(goal)  # 发送 NavigateToPose action。

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()  # 读取反馈。
        if feedback is not None and feedback.navigation_time.sec > args.timeout:
            navigator.cancelTask()  # 超时取消。
            print(f"[失败] 导航超过 {args.timeout:.1f}s，已取消。")
            break

    result = navigator.getResult()  # 获取最终结果。
    if result == TaskResult.SUCCEEDED:
        print("[成功] Nav2 已到达目标点。")
    elif result == TaskResult.CANCELED:
        print("[取消] Nav2 任务被取消。")
    else:
        print("[失败] Nav2 未能到达目标点。")

    if args.shutdown_nav2:
        navigator.lifecycleShutdown()  # 需要时才关闭 Nav2 lifecycle，避免默认把调试环境关掉。
    rclpy.shutdown()  # 关闭 ROS2。


if __name__ == "__main__":
    main()
