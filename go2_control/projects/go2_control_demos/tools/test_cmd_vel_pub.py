#!/usr/bin/env python3
"""
cmd_vel 测试发布者 —— 发送固定速度指令测试桥接链路。

作用：发布一个 /cmd_vel 消息，验证 桥接节点→共享文件→runner 的链路。

运行方式：
  source /opt/ros/humble/setup.bash
  python3 projects/go2_control_demos/tools/test_cmd_vel_pub.py [vx] [vy] [yaw_rate]
"""

import sys
import os

# ROS2 日志目录修复（WSL2 特殊配置）
os.environ.setdefault("ROS_LOG_DIR", "/tmp/ros_logs")
os.makedirs("/tmp/ros_logs", exist_ok=True)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def main():
    rclpy.init()
    node = Node("test_cmd_vel_pub")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)

    # 解析命令行参数
    vx = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
    vy = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

    msg = Twist()
    msg.linear.x = vx
    msg.linear.y = vy
    msg.angular.z = yaw

    pub.publish(msg)
    node.get_logger().info(f"已发布: vx={vx:.2f}, vy={vy:.2f}, yaw={yaw:.2f}")

    # 确保消息发出
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()