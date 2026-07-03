#!/usr/bin/env python3
"""
Go2 ROS2-MuJoCo 桥接的最小闭环冒烟测试。

这个脚本只发布 /cmd_vel，不做路径规划。用途是快速验证：
  1. ROS2 话题能发出去。
  2. go2_mujoco_ros2_bridge.py 能收到 /cmd_vel。
  3. MuJoCo 中的 Go2 RL 底层控制器能被 ROS2 速度命令驱动。
  4. /odom、/scan、/tf 能持续发布。

如果这个脚本能让 Go2 在 MuJoCo 里稳定完成“前进、转向、横移、停止”，再接 Nav2 就更稳。
"""

from __future__ import annotations

import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelSmokeTest(Node):
    """按固定时间表发布速度命令。"""

    def __init__(self, args: argparse.Namespace):
        super().__init__("go2_cmd_vel_smoke_test")  # ROS2 节点名。
        self.args = args  # 保存命令行参数。
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)  # 发布给桥接节点的速度命令。
        self.start_wall_time: float | None = None  # 第一次发现订阅者后才开始计时。
        self.last_phase = ""  # 上一次打印的运动阶段。
        self.last_status_wall_time = 0.0  # 上一次打印订阅状态的真实时间。
        self.timer = self.create_timer(0.05, self._timer_cb)  # 20Hz 发布，接近 Nav2 controller 频率。
        self.get_logger().info("cmd_vel smoke test started.")  # 启动日志。

    def _timer_cb(self) -> None:
        """按固定脚本生成速度命令并发布。"""
        sub_count = self.pub.get_subscription_count()  # 当前 /cmd_vel 订阅者数量。
        now = time.monotonic()  # 当前真实时间。

        if sub_count == 0 and self.args.require_subscriber:
            if now - self.last_status_wall_time >= 1.0:
                self.last_status_wall_time = now  # 每秒检查一次连接状态。
                self.get_logger().warn(
                    "waiting for /cmd_vel subscriber. Start scripts/run_go2_nav_bridge.sh in another terminal first."
                )  # 没有桥接节点订阅时给出明确提示。
            return  # 没有桥接节点时不消耗动作脚本时间。

        if self.start_wall_time is None:
            self.start_wall_time = now  # 发现订阅者后再开始动作序列。
            self.get_logger().info(f"/cmd_vel subscriber count={sub_count}, starting scripted motion.")

        elapsed = now - self.start_wall_time  # 已运行真实时间。
        msg = Twist()  # 默认全 0，即停止。

        if elapsed < self.args.wait:
            phase = "wait"
            pass  # 阶段 0：等待 Go2 站稳。
        elif elapsed < self.args.wait + self.args.forward_duration:
            phase = "forward"
            msg.linear.x = self.args.forward_vx  # 阶段 1：前进。
        elif elapsed < self.args.wait + self.args.forward_duration + self.args.turn_duration:
            phase = "turn"
            msg.angular.z = self.args.turn_yaw  # 阶段 2：原地转向。
        elif elapsed < self.args.wait + self.args.forward_duration + self.args.turn_duration + self.args.left_duration:
            phase = "strafe_left"
            msg.linear.y = self.args.strafe_vy  # 阶段 3：向左横移。
        elif elapsed < self.args.wait + self.args.forward_duration + self.args.turn_duration + self.args.left_duration + self.args.right_duration:
            phase = "strafe_right"
            msg.linear.y = -self.args.strafe_vy  # 阶段 4：向右横移。
        else:
            self.get_logger().info("cmd_vel smoke test finished, publishing stop command.")  # 结束提示。
            self.pub.publish(msg)  # 结束前再发一次停止。
            raise SystemExit(0)  # 退出脚本。

        if phase != self.last_phase:
            self.last_phase = phase  # 保存阶段，避免刷屏。
            self.get_logger().info(
                f"phase={phase} cmd=({msg.linear.x:+.2f},{msg.linear.y:+.2f},{msg.angular.z:+.2f})"
            )  # 打印当前阶段和速度命令。

        if now - self.last_status_wall_time >= 1.0:
            self.last_status_wall_time = now  # 每秒检查一次连接状态。
            if sub_count == 0:
                self.get_logger().warn(
                    "no /cmd_vel subscriber found. Start scripts/run_go2_nav_bridge.sh in another terminal first."
                )  # 没有桥接节点订阅时给出明确提示。

        self.pub.publish(msg)  # 发布当前阶段速度。


def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(description="Publish scripted /cmd_vel for Go2 MuJoCo ROS2 bridge")
    parser.add_argument("--wait", type=float, default=5.0, help="开始运动前等待时间，给 Go2 站起和 policy 接管")
    parser.add_argument("--forward-vx", type=float, default=0.25, help="前进速度")
    parser.add_argument("--forward-duration", type=float, default=4.0, help="前进持续时间")
    parser.add_argument("--turn-yaw", type=float, default=0.35, help="转向角速度")
    parser.add_argument("--turn-duration", type=float, default=4.0, help="转向持续时间")
    parser.add_argument("--strafe-vy", type=float, default=0.18, help="横移速度")
    parser.add_argument("--left-duration", type=float, default=3.0, help="向左横移持续时间")
    parser.add_argument("--right-duration", type=float, default=3.0, help="向右横移持续时间")
    parser.add_argument(
        "--no-require-subscriber",
        action="store_false",
        dest="require_subscriber",
        help="即使没有 /cmd_vel 订阅者也照常跑完整动作序列",
    )
    parser.set_defaults(require_subscriber=True)
    return parser.parse_args()


def main() -> None:
    """主入口。"""
    args = parse_args()  # 解析参数。
    rclpy.init()  # 初始化 ROS2。
    node = CmdVelSmokeTest(args)  # 创建发布节点。
    try:
        rclpy.spin(node)  # 运行到脚本主动退出。
    except SystemExit:
        pass  # 正常结束。
    finally:
        node.destroy_node()  # 销毁节点。
        rclpy.shutdown()  # 关闭 ROS2。


if __name__ == "__main__":
    main()
