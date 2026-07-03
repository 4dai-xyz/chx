#!/usr/bin/env python3
"""Publish a conservative exploration pattern for slam_toolbox mapping."""

from __future__ import annotations

import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class SlamMappingDrive(Node):
    """Drive a small repeatable pattern while SLAM builds a map."""

    def __init__(self, args: argparse.Namespace):
        super().__init__("go2_slam_mapping_drive")
        self.args = args
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.start_wall_time = time.monotonic()
        self.timer = self.create_timer(0.05, self._timer_cb)
        self.get_logger().info("slam mapping drive started.")

    def _timer_cb(self) -> None:
        elapsed = time.monotonic() - self.start_wall_time
        msg = Twist()

        if elapsed < self.args.wait:
            pass
        elif elapsed < self.args.wait + self.args.duration:
            local_t = (elapsed - self.args.wait) % self.args.loop_time
            if local_t < self.args.forward_time:
                msg.linear.x = self.args.vx
            elif local_t < self.args.forward_time + self.args.turn_time:
                msg.angular.z = self.args.yaw
            elif local_t < self.args.forward_time + self.args.turn_time + self.args.strafe_time:
                msg.linear.y = self.args.vy
            elif local_t < self.args.forward_time + self.args.turn_time + 2.0 * self.args.strafe_time:
                msg.linear.y = -self.args.vy
            else:
                msg.angular.z = -0.5 * self.args.yaw
        else:
            self.get_logger().info("slam mapping drive finished, publishing stop command.")
            self.pub.publish(msg)
            raise SystemExit(0)

        self.pub.publish(msg)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish a scripted /cmd_vel pattern for Go2 SLAM mapping")
    parser.add_argument("--wait", type=float, default=5.0, help="Seconds to wait before moving")
    parser.add_argument("--duration", type=float, default=90.0, help="Total exploration duration after wait")
    parser.add_argument("--vx", type=float, default=0.18, help="Forward velocity")
    parser.add_argument("--vy", type=float, default=0.10, help="Strafe velocity")
    parser.add_argument("--yaw", type=float, default=0.30, help="Yaw velocity")
    parser.add_argument("--forward-time", type=float, default=5.0, help="Forward segment duration")
    parser.add_argument("--turn-time", type=float, default=4.0, help="Turn segment duration")
    parser.add_argument("--strafe-time", type=float, default=3.0, help="Each strafe segment duration")
    parser.add_argument("--settle-time", type=float, default=2.0, help="Slow counter-turn segment duration")
    args = parser.parse_args()
    args.loop_time = args.forward_time + args.turn_time + 2.0 * args.strafe_time + args.settle_time
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = SlamMappingDrive(args)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
