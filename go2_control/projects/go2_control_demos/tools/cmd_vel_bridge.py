#!/usr/bin/env python3
"""
cmd_vel 桥接节点 —— ROS2 → 策略之间的粘合剂。

本文件的作用：
  订阅 ROS2 /cmd_vel 话题（geometry_msgs/Twist），
  把收到的速度指令 [vx, vy, yaw_rate] 写入共享文件，
  供 MuJoCo 策略 runner 读取。

运行方式（需要 source ROS2 环境）：
  # 终端 1：先启动这个桥接
  cd /home/ros/unitree_dev
  source /opt/ros/humble/setup.bash
  python3 projects/go2_control_demos/tools/cmd_vel_bridge.py

  # 终端 2：再启动 MuJoCo runner
  cd /home/ros/unitree_dev
  env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
    .venv-unitree/bin/python projects/go2_control_demos/go2_rl_runner.py \
    --cmd-source file

  # 终端 3：发送 cmd_vel 测试指令
  source /opt/ros/humble/setup.bash
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
"""

import os
import json
import time

# ROS2 日志目录修复（WSL2 特殊配置：~/.ros 可能是只读文件系统）
os.environ.setdefault("ROS_LOG_DIR", "/tmp/ros_logs")
os.makedirs("/tmp/ros_logs", exist_ok=True)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# ============================================================
# 1. 共享文件路径
# ============================================================
# 两个进程通过这个文件传递 cmd_vel
# 原子写入：先写临时文件，再 os.replace，避免 reader 读到半截数据
CMD_VEL_FILE = "/tmp/go2_cmd_vel.json"

# 默认指令（无话题数据时：站立不动）
DEFAULT_CMD = {"vx": 0.0, "vy": 0.0, "yaw_rate": 0.0}


# ============================================================
# 2. 原子文件写入
# ============================================================
def write_cmd_atomic(vx: float, vy: float, yaw_rate: float):
    """
    原子写入 cmd_vel 到共享文件。

    使用 tempfile + os.replace，保证 reader 永远不会读到不完整的数据：
      1. 写入临时文件 /tmp/go2_cmd_vel.json.tmp
      2. os.replace 原子替换（Linux 上的 rename 是原子的）
    """
    data = {
        "vx": float(vx),
        "vy": float(vy),
        "yaw_rate": float(yaw_rate),
        "ts": time.time(),
    }
    tmp_path = CMD_VEL_FILE + ".tmp"
    with open(tmp_path, "w") as f:
        json.dump(data, f)
        f.flush()           # 确保写入磁盘
        os.fsync(f.fileno())  # 确保文件系统同步
    os.replace(tmp_path, CMD_VEL_FILE)  # 原子替换


# ============================================================
# 3. ROS2 订阅节点
# ============================================================
class CmdVelBridge(Node):
    """
    订阅 /cmd_vel (Twist)，把数据写入共享文件。

    ROS2 话题：/cmd_vel
    消息类型：geometry_msgs/msg/Twist
      linear.x  → 前进速度 (m/s)
      linear.y  → 侧移速度 (m/s)
      angular.z → 偏航角速度 (rad/s)
    """

    def __init__(self):
        super().__init__("cmd_vel_bridge")
        self.subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10,  # QoS: 队列深度
        )
        self.get_logger().info("cmd_vel_bridge 已就绪，等待 /cmd_vel ...")
        self.get_logger().info(f"共享文件: {CMD_VEL_FILE}")

    def cmd_vel_callback(self, msg: Twist):
        """每次收到 /cmd_vel 消息时调用。"""
        vx = msg.linear.x
        vy = msg.linear.y
        yaw_rate = msg.angular.z
        write_cmd_atomic(vx, vy, yaw_rate)
        # 每 100 次打印一次，避免刷屏
        if int(time.time() * 10) % 100 == 0:
            self.get_logger().info(
                f"收到 cmd_vel: vx={vx:.2f}, vy={vy:.2f}, yaw={yaw_rate:.2f}"
            )


# ============================================================
# 4. 主函数
# ============================================================
def main():
    # 初始化共享文件为默认指令
    write_cmd_atomic(**DEFAULT_CMD)

    rclpy.init()
    node = CmdVelBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前恢复默认指令（方便仿真端安全停止）
        write_cmd_atomic(**DEFAULT_CMD)
        node.destroy_node()
        rclpy.shutdown()
        print("cmd_vel_bridge 已退出。")


if __name__ == "__main__":
    main()
