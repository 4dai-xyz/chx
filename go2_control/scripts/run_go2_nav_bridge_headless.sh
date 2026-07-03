#!/usr/bin/env bash
set -e

# 无窗口运行 Go2 MuJoCo RL 控制器 + ROS2 桥接。
# 适合先做话题自检，或者在另一个终端用 RViz 观察。

cd /home/ros/unitree_dev
source scripts/go2_ros_env.sh

GO2_NAV_MAP_YAML="${GO2_NAV_MAP_YAML:-/home/ros/unitree_dev/projects/go2_nav_sim/maps/go2_office_maze.yaml}"
GO2_MAP_OFFSET_X="${GO2_MAP_OFFSET_X:-0.0}"
GO2_MAP_OFFSET_Y="${GO2_MAP_OFFSET_Y:-0.0}"
echo "Go2 Nav map: ${GO2_NAV_MAP_YAML}"
echo "Go2 map offset: x=${GO2_MAP_OFFSET_X}, y=${GO2_MAP_OFFSET_Y}"

.venv-unitree/bin/python projects/go2_nav_sim/go2_mujoco_ros2_bridge.py \
  --headless \
  --map-yaml "${GO2_NAV_MAP_YAML}" \
  --map-offset-x "${GO2_MAP_OFFSET_X}" \
  --map-offset-y "${GO2_MAP_OFFSET_Y}" \
  "$@"
