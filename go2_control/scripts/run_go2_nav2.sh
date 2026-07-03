#!/usr/bin/env bash
set -e

# 启动 Nav2：map_server + AMCL + planner + controller + BT navigator。
# Go2 的 MuJoCo 桥接节点需要先启动，因为 Nav2 依赖 /tf /odom /scan /clock。

cd /home/ros/unitree_dev
source scripts/go2_ros_env.sh

GO2_NAV_MAP_YAML="${GO2_NAV_MAP_YAML:-/home/ros/unitree_dev/projects/go2_nav_sim/maps/go2_office_maze.yaml}"
echo "Go2 Nav map: ${GO2_NAV_MAP_YAML}"

ros2 launch nav2_bringup bringup_launch.py \
  slam:=False \
  use_sim_time:=True \
  autostart:=True \
  use_composition:=True \
  map:="${GO2_NAV_MAP_YAML}" \
  params_file:=/home/ros/unitree_dev/projects/go2_nav_sim/config/nav2_go2_params.yaml \
  "$@"
