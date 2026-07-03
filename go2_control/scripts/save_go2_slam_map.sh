#!/usr/bin/env bash
set -e

# Save the current /map from slam_toolbox/nav2_map_server.

cd /home/ros/unitree_dev
source scripts/go2_ros_env.sh

MAP_NAME="${1:-/home/ros/unitree_dev/projects/go2_nav_sim/maps/go2_slam_map}"
mkdir -p "$(dirname "${MAP_NAME}")"

ros2 run nav2_map_server map_saver_cli -f "${MAP_NAME}"
