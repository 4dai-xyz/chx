#!/usr/bin/env bash
set -e

# Open RViz in a 2D top-down view for the Go2 Nav2/MuJoCo bridge.

cd /home/ros/unitree_dev
source scripts/go2_ros_env.sh

RVIZ_CONFIG="${GO2_NAV_RVIZ_CONFIG:-/home/ros/unitree_dev/projects/go2_nav_sim/config/go2_nav2_2d_view.rviz}"
echo "Go2 RViz config: ${RVIZ_CONFIG}"

rviz2 -d "${RVIZ_CONFIG}" "$@"
