#!/usr/bin/env bash
set -e

# Start slam_toolbox mapping for the Go2 MuJoCo ROS2 bridge.
# Run the bridge first; do not run Nav2 AMCL/map_server at the same time.

cd /home/ros/unitree_dev
source scripts/go2_ros_env.sh

ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=True \
  slam_params_file:=/home/ros/unitree_dev/projects/go2_nav_sim/config/slam_toolbox_go2_mapper.yaml \
  "$@"
