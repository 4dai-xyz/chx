#!/usr/bin/env bash
set -e

# Start Nav2 using a map saved from scripts/save_go2_slam_map.sh.

cd /home/ros/unitree_dev
source scripts/go2_ros_env.sh

MAP_YAML="${1:-/home/ros/unitree_dev/projects/go2_nav_sim/maps/go2_slam_map.yaml}"

ros2 launch nav2_bringup bringup_launch.py \
  slam:=False \
  use_sim_time:=True \
  autostart:=True \
  use_composition:=True \
  map:="${MAP_YAML}" \
  params_file:=/home/ros/unitree_dev/projects/go2_nav_sim/config/nav2_go2_params.yaml
