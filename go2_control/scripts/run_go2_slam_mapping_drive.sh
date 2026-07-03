#!/usr/bin/env bash
set -e

# Move Go2 through a conservative scripted pattern while slam_toolbox maps.

cd /home/ros/unitree_dev
source scripts/go2_ros_env.sh

.venv-unitree/bin/python projects/go2_nav_sim/slam_mapping_drive.py \
  "$@"
