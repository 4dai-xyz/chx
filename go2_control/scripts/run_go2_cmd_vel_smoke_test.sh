#!/usr/bin/env bash
set -e

# 不经过 Nav2，直接发布 /cmd_vel，验证 ROS2 -> Go2 MuJoCo RL 控制闭环。

cd /home/ros/unitree_dev
source scripts/go2_ros_env.sh

.venv-unitree/bin/python projects/go2_nav_sim/cmd_vel_smoke_test.py \
  "$@"
