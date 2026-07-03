#!/usr/bin/env bash
set -e

# 给 Nav2 发送一个目标点，验证 planner/controller -> /cmd_vel -> Go2 MuJoCo 的完整闭环。

cd /home/ros/unitree_dev
source scripts/go2_ros_env.sh

GO2_NAV_MAP_YAML="${GO2_NAV_MAP_YAML:-/home/ros/unitree_dev/projects/go2_nav_sim/maps/go2_office_maze.yaml}"
GO2_NAV_START_X="${GO2_NAV_START_X:-0.0}"
GO2_NAV_START_Y="${GO2_NAV_START_Y:-0.0}"
GO2_NAV_START_YAW="${GO2_NAV_START_YAW:-0.0}"
GO2_NAV_GOAL_X="${GO2_NAV_GOAL_X:--3.1}"
GO2_NAV_GOAL_Y="${GO2_NAV_GOAL_Y:-2.7}"
GO2_NAV_GOAL_YAW="${GO2_NAV_GOAL_YAW:-0.0}"
echo "Go2 Nav map: ${GO2_NAV_MAP_YAML}"

python3 projects/go2_nav_sim/nav2_send_goal.py \
  --map-yaml "${GO2_NAV_MAP_YAML}" \
  --start-x "${GO2_NAV_START_X}" \
  --start-y "${GO2_NAV_START_Y}" \
  --start-yaw "${GO2_NAV_START_YAW}" \
  --goal-x "${GO2_NAV_GOAL_X}" \
  --goal-y "${GO2_NAV_GOAL_Y}" \
  --goal-yaw "${GO2_NAV_GOAL_YAW}" \
  "$@"
