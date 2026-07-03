#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -eq 0 ]; then
  echo "usage: scripts/run_ros2_clean.sh <command> [args...]"
  exit 2
fi

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEFAULT_ORB_SLAM3_DIR="$REPO/Opensource code/ORB_SLAM3-master"
DEFAULT_PANGOLIN_PREFIX="$REPO/.local/pangolin"
mkdir -p "$REPO/log/runtime_test"

env -i \
  HOME="${HOME:-/home/ros}" \
  USER="${USER:-ros}" \
  DISPLAY="${DISPLAY:-}" \
  WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" \
  XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" \
  PULSE_SERVER="${PULSE_SERVER:-}" \
  PATH=/usr/local/cuda-12.1/bin:/usr/lib/wsl/lib:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  PYTHONNOUSERSITE=1 \
  ROS_LOG_DIR="$REPO/log/runtime_test" \
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}" \
  ROS2_ORBSLAM3_REPO="$REPO" \
  ORB_SLAM3_DIR="${ORB_SLAM3_DIR:-$DEFAULT_ORB_SLAM3_DIR}" \
  PANGOLIN_PREFIX="${PANGOLIN_PREFIX:-$DEFAULT_PANGOLIN_PREFIX}" \
  DPVO_PYTHON="${DPVO_PYTHON:-/home/ros/miniconda3/envs/dpvo/bin/python}" \
  /bin/bash --noprofile --norc -c '
    set -euo pipefail
    cd /home/ros/ros2_orbslam3
    set +u
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    set -u
    export CMAKE_PREFIX_PATH="${PANGOLIN_PREFIX}:${CMAKE_PREFIX_PATH:-}"
    export LD_LIBRARY_PATH="${ORB_SLAM3_DIR}/lib:${PANGOLIN_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
    "$@"
  ' bash "$@"
