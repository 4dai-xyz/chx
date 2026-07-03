#!/usr/bin/env bash
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PACKAGES=("$@")
DEFAULT_ORB_SLAM3_DIR="$REPO/Opensource code/ORB_SLAM3-master"
DEFAULT_PANGOLIN_PREFIX="$REPO/.local/pangolin"

if [ "${#PACKAGES[@]}" -eq 0 ]; then
  PACKAGES=(video_publisher dpvo_localization orbslam3_wrapper)
fi

cd "$REPO"
mkdir -p "$REPO/log/runtime_test"

env -i \
  HOME="${HOME:-/home/ros}" \
  USER="${USER:-ros}" \
  PATH=/usr/local/cuda-12.1/bin:/usr/lib/wsl/lib:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  PYTHONNOUSERSITE=1 \
  ROS_LOG_DIR="$REPO/log/runtime_test" \
  ORB_SLAM3_DIR="${ORB_SLAM3_DIR:-$DEFAULT_ORB_SLAM3_DIR}" \
  PANGOLIN_PREFIX="${PANGOLIN_PREFIX:-$DEFAULT_PANGOLIN_PREFIX}" \
  /bin/bash --noprofile --norc -c '
    set -euo pipefail
    cd /home/ros/ros2_orbslam3
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    export CMAKE_PREFIX_PATH="${PANGOLIN_PREFIX}:${CMAKE_PREFIX_PATH:-}"
    export LD_LIBRARY_PATH="${ORB_SLAM3_DIR}/lib:${PANGOLIN_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
    colcon build --symlink-install --base-paths src --packages-select "$@"
  ' bash "${PACKAGES[@]}"
