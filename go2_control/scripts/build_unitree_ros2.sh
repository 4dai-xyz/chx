#!/usr/bin/env bash
set -euo pipefail

UNITREE_DEV_ROOT="${UNITREE_DEV_ROOT:-/home/ros/unitree_dev}"  # 工作区根目录
ROS_DISTRO="${ROS_DISTRO:-humble}"  # 默认使用 ROS 2 Humble

had_nounset=0  # 记录当前 shell 是否开启 set -u，source ROS 时要临时关闭
case "$-" in
  *u*)
    had_nounset=1
    set +u
    ;;
esac
source "/opt/ros/${ROS_DISTRO}/setup.bash"  # 加载 ROS 2 基础环境
if [[ "${had_nounset}" == "1" ]]; then
  set -u
fi
unset had_nounset

# 只构建 unitree_ros2 的 cyclonedds_ws，构建产物放到 unitree_dev/build 和 install。
colcon build \
  --base-paths "${UNITREE_DEV_ROOT}/src/unitree_ros2/cyclonedds_ws/src" \
  --build-base "${UNITREE_DEV_ROOT}/build/unitree_ros2" \
  --install-base "${UNITREE_DEV_ROOT}/install/unitree_ros2" \
  --merge-install

echo "unitree_ros2 接口构建完成。"
echo "加载：source ${UNITREE_DEV_ROOT}/scripts/unitree_env.sh"
