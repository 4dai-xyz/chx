#!/usr/bin/env bash
set -euo pipefail

UNITREE_DEV_ROOT="${UNITREE_DEV_ROOT:-/home/ros/unitree_dev}"  # 工作区根目录，可通过环境变量覆盖

echo "1/4 构建 SDK2"  # 先构建 Unitree C++ SDK，后续 C++ 仿真桥接会用到
bash "${UNITREE_DEV_ROOT}/scripts/build_sdk2.sh"

echo "2/4 检查/准备 MuJoCo"  # 下载或检查 MuJoCo，并在仿真目录创建链接
bash "${UNITREE_DEV_ROOT}/scripts/install_mujoco.sh"

echo "3/4 配置 Python 环境"  # 创建 .venv-unitree，用于 Python 版 MuJoCo 仿真
bash "${UNITREE_DEV_ROOT}/scripts/setup_python_env.sh"

echo "4/4 尝试构建 C++ MuJoCo 仿真器"  # 编译 C++ 版 unitree_mujoco
if ! bash "${UNITREE_DEV_ROOT}/scripts/build_mujoco_cpp.sh"; then
  echo
  echo "C++ MuJoCo 仿真器构建失败。常见原因是缺少 libglfw3-dev。"
  echo "请先运行：bash ${UNITREE_DEV_ROOT}/scripts/install_system_deps.sh"
  exit 1
fi

echo "可在安装 ROS2 缺失依赖后单独运行："
echo "  bash ${UNITREE_DEV_ROOT}/scripts/build_unitree_ros2.sh"
