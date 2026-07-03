#!/usr/bin/env bash
set -euo pipefail

UNITREE_DEV_ROOT="${UNITREE_DEV_ROOT:-/home/ros/unitree_dev}"  # 工作区根目录
VENV_DIR="${UNITREE_DEV_ROOT}/.venv-unitree"  # Python 版 MuJoCo 使用的虚拟环境

echo "创建/更新 Python 虚拟环境：${VENV_DIR}"
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  python3 -m venv "${VENV_DIR}"  # 使用系统 python3 创建 venv，避免 conda 污染

echo "安装 Python 依赖：unitree_sdk2_python、mujoco 3.3.6、pygame"
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  "${VENV_DIR}/bin/python" -m pip install -e "${UNITREE_DEV_ROOT}/src/unitree_sdk2_python" "mujoco==3.3.6" pygame  # 安装 SDK2 Python、MuJoCo 和键盘/窗口依赖

echo "Python 环境完成。验证命令："
echo "  ${VENV_DIR}/bin/python -c 'import unitree_sdk2py, mujoco, pygame, cyclonedds; print(\"ok\")'"
