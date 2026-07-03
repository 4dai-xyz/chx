#!/usr/bin/env bash
set -euo pipefail

UNITREE_DEV_ROOT="${UNITREE_DEV_ROOT:-/home/ros/unitree_dev}"  # 工作区根目录
VENV_DIR="${UNITREE_DEV_ROOT}/.venv-unitree"  # MuJoCo Python 专用虚拟环境
SIM_PY_DIR="${UNITREE_DEV_ROOT}/src/unitree_mujoco/simulate_python"  # Python 仿真目录

cd "${SIM_PY_DIR}"  # 测试脚本依赖当前目录下的相对路径
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  "${VENV_DIR}/bin/python" ./test/test_unitree_sdk2.py  # 运行 SDK2/Python 桥接测试
