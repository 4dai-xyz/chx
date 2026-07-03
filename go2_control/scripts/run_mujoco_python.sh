#!/usr/bin/env bash
set -euo pipefail

UNITREE_DEV_ROOT="${UNITREE_DEV_ROOT:-/home/ros/unitree_dev}"  # 工作区根目录
VENV_DIR="${UNITREE_DEV_ROOT}/.venv-unitree"  # MuJoCo Python 专用虚拟环境
SIM_PY_DIR="${UNITREE_DEV_ROOT}/src/unitree_mujoco/simulate_python"  # Python 版仿真入口目录

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  echo "未找到 Python 虚拟环境：${VENV_DIR}" >&2
  echo "请先运行：bash ${UNITREE_DEV_ROOT}/scripts/setup_python_env.sh" >&2
  exit 1
fi

cd "${SIM_PY_DIR}"  # 切到脚本目录，保证相对路径能找到模型和配置
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  "${VENV_DIR}/bin/python" ./unitree_mujoco.py  # 启动 Python 版 MuJoCo 实时仿真
