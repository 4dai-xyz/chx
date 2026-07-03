#!/usr/bin/env bash
set -euo pipefail

UNITREE_DEV_ROOT="${UNITREE_DEV_ROOT:-/home/ros/unitree_dev}"  # 工作区根目录
SIM_SRC="${UNITREE_DEV_ROOT}/src/unitree_mujoco/simulate"  # C++ MuJoCo 仿真源码目录
SIM_BUILD="${UNITREE_DEV_ROOT}/build/unitree_mujoco_simulate"  # CMake 独立构建目录
SIM_LOCAL_BUILD="${SIM_SRC}/build"  # unitree_mujoco 原仓库默认查找可执行文件的位置
UNITREE_PREFIX="${UNITREE_DEV_ROOT}/opt/unitree_robotics"  # 本地安装的 Unitree SDK2 前缀

# 清理 conda/DPVO 变量后配置 CMake，避免链接到错误的动态库。
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  CMAKE_PREFIX_PATH="${UNITREE_PREFIX}" \
  cmake -S "${SIM_SRC}" -B "${SIM_BUILD}"

# 使用全部 CPU 核心并行编译。
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  cmake --build "${SIM_BUILD}" -j"$(nproc)"

mkdir -p "${SIM_LOCAL_BUILD}"  # 确保原仓库 build 目录存在
install -m 755 "${SIM_BUILD}/unitree_mujoco" "${SIM_LOCAL_BUILD}/unitree_mujoco"  # 同步一份给原运行脚本

echo "C++ MuJoCo 仿真器构建完成：${SIM_BUILD}/unitree_mujoco"
echo "C++ MuJoCo 运行副本已同步：${SIM_LOCAL_BUILD}/unitree_mujoco"
