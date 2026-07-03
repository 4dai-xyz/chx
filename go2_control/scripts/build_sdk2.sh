#!/usr/bin/env bash
set -euo pipefail

UNITREE_DEV_ROOT="${UNITREE_DEV_ROOT:-/home/ros/unitree_dev}"  # 工作区根目录
SDK_SRC="${UNITREE_DEV_ROOT}/src/unitree_sdk2"  # Unitree SDK2 C++ 源码
SDK_BUILD="${UNITREE_DEV_ROOT}/build/unitree_sdk2"  # SDK2 构建目录
SDK_PREFIX="${UNITREE_DEV_ROOT}/opt/unitree_robotics"  # SDK2 安装目录，供 MuJoCo C++ 桥接查找

# 配置 SDK2；BUILD_EXAMPLES=ON 方便后续参考官方示例。
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  cmake -S "${SDK_SRC}" -B "${SDK_BUILD}" -DBUILD_EXAMPLES=ON

# 编译 SDK2。
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  cmake --build "${SDK_BUILD}" -j"$(nproc)"

# 安装到工作区内部，避免污染 /usr/local。
env -u LD_LIBRARY_PATH -u PYTHONPATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
  cmake --install "${SDK_BUILD}" --prefix "${SDK_PREFIX}"

echo "SDK2 已构建并安装到：${SDK_PREFIX}"
