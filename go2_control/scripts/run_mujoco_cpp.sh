#!/usr/bin/env bash
set -euo pipefail

UNITREE_DEV_ROOT="${UNITREE_DEV_ROOT:-/home/ros/unitree_dev}"  # 工作区根目录
SIM_DIR="${UNITREE_DEV_ROOT}/src/unitree_mujoco/simulate"  # C++ MuJoCo 仿真源码目录
BUILD_BIN="${UNITREE_DEV_ROOT}/build/unitree_mujoco_simulate/unitree_mujoco"  # CMake 构建出的可执行文件
LOCAL_BUILD_DIR="${SIM_DIR}/build"  # 原仓库默认运行目录
SIM_BIN="${LOCAL_BUILD_DIR}/unitree_mujoco"  # 最终执行的仿真器路径

ensure_link() {
  local target="$1"  # 真实文件或目录
  local link="$2"  # 希望创建的符号链接

  if [[ -L "${link}" ]]; then
    ln -sfn "${target}" "${link}"  # 已经是链接就原子更新指向
  elif [[ ! -e "${link}" ]]; then
    ln -s "${target}" "${link}"  # 不存在时创建新链接
  fi
}

if [[ ! -x "${BUILD_BIN}" && ! -x "${SIM_BIN}" ]]; then
  echo "未找到 C++ 仿真器：${BUILD_BIN}" >&2
  echo "请先运行：bash ${UNITREE_DEV_ROOT}/scripts/build_mujoco_cpp.sh" >&2
  exit 1
fi

if [[ -x "${BUILD_BIN}" ]]; then
  mkdir -p "${LOCAL_BUILD_DIR}"  # 确保原仓库 build 目录存在
  install -m 755 "${BUILD_BIN}" "${SIM_BIN}"  # 同步最新构建产物到原仓库 build 目录
fi

mkdir -p "${UNITREE_DEV_ROOT}/build"  # C++ 程序会从工作区 build 下读取 config.yaml
ensure_link "${SIM_DIR}/config.yaml" "${UNITREE_DEV_ROOT}/build/config.yaml"  # 修复 bad file: build/config.yaml
ensure_link "${UNITREE_DEV_ROOT}/src/unitree_mujoco/unitree_robots" "${UNITREE_DEV_ROOT}/unitree_robots"  # 让模型相对路径可用

cd "${SIM_DIR}"  # MuJoCo 资源路径依赖当前工作目录
export LD_LIBRARY_PATH="${UNITREE_DEV_ROOT}/opt/unitree_robotics/lib:${SIM_DIR}/mujoco/lib:${LD_LIBRARY_PATH:-}"  # 加载 SDK2 和 MuJoCo 动态库
echo "C++ MuJoCo 启动路径：${SIM_BIN}"
echo "C++ MuJoCo 工作目录：$(pwd)"
exec "${SIM_BIN}" -r go2 -s scene.xml  # 启动 Go2 默认 scene.xml 场景
