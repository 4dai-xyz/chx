#!/usr/bin/env bash
set -euo pipefail

UNITREE_DEV_ROOT="${UNITREE_DEV_ROOT:-/home/ros/unitree_dev}"  # 工作区根目录
MUJOCO_VERSION="${MUJOCO_VERSION:-3.3.6}"  # 与 Python 依赖保持一致的 MuJoCo 版本
MUJOCO_ARCHIVE="mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz"  # 官方 Linux 压缩包文件名
MUJOCO_URL="https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/${MUJOCO_ARCHIVE}"  # 官方下载地址
DOWNLOAD_DIR="${UNITREE_DEV_ROOT}/downloads"  # 下载缓存目录
MUJOCO_DIR="/home/ros/.mujoco/mujoco-${MUJOCO_VERSION}"  # MuJoCo 解压安装目录
SIMULATE_DIR="${UNITREE_DEV_ROOT}/src/unitree_mujoco/simulate"  # unitree_mujoco C++ 仿真目录

mkdir -p "${DOWNLOAD_DIR}" /home/ros/.mujoco  # 确保缓存和安装目录存在

if [[ ! -f "${DOWNLOAD_DIR}/${MUJOCO_ARCHIVE}" ]]; then
  echo "下载 MuJoCo ${MUJOCO_VERSION}: ${MUJOCO_URL}"
  wget -O "${DOWNLOAD_DIR}/${MUJOCO_ARCHIVE}" "${MUJOCO_URL}"  # 下载 MuJoCo 官方包
else
  echo "已存在下载包：${DOWNLOAD_DIR}/${MUJOCO_ARCHIVE}"
fi

if [[ ! -d "${MUJOCO_DIR}" ]]; then
  echo "解压到 /home/ros/.mujoco"
  tar -xzf "${DOWNLOAD_DIR}/${MUJOCO_ARCHIVE}" -C /home/ros/.mujoco  # 解压官方包
else
  echo "MuJoCo 已存在：${MUJOCO_DIR}"
fi

if [[ -d "${SIMULATE_DIR}" ]]; then
  rm -f "${SIMULATE_DIR}/mujoco"  # 删除旧链接，避免指向旧版本
  ln -s "${MUJOCO_DIR}" "${SIMULATE_DIR}/mujoco"  # 创建 unitree_mujoco 期望的 mujoco 目录链接
  echo "已创建链接：${SIMULATE_DIR}/mujoco -> ${MUJOCO_DIR}"
fi

echo "MuJoCo 安装准备完成。"
