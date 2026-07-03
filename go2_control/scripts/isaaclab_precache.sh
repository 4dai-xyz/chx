#!/usr/bin/env bash
set -euo pipefail

ISAAC_ROOT="${ISAAC_ROOT:-/home/ros/isaac_go2}"  # Isaac 工作区根目录
CONDA_SH="${CONDA_SH:-/home/ros/miniconda3/etc/profile.d/conda.sh}"  # conda 初始化脚本路径
CONDA_ENV="${CONDA_ENV:-env_isaaclab312}"  # Isaac Lab conda 环境名
CONDA_ROOT="${CONDA_ROOT:-/home/ros/miniconda3}"  # Miniconda 根目录
CONDA_ENV_DIR="${CONDA_ENV_DIR:-${CONDA_ROOT}/envs/${CONDA_ENV}}"  # conda 环境完整路径
ISAAC_ASSET_CACHE="${ISAAC_ASSET_CACHE:-/home/ros/isaac_go2/assets_cache}"  # Isaac 缓存目录
ISAACSIM_SITE="${ISAACSIM_SITE:-/home/ros/miniconda3/envs/${CONDA_ENV}/lib/python3.12/site-packages/isaacsim}"  # isaacsim pip 包位置
ASSET_ROOT_URL="${ASSET_ROOT_URL:-https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0}"  # Isaac 资产根 URL
GO2_ASSET_DIR="${ISAAC_ASSET_CACHE}/Assets/Isaac/6.0/Isaac/IsaacLab/Robots/Unitree/Go2"  # Go2 USD 本地缓存目录

unset PYTHONPATH  # 清理外部 Python 路径
unset LD_LIBRARY_PATH  # 清理外部动态库路径
unset CONDA_PREFIX  # 清理 conda 状态
unset CONDA_DEFAULT_ENV  # 清理 conda 环境名
unset CONDA_SHLVL  # 清理 conda 层级
unset CONDA_PROMPT_MODIFIER  # 清理 conda 提示符
export CONDA_NO_PLUGINS=true
export CONDA_SOLVER=classic
export CONDA_PREFIX="${CONDA_ENV_DIR}"
export CONDA_DEFAULT_ENV="${CONDA_ENV}"
export CONDA_SHLVL=1
export CONDA_PROMPT_MODIFIER="(${CONDA_ENV}) "
export PATH="${CONDA_ENV_DIR}/bin:${CONDA_ROOT}/condabin:${PATH}"
export OMNI_KIT_ACCEPT_EULA=Y
export ACCEPT_EULA=Y
export TMPDIR="${ISAAC_ASSET_CACHE}"

mkdir -p "${ISAAC_ASSET_CACHE}" "${GO2_ASSET_DIR}/Props"  # 确保缓存目录存在

if [[ ! -x "${CONDA_ENV_DIR}/bin/python" ]]; then
  echo "未找到 Isaac Lab Python：${CONDA_ENV_DIR}/bin/python" >&2
  exit 1
fi

echo "预缓存 Isaac Lab headless 运行扩展..."
PATH="${CONDA_PREFIX}/bin:${PATH}" isaacsim "${ISAAC_ROOT}/IsaacLab/apps/isaaclab.python.headless.kit" \
  --ext-folder "${ISAACSIM_SITE}/kit/exts" \
  --ext-folder "${ISAACSIM_SITE}/kit/extscore" \
  --ext-folder "${ISAACSIM_SITE}/exts" \
  --ext-folder "${ISAACSIM_SITE}/extscache" \
  --ext-folder "${ISAACSIM_SITE}/extsPhysics" \
  --ext-folder "${ISAACSIM_SITE}/isaacsim/exts" \
  --ext-folder "${ISAACSIM_SITE}/isaacsim/extscache" \
  --ext-folder "${ISAACSIM_SITE}/isaacsim/extsPhysics" \
  --ext-folder "${ISAAC_ROOT}/IsaacLab/source" \
  --ext-precache-mode \
  --no-window  # 只预热扩展缓存，不打开 Isaac 窗口

echo
echo "预缓存 Go2 USD 资产..."
wget --continue --tries=10 --timeout=60 --read-timeout=300 \
  -O "${GO2_ASSET_DIR}/go2.usd" \
  "${ASSET_ROOT_URL}/Isaac/IsaacLab/Robots/Unitree/Go2/go2.usd"  # 下载 Go2 主 USD

wget --continue --tries=10 --timeout=60 --read-timeout=300 \
  -O "${GO2_ASSET_DIR}/Props/instanceable_meshes.usd" \
  "${ASSET_ROOT_URL}/Isaac/IsaacLab/Robots/Unitree/Go2/Props/instanceable_meshes.usd"  # 下载 Go2 mesh 引用 USD

echo
echo "Isaac Lab 扩展和 Go2 资产预缓存完成：${ISAAC_ASSET_CACHE}"
