#!/usr/bin/env bash
set -euo pipefail

ISAAC_ROOT="${ISAAC_ROOT:-/home/ros/isaac_go2}"  # Isaac 工作区根目录
CONDA_SH="${CONDA_SH:-/home/ros/miniconda3/etc/profile.d/conda.sh}"  # conda 初始化脚本路径
CONDA_ENV="${CONDA_ENV:-env_isaaclab312}"  # Isaac Lab conda 环境名
CONDA_ROOT="${CONDA_ROOT:-/home/ros/miniconda3}"  # Miniconda 根目录
CONDA_ENV_DIR="${CONDA_ENV_DIR:-${CONDA_ROOT}/envs/${CONDA_ENV}}"  # conda 环境完整路径
TASK="${TASK:-Isaac-Velocity-Flat-Unitree-Go2-v0}"  # 默认训练 Go2 平地速度任务
NUM_ENVS="${NUM_ENVS:-32}"  # 并行环境数，显存越大可以越高
MAX_ITERATIONS="${MAX_ITERATIONS:-50}"  # 训练迭代数，小值用于快速冒烟测试
ISAAC_ASSET_CACHE="${ISAAC_ASSET_CACHE:-/home/ros/isaac_go2/assets_cache}"  # Isaac 缓存目录

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
LD_PATHS="${CONDA_ENV_DIR}/lib"  # 原生 Ubuntu 使用 conda 库路径
if [[ -d /usr/lib/wsl/lib ]]; then
  LD_PATHS="/usr/lib/wsl/lib:${LD_PATHS}"  # WSL 中补充 CUDA 驱动库路径
fi
export LD_LIBRARY_PATH="${LD_PATHS}"  # 设置最终动态库路径
export OMNI_KIT_ACCEPT_EULA=Y
export ACCEPT_EULA=Y
export TMPDIR="${ISAAC_ASSET_CACHE}"

mkdir -p "${ISAAC_ASSET_CACHE}"  # 确保缓存目录存在

if [[ ! -x "${CONDA_ENV_DIR}/bin/python" ]]; then
  echo "未找到 Isaac Lab Python：${CONDA_ENV_DIR}/bin/python" >&2
  exit 1
fi

cd "${ISAAC_ROOT}/IsaacLab"  # 进入 IsaacLab 源码根目录
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py \
  --task "${TASK}" \
  --num_envs "${NUM_ENVS}" \
  --max_iterations "${MAX_ITERATIONS}" \
  --headless  # 训练时不打开窗口，节省显存和渲染开销
