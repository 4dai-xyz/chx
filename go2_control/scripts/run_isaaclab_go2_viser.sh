#!/usr/bin/env bash
set -euo pipefail

ISAAC_ROOT="${ISAAC_ROOT:-/home/ros/isaac_go2}"  # Isaac 工作区根目录
CONDA_ROOT="${CONDA_ROOT:-/home/ros/miniconda3}"  # Miniconda 安装目录
CONDA_ENV="${CONDA_ENV:-env_isaaclab312}"  # Isaac Lab 专用 conda 环境名
CONDA_ENV_DIR="${CONDA_ENV_DIR:-${CONDA_ROOT}/envs/${CONDA_ENV}}"  # conda 环境完整路径
TASK="${TASK:-Isaac-Velocity-Flat-Unitree-Go2-Play-v0}"  # 默认使用 Play 任务，减少训练随机扰动
NUM_ENVS="${NUM_ENVS:-1}"  # 浏览器观察时默认只开 1 个环境
ISAAC_DEVICE="${ISAAC_DEVICE:-cuda:0}"  # 物理计算设备，用户终端中优先用 GPU
ISAAC_LOCAL_ASSET_ROOT="${ISAAC_LOCAL_ASSET_ROOT:-/home/ros/isaac_go2/assets_cache/Assets/Isaac/6.0}"  # 本地 Isaac 资产根目录

unset PYTHONPATH  # 清理外部 Python 路径，避免 DPVO/ROS 包污染 Isaac
unset LD_LIBRARY_PATH  # 清理动态库路径，后面只放 WSL CUDA 库
unset CONDA_PREFIX  # 清理当前 shell 的 conda 状态
unset CONDA_DEFAULT_ENV  # 清理当前 shell 的 conda 环境名
unset CONDA_SHLVL  # 清理 conda 嵌套层级
unset CONDA_PROMPT_MODIFIER  # 清理 conda 提示符变量

export CONDA_NO_PLUGINS=true  # 禁用 conda 插件，避免 zstandard/libmamba 报错干扰
export CONDA_SOLVER=classic  # 使用 classic solver，和本项目安装脚本一致
export CONDA_PREFIX="${CONDA_ENV_DIR}"  # 手动指定 Isaac conda 前缀
export CONDA_DEFAULT_ENV="${CONDA_ENV}"  # 手动指定 conda 环境名
export CONDA_SHLVL=1  # 让部分依赖认为当前处在 conda 环境中
export PATH="${CONDA_ENV_DIR}/bin:${CONDA_ROOT}/condabin:${PATH}"  # 优先使用 Isaac 环境里的 python
export LD_LIBRARY_PATH="/usr/lib/wsl/lib"  # WSL 中 CUDA 驱动库位置，供 Newton/Warp 查找
export ISAAC_LOCAL_ASSET_ROOT  # 传给 Python 脚本，用于重定向 Isaac 资产路径
export OMNI_KIT_ACCEPT_EULA=Y  # 自动接受 Isaac/Omniverse EULA
export ACCEPT_EULA=Y  # 自动接受 Isaac/Omniverse EULA

if [[ ! -x "${CONDA_ENV_DIR}/bin/python" ]]; then
  echo "未找到 Isaac Lab Python：${CONDA_ENV_DIR}/bin/python" >&2
  exit 1
fi

echo "Isaac Lab WSL 浏览器模式："
echo "  TASK=${TASK}"
echo "  NUM_ENVS=${NUM_ENVS}"
echo "  ISAAC_DEVICE=${ISAAC_DEVICE}"
echo "  VISUALIZER=viser"
echo "  PHYSICS=newton_mjwarp"
echo
echo "启动成功后，请在 Windows 浏览器打开终端打印的 Viser URL。"
echo "默认地址通常是：http://localhost:8080"
echo "如果 3-5 分钟内没有 URL 或页面没有响应，请按 Ctrl+C 结束。"

cd "${ISAAC_ROOT}/IsaacLab"  # Isaac Lab 脚本必须在源码根目录附近运行
"${CONDA_ENV_DIR}/bin/python" \
  /home/ros/unitree_dev/scripts/isaaclab_go2_viser.py \
  --task "${TASK}" \
  --num_envs "${NUM_ENVS}" \
  --device "${ISAAC_DEVICE}" \
  --visualizer viser \
  presets=newton_mjwarp  # 使用 Newton/MJWarp + Viser，绕开 WSL 中的 Kit/RTX 窗口限制
