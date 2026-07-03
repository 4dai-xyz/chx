#!/usr/bin/env bash
set -euo pipefail

ISAAC_ROOT="${ISAAC_ROOT:-/home/ros/isaac_go2}"  # Isaac 工作区根目录
CONDA_SH="${CONDA_SH:-/home/ros/miniconda3/etc/profile.d/conda.sh}"  # conda 初始化脚本路径
CONDA_ENV="${CONDA_ENV:-env_isaaclab312}"  # Isaac Lab conda 环境名
CONDA_ROOT="${CONDA_ROOT:-/home/ros/miniconda3}"  # Miniconda 根目录
CONDA_ENV_DIR="${CONDA_ENV_DIR:-${CONDA_ROOT}/envs/${CONDA_ENV}}"  # conda 环境完整路径
ISAAC_ASSET_CACHE="${ISAAC_ASSET_CACHE:-/home/ros/isaac_go2/assets_cache}"  # Isaac 资产/临时缓存目录

unset PYTHONPATH  # 清理外部 Python 包路径，避免污染 Isaac 检查
unset LD_LIBRARY_PATH  # 清理外部动态库路径
unset CONDA_PREFIX  # 清理当前 shell 的 conda 状态
unset CONDA_DEFAULT_ENV  # 清理当前 shell 的 conda 环境名
unset CONDA_SHLVL  # 清理 conda 嵌套层级
unset CONDA_PROMPT_MODIFIER  # 清理 conda 提示符
export CONDA_NO_PLUGINS=true  # 禁用 conda 插件，避免插件报错影响脚本
export CONDA_SOLVER=classic  # 使用 classic solver
export CONDA_PREFIX="${CONDA_ENV_DIR}"  # 手动指定 Isaac 环境前缀
export CONDA_DEFAULT_ENV="${CONDA_ENV}"  # 手动指定 Isaac 环境名
export CONDA_SHLVL=1  # 让部分工具识别当前为 conda 环境
export CONDA_PROMPT_MODIFIER="(${CONDA_ENV}) "  # 设置提示符环境名
export PATH="${CONDA_ENV_DIR}/bin:${CONDA_ROOT}/condabin:${PATH}"  # 优先使用 Isaac 环境 python
LD_PATHS="${CONDA_ENV_DIR}/lib"  # 原生 Ubuntu 只需要 conda 环境库
if [[ -d /usr/lib/wsl/lib ]]; then
  LD_PATHS="/usr/lib/wsl/lib:${LD_PATHS}"  # WSL 中额外加入 NVIDIA CUDA 驱动库
fi
export LD_LIBRARY_PATH="${LD_PATHS}"  # 最终动态库搜索路径
export OMNI_KIT_ACCEPT_EULA=Y  # 自动接受 Omniverse/Isaac EULA
export ACCEPT_EULA=Y  # 自动接受 Omniverse/Isaac EULA
export TMPDIR="${ISAAC_ASSET_CACHE}"  # 把临时文件放进 Isaac 缓存目录

mkdir -p "${ISAAC_ASSET_CACHE}"  # 确保缓存目录存在

if [[ ! -x "${CONDA_ENV_DIR}/bin/python" ]]; then
  echo "未找到 Isaac Lab Python：${CONDA_ENV_DIR}/bin/python" >&2
  exit 1
fi

echo "Python:"
python --version

echo
echo "Platform compatibility:"
if grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null; then
  echo "platform: WSL2"  # WSL 只能说明 CUDA 计算可能可用
  echo "cuda_compute: may work"
  echo "isaac_gui_rtx_vulkan: unsupported by NVIDIA in WSL2"
  echo "result: Python/CUDA checks below do not prove that an Isaac window can open"
else
  echo "platform: native Linux"  # 原生 Linux 才适合标准 Isaac Kit/RTX 窗口
fi

echo
echo "Torch/CUDA:"
python - <<'PY'
import torch
print("torch:", torch.__version__)
print("cuda_available:", torch.cuda.is_available())
print("cuda_device_count:", torch.cuda.device_count())
if torch.cuda.is_available():
    print("device:", torch.cuda.get_device_name(0))
PY

echo
echo "Isaac Sim import:"
python - <<'PY'
import isaacsim
print("isaacsim import ok")
PY

echo
echo "Registered Go2 tasks:"
cd "${ISAAC_ROOT}/IsaacLab"  # 进入 IsaacLab 源码根目录，确保任务包可导入
python - <<'PY'
import gymnasium as gym
import isaaclab_tasks  # noqa: F401

tasks = sorted(spec.id for spec in gym.registry.values() if "Go2" in spec.id)
for task in tasks:
    print(task)
print("count:", len(tasks))
PY

echo
if [[ "${RUN_ISAAC_APP_SMOKE:-0}" == "1" ]]; then
  echo "Isaac Lab app smoke test:"
  ./isaaclab.sh -p scripts/environments/list_envs.py --keyword Go2  # 可选启动 Isaac App 级别检查
else
  echo "Isaac Lab app smoke test: skipped"
  echo "需要启动 Isaac Sim/Kit 级别检查时再运行："
  echo "  RUN_ISAAC_APP_SMOKE=1 bash scripts/isaaclab_check.sh"
fi
