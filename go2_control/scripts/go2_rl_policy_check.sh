#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="${PROJECT_ROOT:-/home/ros/unitree_dev/projects/open_source_deploy_simtoreal_rl_go2}"  # TheoBounac Go2 RL 部署仓库路径
CONDA_ROOT="${CONDA_ROOT:-/home/ros/miniconda3}"  # Miniconda 根目录
CONDA_ENV="${CONDA_ENV:-env_isaaclab312}"  # 默认使用已经装好的 IsaacLab Python 环境
PYTHON_BIN="${PYTHON_BIN:-${CONDA_ROOT}/envs/${CONDA_ENV}/bin/python}"  # 用指定环境里的 Python 加载 TorchScript policy
export PROJECT_ROOT  # 让内嵌 Python 也能读取仓库路径

unset LD_LIBRARY_PATH  # 清理当前 shell 中可能来自 DPVO/其他环境的动态库路径
unset PYTHONPATH  # 清理当前 shell 中可能来自 DPVO/其他环境的 Python 包路径
unset CONDA_PREFIX  # 清理当前 conda 状态，避免混用多个环境
unset CONDA_DEFAULT_ENV  # 清理当前 conda 环境名，避免误导脚本

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "未找到 Python：${PYTHON_BIN}" >&2
  echo "可以通过 PYTHON_BIN=/path/to/python bash scripts/go2_rl_policy_check.sh 指定其他 Python。" >&2
  exit 1
fi

if [[ ! -d "${PROJECT_ROOT}" ]]; then
  echo "未找到 TheoBounac Go2 RL 仓库：${PROJECT_ROOT}" >&2
  exit 1
fi

"${PYTHON_BIN}" - <<'PY'
from pathlib import Path
import os
import torch

project_root = Path(os.environ["PROJECT_ROOT"])  # 本地 Go2 RL 部署仓库路径，由 shell 传入，便于后续迁移目录
policy_dir = project_root / "pre_train"  # 预训练策略文件夹
policy_names = ["policy_rough.pt", "policy_rough_2.pt"]  # 当前仓库提供的两个 Go2 TorchScript 策略

for policy_name in policy_names:
    policy_path = policy_dir / policy_name  # 当前要检查的策略文件路径
    if not policy_path.exists():
        print(f"[缺失] {policy_path}")
        continue

    policy = torch.jit.load(str(policy_path), map_location="cpu")  # 只在 CPU 上加载，避免检查阶段占用 GPU
    obs = torch.zeros(1, 52)  # TheoBounac go2.yaml 中 num_obs=52，构造一帧零观测做接口检查
    with torch.no_grad():
        action = policy(obs)  # 策略应该输出 12 个关节动作，对应 Go2 12 个腿部关节

    print(f"[通过] {policy_name}")
    print(f"  输入观测维度: {tuple(obs.shape)}")
    print(f"  输出动作维度: {tuple(action.shape)}")
    print(f"  动作最小值: {float(action.min()):.6f}")
    print(f"  动作最大值: {float(action.max()):.6f}")
PY
