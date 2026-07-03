#!/usr/bin/env bash
set -euo pipefail

ISAAC_ROOT="${ISAAC_ROOT:-/home/ros/isaac_go2}"  # Isaac 工作区根目录
CONDA_SH="${CONDA_SH:-/home/ros/miniconda3/etc/profile.d/conda.sh}"  # conda 初始化脚本路径
CONDA_ENV="${CONDA_ENV:-env_isaaclab312}"  # Isaac Lab conda 环境名
CONDA_ROOT="${CONDA_ROOT:-/home/ros/miniconda3}"  # Miniconda 根目录
CONDA_ENV_DIR="${CONDA_ENV_DIR:-${CONDA_ROOT}/envs/${CONDA_ENV}}"  # conda 环境完整路径
TASK="${TASK:-Isaac-Velocity-Flat-Unitree-Go2-v0}"  # 默认 Go2 平地速度任务
NUM_ENVS="${NUM_ENVS:-4}"  # 默认并行环境数，显存小可以改成 1
ISAAC_ASSET_CACHE="${ISAAC_ASSET_CACHE:-/home/ros/isaac_go2/assets_cache}"  # Isaac 缓存目录
AGENT_MODE="${AGENT_MODE:-zero}"  # 未加载 checkpoint 时的动作模式：zero 或 random
USE_PRETRAINED_CHECKPOINT="${USE_PRETRAINED_CHECKPOINT:-0}"  # 是否尝试使用 Isaac Lab 内置预训练模型
DISABLE_FABRIC="${DISABLE_FABRIC:-0}"  # 是否关闭 Fabric，通常保持 0
ISAAC_DEVICE="${ISAAC_DEVICE:-cuda:0}"  # 仿真/策略运行设备
HEADLESS="${HEADLESS:-0}"  # 是否无窗口运行
ENABLE_CAMERAS="${ENABLE_CAMERAS:-0}"  # 是否启用相机传感器
ALLOW_UNSUPPORTED_WSL_GUI="${ALLOW_UNSUPPORTED_WSL_GUI:-0}"  # 是否强行在 WSL 中尝试 Isaac 标准窗口

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
LD_PATHS="${CONDA_ENV_DIR}/lib"  # 原生 Ubuntu 只需要 conda 库路径
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

if grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null \
  && [[ "${HEADLESS}" != "1" ]] \
  && [[ "${ALLOW_UNSUPPORTED_WSL_GUI}" != "1" ]]; then
  cat >&2 <<'EOF'
检测到当前系统是 WSL2，并且请求启动 Isaac Sim 图形窗口。

NVIDIA 官方不支持在 WSL2 中运行 Isaac Sim Python/Kit 图形程序。PyTorch 能识别
CUDA 只代表 CUDA 计算可用，不代表 Isaac 的 RTX/Vulkan 渲染器能创建显卡设备。
典型现象是日志显示 “No device could be created”，随后仿真循环继续运行，但没有窗口。

可选方案：
  1. 当前 WSL 中使用 MuJoCo 做实时可视化。
  2. 在原生 Windows 11 或原生 Ubuntu 22.04/24.04 安装 Isaac Sim/Lab。
  3. 使用满足配置的远程 Linux GPU 机器运行 Isaac。

如果只是明确知道风险并想继续实验，可临时设置：
  ALLOW_UNSUPPORTED_WSL_GUI=1

注意：这不会修复 WSL 的 RTX/Vulkan 限制，通常仍然不会出现窗口。
EOF
  exit 2  # 在 WSL 中默认阻止 Isaac 标准窗口，避免用户误以为卡住
fi

cd "${ISAAC_ROOT}/IsaacLab"  # 进入 IsaacLab 源码根目录
echo "Isaac Lab 启动参数：TASK=${TASK}, NUM_ENVS=${NUM_ENVS}, ISAAC_DEVICE=${ISAAC_DEVICE}, DISABLE_FABRIC=${DISABLE_FABRIC}, HEADLESS=${HEADLESS}"

if [[ -n "${CHECKPOINT:-}" || "${USE_PRETRAINED_CHECKPOINT}" == "1" ]]; then
  args=(--task "${TASK}" --num_envs "${NUM_ENVS}" --device "${ISAAC_DEVICE}")  # play.py 基础参数
  if [[ "${HEADLESS}" == "1" ]]; then
    args+=(--headless)  # 无窗口播放
  fi
  if [[ "${ENABLE_CAMERAS}" == "1" ]]; then
    args+=(--enable_cameras)  # 启用相机传感器
  fi
  if [[ "${DISABLE_FABRIC}" == "1" ]]; then
    args+=(--disable_fabric)  # 兼容部分不支持 Fabric 的情况
  fi
  if [[ "${USE_PRETRAINED_CHECKPOINT}" == "1" ]]; then
    args+=(--use_pretrained_checkpoint)  # 使用任务配置中声明的预训练模型
  fi
  if [[ -n "${CHECKPOINT:-}" ]]; then
    args+=(--checkpoint "${CHECKPOINT}")  # 指定本地策略模型文件
  fi
  if [[ -n "${LOAD_RUN:-}" ]]; then
    args+=(--load_run "${LOAD_RUN}")  # 指定 logs/rsl_rl 下的训练运行目录
  fi

  ./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py "${args[@]}"  # 加载策略并播放
  exit 0
fi

case "${AGENT_MODE}" in
  zero)
    echo "未指定 checkpoint，使用 zero_agent 打开 Go2 Isaac 场景。"
    echo "这一步只用于看仿真画面，不代表已经加载训练好的行走策略。"
    args=(--task "${TASK}" --num_envs "${NUM_ENVS}" --device "${ISAAC_DEVICE}")  # zero_agent 基础参数
    if [[ "${HEADLESS}" == "1" ]]; then
      args+=(--headless)
    fi
    if [[ "${ENABLE_CAMERAS}" == "1" ]]; then
      args+=(--enable_cameras)
    fi
    if [[ "${DISABLE_FABRIC}" == "1" ]]; then
      args+=(--disable_fabric)
    fi
    ./isaaclab.sh -p scripts/environments/zero_agent.py \
      "${args[@]}"  # 只发送零动作，适合测试场景加载，不代表会稳定行走
    ;;
  random)
    echo "未指定 checkpoint，使用 random_agent 打开 Go2 Isaac 场景。"
    echo "随机动作可能让机器人抖动或摔倒，只适合测试画面和环境。"
    args=(--task "${TASK}" --num_envs "${NUM_ENVS}" --device "${ISAAC_DEVICE}")  # random_agent 基础参数
    if [[ "${HEADLESS}" == "1" ]]; then
      args+=(--headless)
    fi
    if [[ "${ENABLE_CAMERAS}" == "1" ]]; then
      args+=(--enable_cameras)
    fi
    if [[ "${DISABLE_FABRIC}" == "1" ]]; then
      args+=(--disable_fabric)
    fi
    ./isaaclab.sh -p scripts/environments/random_agent.py \
      "${args[@]}"  # 发送随机动作，适合测试动作通道，机器人可能抖动或摔倒
    ;;
  *)
    echo "未知 AGENT_MODE=${AGENT_MODE}，可用值：zero 或 random" >&2
    exit 1
    ;;
esac
