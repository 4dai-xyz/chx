#!/usr/bin/env bash
set -euo pipefail

ISAAC_ROOT="${ISAAC_ROOT:-/home/ros/isaac_go2}"  # Isaac 工作区根目录
CONDA_SH="${CONDA_SH:-/home/ros/miniconda3/etc/profile.d/conda.sh}"  # conda 初始化脚本路径
CONDA_ENV="${CONDA_ENV:-env_isaaclab312}"  # Isaac Lab conda 环境名
PYTHON_VERSION="${PYTHON_VERSION:-3.12}"  # Isaac Lab 3 使用 Python 3.12
ISAAC_LAB_BRANCH="${ISAAC_LAB_BRANCH:-release/3.0.0-beta2}"  # 固定 IsaacLab 分支，保证环境可复现
ISAAC_LAB_TARBALL="${ISAAC_LAB_TARBALL:-${ISAAC_ROOT}/IsaacLab-${ISAAC_LAB_BRANCH//\//_}.tar.gz}"  # 源码压缩包缓存
ISAAC_LAB_GIT_TIMEOUT="${ISAAC_LAB_GIT_TIMEOUT:-1800}"  # git clone 最长等待时间
ISAAC_LAB_CLONE_MODE="${ISAAC_LAB_CLONE_MODE:-git}"  # 默认完整 git clone，失败后用源码包

unset PYTHONPATH  # 清理外部 Python 路径
unset LD_LIBRARY_PATH  # 清理外部动态库路径
unset CONDA_PREFIX  # 清理 conda 状态
unset CONDA_DEFAULT_ENV  # 清理 conda 环境名
unset CONDA_SHLVL  # 清理 conda 层级
unset CONDA_PROMPT_MODIFIER  # 清理 conda 提示符
export CONDA_NO_PLUGINS=true  # 禁用 conda 插件，减少 libmamba/zstandard 影响
export CONDA_SOLVER=classic  # 使用 classic solver
export PIP_PROGRESS_BAR=off  # 关闭 pip 进度条，日志更干净
export PIP_DEFAULT_TIMEOUT=120  # 增加 pip 网络超时
export PIP_RETRIES=10  # 增加 pip 重试次数
export OMNI_KIT_ACCEPT_EULA=Y  # 自动接受 Isaac/Omniverse EULA
export ACCEPT_EULA=Y  # 自动接受 Isaac/Omniverse EULA

move_path_aside() {
  local path="$1"  # 要备份移走的路径
  local suffix

  if [[ -e "${path}" ]]; then
    suffix="$(date +%Y%m%d_%H%M%S)"  # 用时间戳避免覆盖旧备份
    mv "${path}" "${path}.incomplete.${suffix}"  # 不直接删除，保留现场方便排查
  fi
}

download_isaaclab_tarball() {
  local url="$1"  # 源码包下载地址
  local tmp_tarball="${ISAAC_LAB_TARBALL}.download"  # 下载中的临时文件

  echo "下载 IsaacLab 源码包：${url}"
  rm -f "${tmp_tarball}"

  if command -v wget >/dev/null 2>&1; then
    wget --tries=20 --timeout=60 --read-timeout=300 --no-verbose \
      -O "${tmp_tarball}" \
      "${url}"
  else
    curl -L --retry 20 --connect-timeout 30 --max-time 3600 \
      -o "${tmp_tarball}" \
      "${url}"
  fi

  echo "校验 IsaacLab 源码包完整性..."
  if tar -tzf "${tmp_tarball}" >/dev/null; then
    mv "${tmp_tarball}" "${ISAAC_LAB_TARBALL}"  # 校验通过后才替换正式缓存
    return 0
  fi

  echo "源码包不完整，保留失败文件：${tmp_tarball}" >&2
  return 1
}

ensure_isaaclab_source() {
  local archive_dir="IsaacLab-${ISAAC_LAB_BRANCH//\//-}"  # GitHub tar.gz 解压后的目录名
  local github_archive_url="https://github.com/isaac-sim/IsaacLab/archive/refs/heads/${ISAAC_LAB_BRANCH}.tar.gz"  # GitHub archive 地址
  local codeload_url="https://codeload.github.com/isaac-sim/IsaacLab/tar.gz/refs/heads/${ISAAC_LAB_BRANCH}"  # codeload 备用地址

  cd "${ISAAC_ROOT}"  # IsaacLab 源码放在 ISAAC_ROOT 下

  if [[ -f IsaacLab/isaaclab.sh ]]; then
    echo "已找到 IsaacLab：${ISAAC_ROOT}/IsaacLab"
    if [[ -d IsaacLab/.git ]]; then
      git -C IsaacLab fetch --all --tags || true  # 如果是 git 仓库，尝试更新远端信息
      git -C IsaacLab checkout "${ISAAC_LAB_BRANCH}" || true  # 尝试切到指定分支
    fi
    return 0
  fi

  move_path_aside IsaacLab  # 发现不完整源码时先移走

  if [[ "${ISAAC_LAB_CLONE_MODE}" == "git" ]]; then
    echo "尝试完整克隆 IsaacLab 分支：${ISAAC_LAB_BRANCH}"
    if timeout "${ISAAC_LAB_GIT_TIMEOUT}" git clone --branch "${ISAAC_LAB_BRANCH}" https://github.com/isaac-sim/IsaacLab.git; then
      return 0
    fi

    echo "完整 git clone 未成功，改用源码压缩包继续完成安装。"
    move_path_aside IsaacLab
  fi

  if [[ -s "${ISAAC_LAB_TARBALL}" ]]; then
    echo "检查已有源码包缓存：${ISAAC_LAB_TARBALL}"
    if ! tar -tzf "${ISAAC_LAB_TARBALL}" >/dev/null; then
      move_path_aside "${ISAAC_LAB_TARBALL}"
    fi
  fi

  if [[ ! -s "${ISAAC_LAB_TARBALL}" ]]; then
    download_isaaclab_tarball "${github_archive_url}" || download_isaaclab_tarball "${codeload_url}"
  fi

  move_path_aside "${archive_dir}"
  tar -xzf "${ISAAC_LAB_TARBALL}"  # 解压源码包
  mv "${archive_dir}" IsaacLab  # 统一源码目录名
}

if [[ ! -f "${CONDA_SH}" ]]; then
  echo "未找到 conda 初始化脚本：${CONDA_SH}" >&2
  exit 1
fi

mkdir -p "${ISAAC_ROOT}"  # 确保 Isaac 工作区目录存在

# shellcheck disable=SC1090
source "${CONDA_SH}"  # 加载 conda 命令

if ! conda env list | awk '{print $1}' | grep -qx "${CONDA_ENV}"; then
  conda create -y --solver classic -n "${CONDA_ENV}" "python=${PYTHON_VERSION}"  # 不存在就创建 Isaac 环境
fi

conda activate "${CONDA_ENV}"  # 进入 Isaac 环境
python -m pip install --progress-bar off --upgrade pip  # 升级 pip，减少安装兼容问题

ensure_isaaclab_source  # 确保 IsaacLab 源码存在

cd "${ISAAC_ROOT}/IsaacLab"  # 进入 IsaacLab 源码根目录
./isaaclab.sh --install "isaacsim,rl[rsl-rl]"  # 安装 Isaac Sim 和 rsl_rl 强化学习依赖

echo
echo "Isaac Sim/Lab 安装步骤完成。"
echo "下一步验证："
echo "  bash /home/ros/unitree_dev/scripts/isaaclab_check.sh"
