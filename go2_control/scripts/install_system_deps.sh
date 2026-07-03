#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"  # 默认安装 ROS 2 Humble 对应依赖

echo "安装 Go2/Unitree 开发所需系统依赖。"
echo "这一步需要管理员权限；请在普通 WSL Ubuntu 终端中执行。"

if [[ "${EUID}" -eq 0 ]]; then
  SUDO=()  # 当前已经是 root，不需要 sudo
elif id -nG | tr ' ' '\n' | grep -qx sudo; then
  SUDO=(sudo)  # 当前用户在 sudo 组，后续 apt 使用 sudo
else
  cat >&2 <<'EOF'
当前用户不在 sudo 组，无法安装系统包。

请在 Windows PowerShell 里执行下面命令，进入本 WSL 的 root shell：

  wsl -d Ubuntu-22.04 -u root

然后在 root shell 里执行：

  usermod -aG sudo ros
  exit

最后在 Windows PowerShell 里重启 WSL：

  wsl --shutdown

重新打开 Ubuntu 后，再回到工作区运行：

  cd /home/ros/unitree_dev
  bash scripts/install_system_deps.sh
EOF
  exit 1
fi

"${SUDO[@]}" apt update  # 更新 apt 软件源索引
"${SUDO[@]}" apt install -y \
  build-essential \
  cmake \
  g++ \
  git \
  joystick \
  make \
  mesa-utils \
  python3-colcon-common-extensions \
  python3-pip \
  python3-venv \
  libboost-all-dev \
  libeigen3-dev \
  libfmt-dev \
  libglfw3-dev \
  libspdlog-dev \
  libyaml-cpp-dev \
  "ros-${ROS_DISTRO}-rmw-cyclonedds-cpp" \
  "ros-${ROS_DISTRO}-rosidl-generator-dds-idl"  # 安装 MuJoCo/SDK2/ROS2 构建和运行依赖

echo "系统依赖安装完成。"
