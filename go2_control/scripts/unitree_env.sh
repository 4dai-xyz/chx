#!/usr/bin/env bash
# 从交互终端 source 本文件：
#   source /home/ros/unitree_dev/scripts/unitree_env.sh
#
# 作用：
# - 清理当前 DPVO/conda 环境对 C++/ROS/DDS 的库路径污染。
# - 加载 ROS 2 Humble。
# - 加载本工作区编译出的 unitree_ros2 接口。
# - 设置 Unitree SDK2、MuJoCo 和 CycloneDDS 常用环境变量。

UNITREE_DEV_ROOT="${UNITREE_DEV_ROOT:-/home/ros/unitree_dev}"  # Go2 开发工作区根目录
ROS_DISTRO="${ROS_DISTRO:-humble}"  # 默认加载 ROS 2 Humble
UNITREE_LOCAL_PREFIX="${UNITREE_LOCAL_PREFIX:-${UNITREE_DEV_ROOT}/opt/unitree_robotics}"  # SDK2 本地安装目录
MUJOCO_HOME="${MUJOCO_HOME:-/home/ros/.mujoco/mujoco-3.3.6}"  # MuJoCo 安装目录

_unitree_path_without_conda_envs() {
  local old_ifs="${IFS}"  # 保存原 IFS，函数结束时恢复
  local item
  local new_path=""  # 重新拼出来的 PATH
  IFS=':'  # PATH 用冒号分隔
  for item in ${PATH:-}; do
    case "${item}" in
      *"/miniconda3/envs/"*) ;;  # 去掉 conda env 下的 bin，避免污染 ROS/C++
      *) new_path="${new_path:+${new_path}:}${item}" ;;  # 保留普通系统路径
    esac
  done
  IFS="${old_ifs}"  # 恢复 IFS
  printf '%s' "${new_path}"  # 输出清理后的 PATH
}

_unitree_ld_without_conda_envs() {
  local old_ifs="${IFS}"  # 保存原 IFS
  local item
  local new_ld=""  # 重新拼出来的 LD_LIBRARY_PATH
  IFS=':'  # LD_LIBRARY_PATH 用冒号分隔
  for item in ${LD_LIBRARY_PATH:-}; do
    case "${item}" in
      *"/miniconda3/envs/"*) ;;  # 去掉 conda env 动态库，避免 libtinfo 等冲突
      *) new_ld="${new_ld:+${new_ld}:}${item}" ;;  # 保留普通系统动态库路径
    esac
  done
  IFS="${old_ifs}"  # 恢复 IFS
  printf '%s' "${new_ld}"  # 输出清理后的 LD_LIBRARY_PATH
}

_unitree_source_setup_bash() {
  local setup_file="$1"  # 要 source 的 setup.bash
  local had_nounset=0  # 记录当前是否开启 set -u
  case "$-" in
    *u*)
      had_nounset=1
      set +u  # ROS setup.bash 内部可能访问未定义变量，临时关闭 nounset
      ;;
  esac

  source "${setup_file}"  # 加载 ROS 或工作区环境

  if [[ "${had_nounset}" == "1" ]]; then
    set -u  # 恢复原来的 nounset 状态
  fi
}

export PATH="$(_unitree_path_without_conda_envs)"  # 清理 PATH 中的 conda env 项
_unitree_clean_ld="$(_unitree_ld_without_conda_envs)"  # 清理 LD_LIBRARY_PATH 中的 conda env 项
if [[ -n "${_unitree_clean_ld}" ]]; then
  export LD_LIBRARY_PATH="${_unitree_clean_ld}"  # 保留清理后的动态库路径
else
  unset LD_LIBRARY_PATH  # 如果清理后为空，就彻底取消该变量
fi
unset _unitree_clean_ld
unset PYTHONPATH  # 清理 Python 包路径，避免 conda/DPVO 污染 ROS
unset CONDA_DEFAULT_ENV  # 清理 conda 环境名
unset CONDA_PREFIX  # 清理 conda 环境前缀

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS 2 ${ROS_DISTRO} was not found at /opt/ros/${ROS_DISTRO}/setup.bash" >&2
  return 1 2>/dev/null || exit 1
fi

_unitree_source_setup_bash "/opt/ros/${ROS_DISTRO}/setup.bash"  # 加载系统 ROS 2 环境

export UNITREE_DEV_ROOT  # 暴露工作区根目录给后续命令使用
export UNITREE_LOCAL_PREFIX  # 暴露 SDK2 安装目录
export MUJOCO_HOME  # 暴露 MuJoCo 安装目录
export CMAKE_PREFIX_PATH="${UNITREE_LOCAL_PREFIX}:${CMAKE_PREFIX_PATH:-}"  # 让 CMake 找到本地 SDK2

if [[ -d "${UNITREE_LOCAL_PREFIX}/lib" ]]; then
  export LD_LIBRARY_PATH="${UNITREE_LOCAL_PREFIX}/lib:${LD_LIBRARY_PATH:-}"  # 让运行时找到 Unitree SDK2 动态库
fi

if [[ -d "${MUJOCO_HOME}/lib" ]]; then
  export LD_LIBRARY_PATH="${MUJOCO_HOME}/lib:${LD_LIBRARY_PATH:-}"  # 让运行时找到 MuJoCo 动态库
fi

_unitree_ros2_setup=""  # 最终找到的 unitree_ros2 setup.bash
for _unitree_candidate in \
  "${UNITREE_DEV_ROOT}/install/unitree_ros2/setup.bash" \
  "${UNITREE_DEV_ROOT}/src/unitree_ros2/cyclonedds_ws/install/setup.bash"; do
  _unitree_candidate_root="$(dirname "${_unitree_candidate}")"  # setup.bash 所在 install 根目录
  if [[ -f "${_unitree_candidate}" && -d "${_unitree_candidate_root}/share/unitree_go" ]]; then
    _unitree_ros2_setup="${_unitree_candidate}"  # 找到包含 unitree_go 包的工作区环境
    break
  fi
done

if [[ -n "${_unitree_ros2_setup}" ]]; then
  _unitree_source_setup_bash "${_unitree_ros2_setup}"  # 加载编译出的 Unitree ROS2 接口
else
  echo "Unitree ROS2 接口尚未完整构建。需要能找到 share/unitree_go。" >&2
  echo "构建命令：bash ${UNITREE_DEV_ROOT}/scripts/build_unitree_ros2.sh" >&2
  echo "检查过的 setup 文件：" >&2
  echo "  ${UNITREE_DEV_ROOT}/install/unitree_ros2/setup.bash" >&2
  echo "  ${UNITREE_DEV_ROOT}/src/unitree_ros2/cyclonedds_ws/install/setup.bash" >&2
fi
unset _unitree_ros2_setup
unset _unitree_candidate
unset _unitree_candidate_root
unset -f _unitree_source_setup_bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # Unitree ROS2 推荐使用 CycloneDDS

if ! find "/opt/ros/${ROS_DISTRO}" -name librmw_cyclonedds_cpp.so -print -quit 2>/dev/null | grep -q .; then
  echo "警告：ros-${ROS_DISTRO}-rmw-cyclonedds-cpp 还没有安装。" >&2
  echo "请运行：bash ${UNITREE_DEV_ROOT}/scripts/install_system_deps.sh" >&2
fi

if [[ -n "${UNITREE_NET_IF:-}" ]]; then
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${UNITREE_NET_IF}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"  # 手动指定 DDS 使用的网卡
else
  unset CYCLONEDDS_URI  # 不指定网卡时交给 CycloneDDS 自动选择
fi

echo "Go2/Unitree 开发环境已加载"
echo "  工作区: ${UNITREE_DEV_ROOT}"
echo "  ROS 2: ${ROS_DISTRO}"
echo "  RMW:   ${RMW_IMPLEMENTATION}"
if [[ -n "${UNITREE_NET_IF:-}" ]]; then
  echo "  网卡:  ${UNITREE_NET_IF}"
else
  echo "  网卡:  CycloneDDS 默认自动选择"
fi
