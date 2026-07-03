#!/usr/bin/env bash
set -u  # 遇到未定义变量时报错，避免环境变量写错却继续执行。

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"  # ROS2 Humble 环境入口。
WS_SETUP="${WS_SETUP:-/home/ros/unitree_dev/projects/go2_ros2_sdk_ws/install/setup.bash}"  # go2_ros2_sdk 工作区安装入口。

echo "== Go2 ROS2 SDK 环境检查 =="  # 输出检查标题。
echo "ROS_SETUP=${ROS_SETUP}"  # 打印 ROS2 setup 路径，方便排查 source 错误。
echo "WS_SETUP=${WS_SETUP}"  # 打印工作区 setup 路径，方便确认是否已经 build。

if [[ ! -f "${ROS_SETUP}" ]]; then  # 如果 ROS2 setup 不存在。
  echo "MISS ROS2: 找不到 ${ROS_SETUP}"  # 提示 ROS2 没装好或路径不对。
  exit 1  # 没有 ROS2 就无法继续检查。
fi

set +u  # ROS2 setup 内部会读取一些未定义变量，source 时需要临时关闭 set -u。
source "${ROS_SETUP}"  # 加载 ROS2 环境。
set -u  # ROS2 环境加载完成后重新开启未定义变量检查。

if [[ -f "${WS_SETUP}" ]]; then  # 如果工作区已经构建过。
  set +u  # 工作区 setup 同样可能读取未定义变量。
  source "${WS_SETUP}"  # 加载 go2_ros2_sdk 工作区环境。
  set -u  # 工作区环境加载完成后恢复检查。
else
  echo "WARN workspace: 还没有 ${WS_SETUP}，请先 colcon build 基础包。"  # 提示工作区还未构建。
fi

echo
echo "-- ROS2 基础信息 --"  # 输出 ROS2 状态。
echo "ROS_DISTRO=${ROS_DISTRO:-未设置}"  # 打印 ROS2 发行版。
echo "python3=$(which python3) $(python3 --version)"  # 打印 Python 路径和版本。
echo "ros2=$(which ros2)"  # 打印 ros2 命令路径。

echo
echo "-- 已构建的本项目 ROS2 包 --"  # 输出本项目包是否可见。
missing_project_pkgs=0  # 统计本项目 ROS2 包缺失数量。
for pkg in go2_interfaces go2_robot_sdk lidar_processor speech_processor coco_detector lidar_processor_cpp; do  # 逐个检查包。
  if ros2 pkg prefix "${pkg}" >/tmp/go2_pkg_prefix.out 2>/tmp/go2_pkg_prefix.err; then  # 查询包前缀。
    echo "OK   ${pkg}: $(cat /tmp/go2_pkg_prefix.out)"  # 包可见。
  else
    echo "MISS ${pkg}: $(cat /tmp/go2_pkg_prefix.err)"  # 包不可见。
    missing_project_pkgs=$((missing_project_pkgs + 1))  # 缺失计数加一。
  fi
done

echo
echo "-- ROS2 系统依赖 --"  # 输出系统 ROS2 依赖是否存在。
missing_ros_pkgs=0  # 统计 ROS2 系统依赖缺失数量。
for pkg in joy teleop_twist_joy twist_mux nav2_bringup nav2_amcl nav2_map_server slam_toolbox pointcloud_to_laserscan foxglove_bridge pcl_ros; do  # 逐个检查依赖包。
  if ros2 pkg prefix "${pkg}" >/tmp/go2_pkg_prefix.out 2>/tmp/go2_pkg_prefix.err; then  # 查询系统包。
    echo "OK   ${pkg}"  # 已安装。
  else
    echo "MISS ${pkg}"  # 未安装。
    missing_ros_pkgs=$((missing_ros_pkgs + 1))  # 缺失计数加一。
  fi
done

echo
echo "-- Python 运行依赖 --"  # 输出 Python 依赖是否存在。
python_missing_file="/tmp/go2_python_missing_count"  # Python 检查结果临时文件。
rm -f "${python_missing_file}"  # 清理上一次检查结果。
python3 - <<'PY'  # 使用当前 ROS2 Python 检查导入。
mods = ["aiortc", "aiohttp", "cv_bridge", "open3d", "torch", "torchvision", "Crypto", "cv2", "pydub", "numpy"]
missing = 0
for mod in mods:
    try:
        __import__(mod)
        print(f"OK   {mod}")  # 依赖可导入。
    except Exception as exc:
        print(f"MISS {mod}: {type(exc).__name__}: {exc}")  # 依赖缺失或导入失败。
        missing += 1
with open("/tmp/go2_python_missing_count", "w") as f:
    f.write(str(missing))
PY
missing_python_mods="$(cat "${python_missing_file}" 2>/dev/null || echo 0)"  # 读取 Python 缺失数量。
total_missing=$((missing_project_pkgs + missing_ros_pkgs + missing_python_mods))  # 汇总缺失数量。

echo
echo "-- 检查结果汇总 --"  # 输出检查结论。
if [[ "${total_missing}" -eq 0 ]]; then  # 没有发现缺失项。
  echo "OK   Go2 ROS2 SDK 的已构建包、ROS2 系统依赖、Python 运行依赖当前都可见。"  # 明确说明依赖齐全。
else
  echo "MISS 发现 ${total_missing} 个缺失项，请优先补齐上面标记为 MISS 的包或模块。"  # 明确说明有缺失。
fi
echo "NOTE 真实 Go2 运动前务必先低速、空旷、可急停，并确认手机 App 已断开 WebRTC 连接。"  # 实机安全建议。
