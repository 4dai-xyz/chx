#!/usr/bin/env bash
set -e

# Print the ROS2 graph used by the Go2 MuJoCo navigation bridge.

cd /home/ros/unitree_dev
source scripts/go2_ros_env.sh

run_ros2_check() {
  local label="$1"
  shift

  echo
  echo "== ${label} =="
  timeout 8 "$@" || echo "[WARN] command failed or timed out: $*"
}

echo
echo "== matching processes =="
matching_pids="$(pgrep -f 'go2_mujoco_ros2_bridge|cmd_vel_smoke_test|nav2|slam_toolbox' || true)"
if [[ -n "${matching_pids}" ]]; then
  ps -o pid,ppid,etime,stat,cmd -p $(printf '%s\n' "${matching_pids}" | tr '\n' ' ')
else
  echo "[WARN] no matching Go2/Nav2/SLAM process found"
fi

for pid in ${matching_pids}; do
  if [[ -r "/proc/${pid}/environ" ]]; then
    echo
    echo "== env for pid ${pid} =="
    tr '\0' '\n' < "/proc/${pid}/environ" \
      | grep -E '^(ROS_|RMW_|FASTRTPS|CYCLONEDDS|LD_LIBRARY_PATH|PYTHONPATH|CONDA|AMENT|PATH=)' \
      | sort || true
  fi
done

echo
echo "== reset ROS2 daemon =="
timeout 5 ros2 daemon stop || true

run_ros2_check "ROS2 nodes" ros2 node list --no-daemon --spin-time 2

run_ros2_check "ROS2 topics" ros2 topic list --no-daemon --spin-time 2

run_ros2_check "/cmd_vel graph" ros2 topic info /cmd_vel --verbose --no-daemon --spin-time 2

run_ros2_check "/odom graph" ros2 topic info /odom --verbose --no-daemon --spin-time 2

run_ros2_check "/scan graph" ros2 topic info /scan --verbose --no-daemon --spin-time 2
