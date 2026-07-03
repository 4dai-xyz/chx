#!/usr/bin/env bash
# Source this file from Go2 ROS2 scripts to keep DDS discovery consistent.

_go2_ros_env_had_nounset=0
case "$-" in
  *u*)
    _go2_ros_env_had_nounset=1
    set +u
    ;;
esac

unset LD_LIBRARY_PATH
unset PYTHONPATH
unset CONDA_PREFIX
unset CONDA_DEFAULT_ENV

source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID="${GO2_ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${GO2_ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${GO2_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [[ "${RMW_IMPLEMENTATION}" == "rmw_cyclonedds_cpp" ]]; then
  GO2_CYCLONEDDS_INTERFACE="${GO2_CYCLONEDDS_INTERFACE:-lo}"
  GO2_CYCLONEDDS_MAX_AUTO_INDEX="${GO2_CYCLONEDDS_MAX_AUTO_INDEX:-120}"
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${GO2_CYCLONEDDS_INTERFACE}\" priority=\"default\" multicast=\"default\" /></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>${GO2_CYCLONEDDS_MAX_AUTO_INDEX}</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>"
else
  unset CYCLONEDDS_URI
fi
unset ROS_DISCOVERY_SERVER
unset FASTRTPS_DEFAULT_PROFILES_FILE

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs_unitree_dev}"
mkdir -p "${ROS_LOG_DIR}"

echo "Go2 ROS2 env: ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}, RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
if [[ -n "${CYCLONEDDS_URI:-}" ]]; then
  echo "Go2 ROS2 env: CYCLONEDDS interface=${GO2_CYCLONEDDS_INTERFACE}"
  echo "Go2 ROS2 env: CYCLONEDDS max_auto_participant_index=${GO2_CYCLONEDDS_MAX_AUTO_INDEX}"
fi

if [[ "${_go2_ros_env_had_nounset}" == "1" ]]; then
  set -u
fi
unset _go2_ros_env_had_nounset
