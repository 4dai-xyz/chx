#!/usr/bin/env bash
# 在后台运行 VGGT -> COLMAP -> gsplat/3DGS 训练，避免远程终端断开后任务被杀掉。

set -euo pipefail

REPO="/home/ros/ros2_orbslam3"
PYTHON="/home/ros/miniconda3/envs/dpvo/bin/python"
RUN_DIR="${REPO}/output/vggt_input_video_show/gsplat_run"
PID_FILE="${RUN_DIR}/gsplat.pid"
LOG_FILE="${RUN_DIR}/gsplat.log"
EXIT_FILE="${RUN_DIR}/gsplat.exit"
SESSION_NAME="vggt_gsplat"

usage() {
  cat <<USAGE
用法：
  $0 start [run_vggt_gsplat_train.py 参数]
  $0 status
  $0 log
  $0 attach
  $0 stop

常用：
  $0 start --steps 3 --result-dir ${REPO}/output/vggt_input_video_show/gsplat_smoke_results
  $0 start --steps 30000 --result-dir ${REPO}/output/vggt_input_video_show/gsplat_results
  $0 log
  $0 attach
USAGE
}

is_running() {
  if command -v tmux >/dev/null 2>&1 && tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
    return 0
  fi
  [[ -f "${PID_FILE}" ]] && kill -0 "$(cat "${PID_FILE}")" 2>/dev/null
}

ACTION="${1:-}"
if [[ -z "${ACTION}" ]]; then
  usage
  exit 1
fi
shift || true

mkdir -p "${RUN_DIR}"

case "${ACTION}" in
  start)
    if is_running; then
      echo "已有 gsplat 后台任务在运行，PID: $(cat "${PID_FILE}")"
      echo "日志: ${LOG_FILE}"
      exit 0
    fi
    echo "启动 gsplat 后台任务..."
    echo "日志: ${LOG_FILE}"
    rm -f "${EXIT_FILE}"
    if command -v tmux >/dev/null 2>&1; then
      tmux new-session -d -s "${SESSION_NAME}" \
        "cd '${REPO}' && echo \"[开始] \$(date '+%F %T')\" >'${LOG_FILE}' && '${PYTHON}' '${REPO}/scripts/run_vggt_gsplat_train.py' $* >>'${LOG_FILE}' 2>&1; code=\$?; echo \"[结束] \$(date '+%F %T') exit=\${code}\" >>'${LOG_FILE}'; echo \${code} >'${EXIT_FILE}'; sleep 5"
      tmux list-panes -t "${SESSION_NAME}" -F '#{pane_pid}' >"${PID_FILE}"
      echo "tmux 会话: ${SESSION_NAME}"
      echo "PID: $(cat "${PID_FILE}")"
    else
      setsid bash -lc "cd '${REPO}' && echo \"[开始] \$(date '+%F %T')\" >'${LOG_FILE}' && '${PYTHON}' '${REPO}/scripts/run_vggt_gsplat_train.py' $* >>'${LOG_FILE}' 2>&1; code=\$?; echo \"[结束] \$(date '+%F %T') exit=\${code}\" >>'${LOG_FILE}'; echo \${code} >'${EXIT_FILE}'" >/dev/null 2>&1 &
      echo "$!" >"${PID_FILE}"
      echo "PID: $(cat "${PID_FILE}")"
    fi
    echo "查看日志: ${REPO}/scripts/run_vggt_gsplat_background.sh log"
    echo "重新接入: ${REPO}/scripts/run_vggt_gsplat_background.sh attach"
    ;;
  status)
    if is_running; then
      echo "运行中，PID: $(cat "${PID_FILE}")"
      if command -v tmux >/dev/null 2>&1 && tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
        tmux list-sessions | grep "${SESSION_NAME}" || true
      else
        ps -p "$(cat "${PID_FILE}")" -o pid,etime,cmd
      fi
    else
      echo "没有正在运行的 gsplat 后台任务。"
      [[ -f "${PID_FILE}" ]] && echo "上次 PID: $(cat "${PID_FILE}")"
      [[ -f "${EXIT_FILE}" ]] && echo "上次退出码: $(cat "${EXIT_FILE}")"
    fi
    echo "日志: ${LOG_FILE}"
    ;;
  log)
    touch "${LOG_FILE}"
    tail -n 120 -f "${LOG_FILE}"
    ;;
  attach)
    if command -v tmux >/dev/null 2>&1 && tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
      tmux attach-session -t "${SESSION_NAME}"
    else
      echo "当前没有可接入的 tmux 会话。"
      echo "可用 $0 log 查看日志。"
      exit 1
    fi
    ;;
  stop)
    if is_running; then
      if command -v tmux >/dev/null 2>&1 && tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
        tmux kill-session -t "${SESSION_NAME}"
        echo "已停止 tmux 会话: ${SESSION_NAME}"
      else
        kill "$(cat "${PID_FILE}")"
        echo "已发送停止信号，PID: $(cat "${PID_FILE}")"
      fi
    else
      echo "没有正在运行的 gsplat 后台任务。"
    fi
    ;;
  *)
    usage
    exit 1
    ;;
esac
