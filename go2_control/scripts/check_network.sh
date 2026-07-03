#!/usr/bin/env bash
set -u

COMMON_IPS=(  # 常见 Go2 / Unitree / 直连网段地址，用于快速判断网线通信是否通
  192.168.123.18
  192.168.123.161
  192.168.123.99
  10.21.200.1
  10.21.200.254
)

COMMON_PORTS=(22 80 8080 11311 11811)  # 常见 SSH、HTTP、ROS、DDS/调试端口

echo "== 网卡 =="
ip -br addr || true  # 查看 WSL/Ubuntu 当前有哪些网卡和 IP

echo
echo "== 路由 =="
ip route || true  # 查看是否有到 Go2 网段的路由

echo
echo "== 邻居表 =="
ip neigh || true  # 查看二层邻居，能辅助判断网线是否看到设备

echo
echo "== Ping 常见 Unitree / 本地 IP =="
for ip in "${COMMON_IPS[@]}"; do
  if ping -c 1 -W 1 "${ip}" >/dev/null 2>&1; then
    echo "可达:   ${ip}"  # ping 通，说明 IP 层通信大概率可用
  else
    echo "不可达: ${ip}"  # ping 不通，可能是网段、路由、防火墙或机器狗网络模式问题
  fi
done

if command -v nc >/dev/null 2>&1; then
  echo
  echo "== TCP 端口探测 =="
  for ip in "${COMMON_IPS[@]}"; do
    for port in "${COMMON_PORTS[@]}"; do
      if nc -z -w 1 "${ip}" "${port}" >/dev/null 2>&1; then
        echo "开放: ${ip}:${port}"  # 端口开放说明对应服务可能在运行
      fi
    done
  done
fi

echo
echo "== 快速解释 =="
echo "192.168.123.18 可达  -> Go2 SDK2/ROS2 直连网络大概率已通。"
echo "192.168.123.161 可达 -> 更像旧 SDK1 机器人网段，不是本 Go2 主线。"
echo "只有 10.x 可达       -> WSL 还没有进入 Go2 的 192.168.123.0/24 直连网段。"
