#!/usr/bin/env bash
# Docker micro-ros-agent on :8888 (ESP32 must reach Orin Wi-Fi IP), then create_map.
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UDP_PORT="${UDP_PORT:-8888}"
MICROROS_IMAGE="${MICROROS_IMAGE:-microros/micro-ros-agent:humble}"

echo "==> Orin IPs (ESP32 MICROROS_AGENT_IP = Wi-Fi IP):"
ip -4 addr show scope global | sed -n 's/.*inet \([0-9.]*\).*/  \1/p' || true

echo "==> Restart Docker micro-ros-agent on UDP ${UDP_PORT}"
pkill -f "micro_ros_agent.*udp4" 2>/dev/null || true
docker rm -f petcam_microros_agent 2>/dev/null || true

docker run -d --name petcam_microros_agent --net=host --ipc=host \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  "${MICROROS_IMAGE}" udp4 --port "${UDP_PORT}" -v6

sleep 2
"${REPO_ROOT}/scripts/check_agent_reachable.sh" "${UDP_PORT}"

echo "==> create_map without native agent (docker owns :${UDP_PORT})"
unset ROS_DISCOVERY_SERVER || true
exec "${REPO_ROOT}/scripts/run_create_map.sh" start_microros_agent:=false "$@"
