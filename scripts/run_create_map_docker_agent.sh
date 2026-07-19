#!/usr/bin/env bash
# Recommended bring-up when native agent DDS discovery fails on Orin:
#   1) Docker micro-ros-agent (host network) — matches ESP32 README
#   2) create_map without starting a second native agent
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UDP_PORT="${UDP_PORT:-8888}"
MICROROS_IMAGE="${MICROROS_IMAGE:-microros/micro-ros-agent:humble}"

echo "==> Stopping leftover agents on :${UDP_PORT} (best effort)"
pkill -f "micro_ros_agent.*udp4" 2>/dev/null || true
docker rm -f petcam_microros_agent 2>/dev/null || true

echo "==> Starting Docker micro-ros-agent (${MICROROS_IMAGE}) udp4:${UDP_PORT}"
echo "    Keep this container running. ESP32 MICROROS_AGENT_IP must be this Orin Wi-Fi IP."
docker run -d --name petcam_microros_agent --net=host --ipc=host \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e ROS_DISCOVERY_SERVER=127.0.0.1:11811 \
  "${MICROROS_IMAGE}" udp4 --port "${UDP_PORT}" -v6

# Discovery server on host for create_map <-> docker agent
if command -v fastdds >/dev/null 2>&1; then
  if ! pgrep -f "fastdds discovery" >/dev/null 2>&1; then
    echo "==> Starting fastdds discovery server on 127.0.0.1:11811"
    fastdds discovery --server-id 0 --ip-address 127.0.0.1 --port 11811 \
      >/tmp/petcam_fastdds_discovery.log 2>&1 &
    sleep 1
  fi
else
  echo "==> WARNING: fastdds CLI missing (sudo apt install ros-${ROS_DISTRO}-fastdds-tools)"
fi

echo "==> Launching create_map WITHOUT native agent"
export ROS_DISCOVERY_SERVER=127.0.0.1:11811
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
exec "${REPO_ROOT}/scripts/run_create_map.sh" start_microros_agent:=false "$@"
