#!/usr/bin/env bash
# Start ONLY the micro-ROS agent — default TCP 8888 (ESP32 MICROROS_TRANSPORT_TCP).
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MICROROS_WS="${MICROROS_WS:-${HOME}/microros_ws}"
PORT="${PORT:-8888}"
TRANSPORT="${TRANSPORT:-tcp4}"

# shellcheck source=/dev/null
source "${REPO_ROOT}/scripts/lib/source_ros.sh"
petcam_source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ ! -f "${MICROROS_WS}/install/local_setup.bash" ]]; then
  echo "ERROR: ${MICROROS_WS}/install/local_setup.bash missing"
  echo "Run: ./scripts/install_microros_agent.sh"
  exit 1
fi
petcam_source "${MICROROS_WS}/install/local_setup.bash"

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
unset ROS_DISCOVERY_SERVER || true

command -v fuser >/dev/null && fuser -k "${PORT}/tcp" 2>/dev/null || true
command -v fuser >/dev/null && fuser -k "${PORT}/udp" 2>/dev/null || true

echo "==> Orin IPs — set ESP32 MICROROS_AGENT_IP to the Wi-Fi address:"
ip -4 addr show scope global | sed -n 's/.*inet \([0-9.]*\).*/  \1/p' || true
echo "==> Starting micro_ros_agent ${TRANSPORT} --port ${PORT} -v6"
echo "==> ESP32 board_config: MICROROS_TRANSPORT must match (${TRANSPORT})"

exec ros2 run micro_ros_agent micro_ros_agent "${TRANSPORT}" --port "${PORT}" -v6
