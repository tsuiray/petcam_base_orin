#!/usr/bin/env bash
# Start ONLY the micro-ROS agent (UDP 8888) for ESP32 bring-up / debug.
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MICROROS_WS="${MICROROS_WS:-${HOME}/microros_ws}"
PORT="${PORT:-8888}"

if [[ -f "${REPO_ROOT}/.local/microros.env" ]]; then
  # shellcheck source=/dev/null
  source "${REPO_ROOT}/.local/microros.env"
fi

# shellcheck source=/dev/null
source "${REPO_ROOT}/scripts/lib/source_ros.sh"
petcam_source "/opt/ros/${ROS_DISTRO}/setup.bash"
[[ -f "${MICROROS_WS}/install/local_setup.bash" ]] && petcam_source "${MICROROS_WS}/install/local_setup.bash"

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
unset ROS_DISCOVERY_SERVER || true

if command -v fuser >/dev/null 2>&1; then
  fuser -k "${PORT}/udp" 2>/dev/null || true
fi

echo "==> Orin IPs — set ESP32 MICROROS_AGENT_IP to the Wi-Fi one:"
ip -4 addr show scope global | sed -n 's/.*inet \([0-9.]*\).*/  \1/p' || true
echo "==> Starting: micro_ros_agent udp4 --port ${PORT} -v6"
echo "==> Then on ESP32 Serial Monitor expect: agent is reachable"

exec ros2 run micro_ros_agent micro_ros_agent udp4 --port "${PORT}" -v6
