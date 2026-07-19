#!/usr/bin/env bash
# Start ONLY the micro-ROS agent (UDP 8888) — use this to verify ESP32 link.
set -e

ROS_DISTRO="${ROS_DISTRO:-humble}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MICROROS_WS="${MICROROS_WS:-${HOME}/microros_ws}"
PORT="${PORT:-8888}"

if [[ -f "${REPO_ROOT}/.local/microros.env" ]]; then
  # shellcheck source=/dev/null
  source "${REPO_ROOT}/.local/microros.env"
fi

# Do not use nounset: ROS setup.bash trips on optional AMENT_* vars
# shellcheck source=/dev/null
source "${REPO_ROOT}/scripts/lib/source_ros.sh"
petcam_source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ ! -f "${MICROROS_WS}/install/local_setup.bash" ]]; then
  echo "ERROR: ${MICROROS_WS}/install/local_setup.bash missing"
  echo "Run: ${REPO_ROOT}/scripts/install_microros_agent.sh"
  exit 1
fi
petcam_source "${MICROROS_WS}/install/local_setup.bash"

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
unset ROS_DISCOVERY_SERVER || true

if ! ros2 pkg prefix micro_ros_agent; then
  echo "ERROR: micro_ros_agent not found"
  exit 1
fi

if command -v fuser >/dev/null 2>&1; then
  fuser -k "${PORT}/udp" 2>/dev/null || true
fi

echo "==> Orin IPs — set ESP32 MICROROS_AGENT_IP to the Wi-Fi address:"
ip -4 addr show scope global | sed -n 's/.*inet \([0-9.]*\).*/  \1/p' || true
echo "==> Starting micro_ros_agent udp4 --port ${PORT} -v6"
echo "==> ESP32 Serial should then show: agent is reachable"

exec ros2 run micro_ros_agent micro_ros_agent udp4 --port "${PORT}" -v6
