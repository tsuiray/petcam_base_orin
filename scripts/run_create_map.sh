#!/usr/bin/env bash
# Launch create_map. Ensures micro-ROS agent can bind UDP :8888 for ESP32.
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MICROROS_WS="${MICROROS_WS:-${HOME}/microros_ws}"

if [[ -f "${REPO_ROOT}/.local/microros.env" ]]; then
  # shellcheck source=/dev/null
  source "${REPO_ROOT}/.local/microros.env"
fi

# shellcheck source=/dev/null
source "${REPO_ROOT}/scripts/lib/source_ros.sh"
petcam_source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ -f "${MICROROS_WS}/install/local_setup.bash" ]]; then
  petcam_source "${MICROROS_WS}/install/local_setup.bash"
fi

if [[ ! -f "${REPO_ROOT}/install/local_setup.bash" ]]; then
  echo "Workspace not built. Run: ${REPO_ROOT}/scripts/build_petcam_ws.sh"
  exit 1
fi
petcam_source "${REPO_ROOT}/install/local_setup.bash"

if ! ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then
  echo "ERROR: micro_ros_agent not found. Run: ./scripts/install_microros_agent.sh"
  exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export MICROROS_WS
# Critical: Discovery Server mode can prevent agent from binding XRCE :8888
unset ROS_DISCOVERY_SERVER || true

FASTDDS_XML="${REPO_ROOT}/install/create_map/share/create_map/config/fastdds_localhost.xml"
if [[ ! -f "${FASTDDS_XML}" ]]; then
  FASTDDS_XML="${REPO_ROOT}/src/create_map/config/fastdds_localhost.xml"
fi
if [[ -f "${FASTDDS_XML}" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_XML}"
fi

ros2 daemon stop >/dev/null 2>&1 || true

# Free UDP 8888 if a dead agent holds it
if command -v fuser >/dev/null 2>&1; then
  fuser -k 8888/udp 2>/dev/null || true
fi

echo "==> Orin IPs (ESP32 MICROROS_AGENT_IP must match Wi-Fi IP):"
ip -4 addr show scope global | sed -n 's/.*inet \([0-9.]*\).*/  \1/p' || true
echo "==> FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS} (ROS_DISCOVERY_SERVER unset)"
echo "==> Starting create_map (agent + odometry + map)..."

exec ros2 launch create_map create_map.launch.py "$@"
