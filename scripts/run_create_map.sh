#!/usr/bin/env bash
# Launch create_map with agent + nodes sharing one FastDDS (microros_ws libs first).
set -e

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

if [[ ! -f "${MICROROS_WS}/install/local_setup.bash" ]]; then
  echo "ERROR: ${MICROROS_WS}/install/local_setup.bash missing"
  echo "Run: ./scripts/install_microros_agent.sh"
  exit 1
fi
petcam_source "${MICROROS_WS}/install/local_setup.bash"

if [[ ! -f "${REPO_ROOT}/install/local_setup.bash" ]]; then
  echo "ERROR: build first: ./scripts/build_petcam_ws.sh"
  exit 1
fi
petcam_source "${REPO_ROOT}/install/local_setup.bash"

if ! ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then
  echo "ERROR: micro_ros_agent not found"
  exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export MICROROS_WS
unset ROS_DISCOVERY_SERVER || true

# Critical: put microros_ws libs FIRST so agent and create_map share FastDDS
export LD_LIBRARY_PATH="${MICROROS_WS}/install/lib:${MICROROS_WS}/install/lib/aarch64-linux-gnu:/opt/ros/${ROS_DISTRO}/lib:/opt/ros/${ROS_DISTRO}/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}"

FASTDDS_XML="${REPO_ROOT}/install/create_map/share/create_map/config/fastdds_localhost.xml"
[[ -f "${FASTDDS_XML}" ]] || FASTDDS_XML="${REPO_ROOT}/src/create_map/config/fastdds_localhost.xml"
[[ -f "${FASTDDS_XML}" ]] && export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_XML}"

ros2 daemon stop >/dev/null 2>&1 || true
command -v fuser >/dev/null && fuser -k 8888/udp 2>/dev/null || true

echo "==> Orin IPs (ESP32 MICROROS_AGENT_IP):"
ip -4 addr show scope global | sed -n 's/.*inet \([0-9.]*\).*/  \1/p' || true
echo "==> LD_LIBRARY_PATH starts with microros_ws (shared FastDDS)"
echo "==> Launch create_map — look for: Receiving /imu/data via QoS=..."

exec ros2 launch create_map create_map.launch.py verbose:=4 "$@"
