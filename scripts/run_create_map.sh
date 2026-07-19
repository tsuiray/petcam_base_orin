#!/usr/bin/env bash
# Launch create_map (discovery server + UDP micro-ROS agent + map).
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

# Optional but recommended for Discovery Server CLI
if ! command -v fastdds >/dev/null 2>&1; then
  echo "==> Installing ros-${ROS_DISTRO}-fastdds-tools (for discovery server)..."
  sudo apt-get update -qq && sudo apt-get install -y "ros-${ROS_DISTRO}-fastdds-tools" || true
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export ROS_DISCOVERY_SERVER="${ROS_DISCOVERY_SERVER:-127.0.0.1:11811}"
export MICROROS_WS

FASTDDS_XML="${REPO_ROOT}/install/create_map/share/create_map/config/fastdds_localhost.xml"
if [[ ! -f "${FASTDDS_XML}" ]]; then
  FASTDDS_XML="${REPO_ROOT}/src/create_map/config/fastdds_localhost.xml"
fi
if [[ -f "${FASTDDS_XML}" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-${FASTDDS_XML}}"
fi

ros2 daemon stop >/dev/null 2>&1 || true

echo "==> ESP32 contract: /imu/data @ UDP:8888"
echo "==> ROS_DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER}"
echo "==> FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS}"
echo "==> FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-}"
echo "==> If /imu/data still missing after ESP32 connects, try:"
echo "      ./scripts/run_create_map_docker_agent.sh"

exec ros2 launch create_map create_map.launch.py "$@"
