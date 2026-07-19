#!/usr/bin/env bash
# Launch create_map (UDP micro-ROS agent + IMU odometry + map viewer).
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

# --- Match ESP32 firmware (cursor/esp32-arduino-hardening-26d4) ---
# ESP32: WiFi UDP → Orin:8888 → micro_ros_agent → /imu/data
# Gap that broke create_map: agent XRCE OK but DDS SHM blocked ROS2 nodes.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

FASTDDS_XML="${REPO_ROOT}/install/create_map/share/create_map/config/fastdds_localhost.xml"
if [[ ! -f "${FASTDDS_XML}" ]]; then
  FASTDDS_XML="${REPO_ROOT}/src/create_map/config/fastdds_localhost.xml"
fi
if [[ -f "${FASTDDS_XML}" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-${FASTDDS_XML}}"
fi

# Stale daemon graph often hides micro-ROS topics
ros2 daemon stop >/dev/null 2>&1 || true

echo "==> ESP32 contract: /imu/data sensor_msgs/Imu BEST_EFFORT 50Hz UDP:8888"
echo "==> ROS_DOMAIN_ID=${ROS_DOMAIN_ID} RMW=${RMW_IMPLEMENTATION}"
echo "==> FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS} (SHM disabled)"
echo "==> FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-}"
echo "==> Tip: kill any other micro-ros-agent / docker agent on :8888 first"

exec ros2 launch create_map create_map.launch.py "$@"
