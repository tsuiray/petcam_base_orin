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

# Same-host DDS discovery between micro_ros_agent and create_map nodes
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
FASTDDS_XML="${REPO_ROOT}/install/create_map/share/create_map/config/fastdds_localhost.xml"
if [[ ! -f "${FASTDDS_XML}" ]]; then
  FASTDDS_XML="${REPO_ROOT}/src/create_map/config/fastdds_localhost.xml"
fi
if [[ -f "${FASTDDS_XML}" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-${FASTDDS_XML}}"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

echo "==> ROS_DOMAIN_ID=${ROS_DOMAIN_ID} RMW=${RMW_IMPLEMENTATION} LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "==> FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-}"

# Pass through any launch args, e.g. use_mock_imu:=true
exec ros2 launch create_map create_map.launch.py "$@"
