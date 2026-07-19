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

# Pass through any launch args, e.g. use_mock_imu:=true
exec ros2 launch create_map create_map.launch.py "$@"
