#!/usr/bin/env bash
# Diagnose create_map / micro-ROS link on Orin.
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
[[ -f "${MICROROS_WS}/install/local_setup.bash" ]] && petcam_source "${MICROROS_WS}/install/local_setup.bash"
[[ -f "${REPO_ROOT}/install/local_setup.bash" ]] && petcam_source "${REPO_ROOT}/install/local_setup.bash"

echo "==> nodes"
ros2 node list || true

echo ""
echo "==> topics (imu / create_map)"
ros2 topic list | grep -E 'imu|create_map|parameter' || true

echo ""
echo "==> publishers on /imu/data"
ros2 topic info /imu/data -v 2>/dev/null || echo "(no /imu/data)"

echo ""
echo "==> publishers on /create_map/debug"
ros2 topic info /create_map/debug -v 2>/dev/null || echo "(no /create_map/debug — is imu_odometry running? rebuild?)"

echo ""
echo "==> hz (3s)"
timeout 3 ros2 topic hz /imu/data 2>/dev/null || echo "(no hz on /imu/data)"
timeout 3 ros2 topic hz /create_map/debug 2>/dev/null || echo "(no hz on /create_map/debug)"

echo ""
echo "If debug missing: git pull && ./scripts/build_petcam_ws.sh && ./scripts/run_create_map.sh"
echo "Keep that launch terminal open; echo debug from another terminal after sourcing install."
