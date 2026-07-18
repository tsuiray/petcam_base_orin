#!/usr/bin/env bash
# Build PetCam Orin packages in this repository (colcon workspace at repo root).
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MICROROS_WS="${MICROROS_WS:-${HOME}/microros_ws}"

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS 2 ${ROS_DISTRO} not found. Run scripts/install_ros2_humble.sh first."
  exit 1
fi

# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ -f "${MICROROS_WS}/install/local_setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${MICROROS_WS}/install/local_setup.bash"
fi

cd "${REPO_ROOT}"
rosdep install --from-paths src --ignore-src -y --rosdistro "${ROS_DISTRO}" || true
colcon build --symlink-install

# shellcheck source=/dev/null
source "${REPO_ROOT}/install/local_setup.bash"

BASHRC="${HOME}/.bashrc"
MARKER="# >>> petcam base orin >>>"
if ! grep -Fq "${MARKER}" "${BASHRC}" 2>/dev/null; then
  {
    echo ""
    echo "${MARKER}"
    echo "source ${REPO_ROOT}/install/local_setup.bash"
    echo "# <<< petcam base orin <<<"
  } >> "${BASHRC}"
fi

echo "==> petcam workspace built at ${REPO_ROOT}"
echo "    source ${REPO_ROOT}/install/local_setup.bash"
echo "    ros2 launch petcam_bringup microros_agent.launch.py"
