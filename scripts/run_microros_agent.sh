#!/usr/bin/env bash
# Convenience wrapper: source ROS + micro-ROS + petcam, then launch the agent.
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MICROROS_WS="${MICROROS_WS:-${HOME}/microros_ws}"

# Optional overrides from env file written by install_microros_agent.sh
if [[ -f "${REPO_ROOT}/.local/microros.env" ]]; then
  # shellcheck source=/dev/null
  source "${REPO_ROOT}/.local/microros.env"
fi

# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ -f "${MICROROS_WS}/install/local_setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${MICROROS_WS}/install/local_setup.bash"
else
  echo "micro-ROS agent not built. Run: ${REPO_ROOT}/scripts/install_microros_agent.sh"
  exit 1
fi

if [[ -f "${REPO_ROOT}/install/local_setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${REPO_ROOT}/install/local_setup.bash"
else
  echo "petcam workspace not built. Run: ${REPO_ROOT}/scripts/build_petcam_ws.sh"
  exit 1
fi

TRANSPORT="${TRANSPORT:-serial}"
SERIAL_DEV="${SERIAL_DEV:-/dev/ttyACM0}"
SERIAL_BAUD="${SERIAL_BAUD:-115200}"
PORT="${PORT:-8888}"
VERBOSE="${VERBOSE:-6}"

# Prefer stable udev symlink when present
if [[ "${TRANSPORT}" == "serial" && -e /dev/petcam_sensing ]]; then
  SERIAL_DEV="/dev/petcam_sensing"
fi

exec ros2 launch petcam_bringup microros_agent.launch.py \
  transport:="${TRANSPORT}" \
  serial_dev:="${SERIAL_DEV}" \
  serial_baud:="${SERIAL_BAUD}" \
  port:="${PORT}" \
  verbose:="${VERBOSE}"
