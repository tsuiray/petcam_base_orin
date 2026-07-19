#!/usr/bin/env bash
# Build micro-ROS Agent (Humble) for the PetCam Orin base station.
# Prereq: ROS 2 Humble installed (scripts/install_ros2_humble.sh).
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
# Default: sibling workspace next to this repo, or override with MICROROS_WS
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MICROROS_WS="${MICROROS_WS:-${HOME}/microros_ws}"

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS 2 ${ROS_DISTRO} not found at /opt/ros/${ROS_DISTRO}."
  echo "Run scripts/install_ros2_humble.sh first."
  exit 1
fi

# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"

echo "==> micro-ROS workspace: ${MICROROS_WS}"
mkdir -p "${MICROROS_WS}/src"
cd "${MICROROS_WS}"

if [[ ! -d src/micro_ros_setup ]]; then
  echo "==> Cloning micro_ros_setup (${ROS_DISTRO})"
  git clone -b "${ROS_DISTRO}" https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
else
  echo "==> micro_ros_setup already present; updating"
  git -C src/micro_ros_setup fetch origin
  git -C src/micro_ros_setup checkout "${ROS_DISTRO}"
  git -C src/micro_ros_setup pull --ff-only || true
fi

echo "==> Installing dependencies"
sudo apt-get update
sudo apt-get install -y python3-pip libspdlog-dev libfmt-dev
rosdep update
rosdep install --from-paths src --ignore-src -y

echo "==> Building micro_ros_setup"
colcon build --packages-select micro_ros_setup
# shellcheck source=/dev/null
source "${MICROROS_WS}/install/local_setup.bash"

if [[ ! -d src/micro_ros_msgs ]] || [[ ! -d src/uros ]]; then
  echo "==> Creating micro-ROS agent workspace contents"
  ros2 run micro_ros_setup create_agent_ws.sh
else
  echo "==> Agent workspace sources already present; skipping create_agent_ws.sh"
fi

echo "==> Building micro-ROS agent"
# Jetson-friendly flags: prefer system logger libs when available
ros2 run micro_ros_setup build_agent.sh
# shellcheck source=/dev/null
source "${MICROROS_WS}/install/local_setup.bash"

if ! ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then
  echo "ERROR: micro_ros_agent package not found after build."
  exit 1
fi

BASHRC="${HOME}/.bashrc"
MARKER="# >>> petcam microros agent >>>"
if ! grep -Fq "${MARKER}" "${BASHRC}" 2>/dev/null; then
  {
    echo ""
    echo "${MARKER}"
    echo "source ${MICROROS_WS}/install/local_setup.bash"
    echo "# <<< petcam microros agent <<<"
  } >> "${BASHRC}"
  echo "==> Appended micro-ROS agent source to ${BASHRC}"
fi

# Persist path for bringup helpers
mkdir -p "${REPO_ROOT}/.local"
echo "MICROROS_WS=${MICROROS_WS}" > "${REPO_ROOT}/.local/microros.env"

echo ""
echo "==> micro-ROS agent ready."
echo "    Source: source /opt/ros/${ROS_DISTRO}/setup.bash && source ${MICROROS_WS}/install/local_setup.bash"
echo "    Launch (UDP — ESP32-S3 default):"
echo "      ros2 launch petcam_bringup microros_agent.launch.py transport:=udp4 port:=8888"
echo "    Launch (serial USB fallback):"
echo "      ros2 launch petcam_bringup microros_agent.launch.py transport:=serial"
echo "    Or quick test:"
echo "      ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6"
echo "      ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v6"
