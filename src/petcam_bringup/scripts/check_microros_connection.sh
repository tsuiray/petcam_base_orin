#!/usr/bin/env bash
# Quick health check after starting the micro-ROS agent and powering the ESP32-S3.
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"

# ROS setup.bash trips `set -u` on optional AMENT_* vars
source_ros() {
  set +u
  # shellcheck source=/dev/null
  source "$1"
  set -u
}

if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  source_ros "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

MICROROS_WS="${MICROROS_WS:-${HOME}/microros_ws}"
if [[ -f "${MICROROS_WS}/install/local_setup.bash" ]]; then
  source_ros "${MICROROS_WS}/install/local_setup.bash"
fi

echo "==> micro_ros_agent package:"
if ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then
  ros2 pkg prefix micro_ros_agent
else
  echo "NOT FOUND. Build with scripts/install_microros_agent.sh"
  exit 1
fi

echo ""
echo "==> Serial candidates (ESP32 USB):"
ls -l /dev/petcam_sensing* /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "(none present)"

echo ""
echo "==> ROS 2 nodes (expect micro_ros_agent + ESP32 nodes after link-up):"
ros2 node list || true

echo ""
echo "==> ROS 2 topics:"
ros2 topic list || true

echo ""
echo "If the agent is running but no ESP32 topics appear:"
echo "  1) ESP32 uses UDP — confirm Orin agent is udp4 on the same port (default 8888)"
echo "     and ESP32 points at this Orin IP."
echo "  2) Restart agent with -v6 and watch for 'session established'."
echo "  3) Same ROS_DOMAIN_ID on Orin and client (default 0)."
