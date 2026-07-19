#!/usr/bin/env bash
set -euo pipefail

# ROS setup.bash trips `set -u` on optional AMENT_* vars
source_ros() {
  set +u
  # shellcheck source=/dev/null
  source "$1"
  set -u
}

source_ros /opt/ros/humble/setup.bash
source_ros /microros_ws/install/local_setup.bash
exec ros2 run micro_ros_agent micro_ros_agent "$@"
