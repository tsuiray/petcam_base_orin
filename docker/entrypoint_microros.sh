#!/usr/bin/env bash
set -euo pipefail
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
# shellcheck source=/dev/null
source /microros_ws/install/local_setup.bash
exec ros2 run micro_ros_agent micro_ros_agent "$@"
