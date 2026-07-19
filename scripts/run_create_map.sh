#!/usr/bin/env bash
# Launch create_map with XRCE→ROS IMU bridge (works even when agent DDS is invisible).
set -e

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

if ! ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then
  echo "ERROR: micro_ros_agent missing — ./scripts/install_microros_agent.sh"
  exit 1
fi
if ! ros2 pkg prefix create_map >/dev/null 2>&1; then
  echo "ERROR: create_map missing — ./scripts/build_petcam_ws.sh"
  exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export MICROROS_WS
unset ROS_DISCOVERY_SERVER || true

ros2 daemon stop >/dev/null 2>&1 || true
command -v fuser >/dev/null && fuser -k 8888/udp 2>/dev/null || true
command -v fuser >/dev/null && fuser -k 8887/udp 2>/dev/null || true

echo "==> Orin Wi-Fi IP (ESP32 MICROROS_AGENT_IP / still port 8888):"
ip -4 addr show scope global | sed -n 's/.*inet \([0-9.]*\).*/  \1/p' || true
echo "==> Bridge mode: ESP32 → :8888 → agent :8887 + publish /imu/data"
echo "==> Expect log: Published /imu/data ... then Receiving /imu/data via QoS=..."

exec ros2 launch create_map create_map.launch.py verbose:=4 "$@"
