#!/usr/bin/env bash
# Check whether the Orin micro-ROS agent is reachable for the ESP32.
set -euo pipefail

PORT="${1:-8888}"

echo "==> Orin IPv4 addresses (put Wi-Fi IP into ESP32 MICROROS_AGENT_IP):"
ip -4 addr show scope global | sed -n 's/.*inet \([0-9.]*\).*/  \1/p' || true

echo ""
echo "==> UDP listeners on :${PORT}"
if ss -ulnp 2>/dev/null | grep -E ":${PORT}[[:space:]]"; then
  echo "OK: something is listening on UDP ${PORT}"
else
  echo "FAIL: nothing listening on UDP ${PORT}"
  echo "ESP32 will print: Waiting for micro-ROS agent / not reachable"
  echo "Start agent: ./scripts/run_microros_agent.sh"
  exit 1
fi

echo ""
echo "==> micro_ros_agent processes"
pgrep -af micro_ros_agent || echo "(none)"

echo ""
echo "On ESP32 board_config.local.h:"
echo "  #define MICROROS_AGENT_IP \"<Orin Wi-Fi IP above>\""
echo "  #define MICROROS_AGENT_PORT ${PORT}"
echo "Same Wi-Fi AP as Orin; disable AP client-isolation."
