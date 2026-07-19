"""Launch micro-ROS Agent — prioritize XRCE UDP bind for ESP32 reachability."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _build_agent(context, *args, **kwargs):
    transport = LaunchConfiguration("transport").perform(context).strip().lower()
    serial_dev = LaunchConfiguration("serial_dev").perform(context)
    serial_baud = LaunchConfiguration("serial_baud").perform(context)
    port = LaunchConfiguration("port").perform(context)
    verbose = LaunchConfiguration("verbose").perform(context)

    if transport == "serial":
        agent_cli = f"serial --dev {serial_dev} -b {serial_baud} -v{verbose}"
        summary = f"serial {serial_dev} @ {serial_baud}"
    elif transport in ("udp4", "udp"):
        agent_cli = f"udp4 --port {port} -v{verbose}"
        summary = f"udp4 port {port} (0.0.0.0)"
    elif transport in ("tcp4", "tcp"):
        agent_cli = f"tcp4 --port {port} -v{verbose}"
        summary = f"tcp4 port {port}"
    else:
        raise RuntimeError(
            f"Unsupported transport '{transport}'. Use serial, udp4, or tcp4."
        )

    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    microros_ws = os.environ.get("MICROROS_WS", os.path.expanduser("~/microros_ws"))
    fastdds_xml = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", "")

    # IMPORTANT: do NOT require ROS_DISCOVERY_SERVER here.
    # If DDS client mode cannot reach a discovery server, agent init can fail
    # and never bind XRCE :8888 → ESP32 reports "agent not reachable".
    cmd = f"""
set -euo pipefail
source /opt/ros/{ros_distro}/setup.bash
if [ -f "{microros_ws}/install/local_setup.bash" ]; then
  source "{microros_ws}/install/local_setup.bash"
fi
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
unset ROS_DISCOVERY_SERVER || true
if [ -n "{fastdds_xml}" ] && [ -f "{fastdds_xml}" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="{fastdds_xml}"
fi
echo "[petcam] Starting micro_ros_agent {summary}"
echo "[petcam] MICROROS_WS={microros_ws}"
if ! ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then
  echo "[petcam] ERROR: micro_ros_agent not found. Run scripts/install_microros_agent.sh" >&2
  exit 1
fi
# Free stale listener on the XRCE port (UDP)
if command -v fuser >/dev/null 2>&1; then
  fuser -k {port}/udp 2>/dev/null || true
fi
exec ros2 run micro_ros_agent micro_ros_agent {agent_cli}
"""

    return [
        LogInfo(msg=f"[petcam] micro_ros_agent launch: {summary}"),
        ExecuteProcess(
            cmd=["bash", "-lc", cmd],
            output="screen",
            name="micro_ros_agent",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "transport",
                default_value="udp4",
                description="XRCE transport: udp4 | serial | tcp4 (ESP32 uses udp4)",
            ),
            DeclareLaunchArgument(
                "serial_dev",
                default_value="/dev/ttyACM0",
                description="Serial device path for ESP32-S3 USB CDC",
            ),
            DeclareLaunchArgument(
                "serial_baud",
                default_value="115200",
                description="Serial baud rate (must match ESP32 client)",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="8888",
                description="UDP/TCP port for network XRCE transport",
            ),
            DeclareLaunchArgument(
                "verbose",
                default_value="6",
                description="micro-ROS agent verbosity 0..6",
            ),
            OpaqueFunction(function=_build_agent),
        ]
    )
