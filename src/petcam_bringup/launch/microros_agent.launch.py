"""Launch micro-ROS Agent — default TCP :8888 (ESP32 MICROROS_TRANSPORT_TCP)."""

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
        summary = f"udp4 --port {port}"
    elif transport in ("tcp4", "tcp"):
        agent_cli = f"tcp4 --port {port} -v{verbose}"
        summary = f"tcp4 --port {port}"
    else:
        raise RuntimeError(
            f"Unsupported transport '{transport}'. Use serial, udp4, or tcp4."
        )

    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    microros_ws = os.environ.get("MICROROS_WS", os.path.expanduser("~/microros_ws"))
    fastdds_xml = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", "")

    # NOTE: do NOT use `set -u` here. ROS setup.bash references optional
    # AMENT_* vars and aborts under nounset — agent never binds :8888.
    cmd = f"""
set -e
echo "[petcam] === micro_ros_agent starting ({summary}) ==="
source /opt/ros/{ros_distro}/setup.bash
if [ -f "{microros_ws}/install/local_setup.bash" ]; then
  source "{microros_ws}/install/local_setup.bash"
  echo "[petcam] sourced {microros_ws}/install/local_setup.bash"
else
  echo "[petcam] ERROR: {microros_ws}/install/local_setup.bash missing" >&2
  echo "[petcam] Run: ./scripts/install_microros_agent.sh" >&2
  exit 1
fi
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
unset ROS_DISCOVERY_SERVER || true
if [ -n "{fastdds_xml}" ] && [ -f "{fastdds_xml}" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="{fastdds_xml}"
fi
if ! ros2 pkg prefix micro_ros_agent; then
  echo "[petcam] ERROR: micro_ros_agent package not found after sourcing" >&2
  exit 1
fi
if command -v fuser >/dev/null 2>&1; then
  fuser -k {port}/tcp 2>/dev/null || true
  fuser -k {port}/udp 2>/dev/null || true
fi
echo "[petcam] exec: ros2 run micro_ros_agent micro_ros_agent {agent_cli}"
exec ros2 run micro_ros_agent micro_ros_agent {agent_cli}
"""

    return [
        LogInfo(msg=f"[petcam] Launching micro_ros_agent ({summary})"),
        ExecuteProcess(
            cmd=["bash", "-c", cmd],
            output="screen",
            name="micro_ros_agent",
            shell=False,
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "transport",
                default_value="tcp4",
                description="XRCE transport: tcp4 (default) | udp4 | serial",
            ),
            DeclareLaunchArgument(
                "serial_dev",
                default_value="/dev/ttyACM0",
                description="Serial device for ESP32 USB",
            ),
            DeclareLaunchArgument(
                "serial_baud",
                default_value="115200",
                description="Serial baud rate",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="8888",
                description="XRCE port (ESP32 MICROROS_AGENT_PORT)",
            ),
            DeclareLaunchArgument(
                "verbose",
                default_value="6",
                description="Agent verbosity 0..6",
            ),
            OpaqueFunction(function=_build_agent),
        ]
    )
