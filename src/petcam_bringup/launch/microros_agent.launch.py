"""Launch micro-ROS Agent via ExecuteProcess (reliable env / DDS discovery)."""

import os
import shutil

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
        agent_args = f"serial --dev {serial_dev} -b {serial_baud} -v{verbose}"
        summary = f"serial {serial_dev} @ {serial_baud}"
    elif transport in ("udp4", "udp"):
        agent_args = f"udp4 --port {port} -v{verbose}"
        summary = f"udp4 port {port}"
    elif transport in ("tcp4", "tcp"):
        agent_args = f"tcp4 --port {port} -v{verbose}"
        summary = f"tcp4 port {port}"
    else:
        raise RuntimeError(
            f"Unsupported transport '{transport}'. Use serial, udp4, or tcp4."
        )

    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    microros_ws = os.environ.get("MICROROS_WS", os.path.expanduser("~/microros_ws"))
    fastdds_xml = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", "")
    discovery = os.environ.get("ROS_DISCOVERY_SERVER", "127.0.0.1:11811")

    # Prefer `ros2 run` after sourcing microros_ws so agent + ROS share one env.
    cmd = (
        "set -e; "
        f"source /opt/ros/{ros_distro}/setup.bash; "
        f"if [ -f '{microros_ws}/install/local_setup.bash' ]; then "
        f"  source '{microros_ws}/install/local_setup.bash'; "
        "fi; "
        "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; "
        "export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; "
        f"export ROS_DISCOVERY_SERVER='{discovery}'; "
        "export ROS_LOCALHOST_ONLY=0; "
        f"if [ -n '{fastdds_xml}' ] && [ -f '{fastdds_xml}' ]; then "
        f"  export FASTRTPS_DEFAULT_PROFILES_FILE='{fastdds_xml}'; "
        "fi; "
        "echo \"[petcam] micro_ros_agent env: "
        "RMW=$RMW_IMPLEMENTATION DDS=$FASTDDS_BUILTIN_TRANSPORTS "
        "DISCOVERY=$ROS_DISCOVERY_SERVER XML=${FASTRTPS_DEFAULT_PROFILES_FILE:-none}\"; "
        f"exec ros2 run micro_ros_agent micro_ros_agent {agent_args}"
    )

    which = shutil.which("ros2") or "ros2"
    return [
        LogInfo(msg=f"[petcam] Starting micro_ros_agent ({summary}) via ExecuteProcess"),
        ExecuteProcess(
            cmd=["bash", "-lc", cmd],
            output="screen",
            name="micro_ros_agent",
        ),
        LogInfo(msg=f"[petcam] ros2 binary resolved near launch host: {which}"),
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
