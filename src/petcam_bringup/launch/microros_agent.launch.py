"""Launch micro-ROS Agent on the PetCam Orin base station for ESP32-S3."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_agent(context, *args, **kwargs):
    transport = LaunchConfiguration("transport").perform(context).strip().lower()
    serial_dev = LaunchConfiguration("serial_dev").perform(context)
    serial_baud = LaunchConfiguration("serial_baud").perform(context)
    port = LaunchConfiguration("port").perform(context)
    verbose = LaunchConfiguration("verbose").perform(context)

    if transport == "serial":
        agent_args = [
            "serial",
            "--dev",
            serial_dev,
            "-b",
            serial_baud,
            f"-v{verbose}",
        ]
        summary = f"serial {serial_dev} @ {serial_baud}"
    elif transport in ("udp4", "udp"):
        agent_args = ["udp4", "--port", port, f"-v{verbose}"]
        summary = f"udp4 port {port}"
    elif transport in ("tcp4", "tcp"):
        agent_args = ["tcp4", "--port", port, f"-v{verbose}"]
        summary = f"tcp4 port {port}"
    else:
        raise RuntimeError(
            f"Unsupported transport '{transport}'. Use serial, udp4, or tcp4."
        )

    return [
        LogInfo(msg=f"[petcam] Starting micro_ros_agent ({summary})"),
        Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            name="micro_ros_agent",
            output="screen",
            arguments=agent_args,
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "transport",
                default_value="serial",
                description="XRCE transport: serial | udp4 | tcp4",
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
