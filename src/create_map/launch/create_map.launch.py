"""Launch create_map with XRCE UDP bridge (bypasses agent DDS discovery).

ESP32 → :8888 bridge → agent :8887
              └─ publish /imu/data → imu_odometry → map_viewer
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration


def _resolve_install_setup() -> str:
    for p in os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep):
        if not p:
            continue
        if os.path.basename(p) == 'install' and os.path.isfile(os.path.join(p, 'local_setup.bash')):
            return os.path.join(p, 'local_setup.bash')
        parent = os.path.dirname(p)
        if os.path.basename(parent) == 'install' and os.path.isfile(
            os.path.join(parent, 'local_setup.bash')
        ):
            return os.path.join(parent, 'local_setup.bash')
    guess = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'local_setup.bash')
    )
    return guess if os.path.isfile(guess) else ''


def _overlay(body: str) -> str:
    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    microros_ws = os.environ.get('MICROROS_WS', os.path.expanduser('~/microros_ws'))
    install_setup = _resolve_install_setup()
    return f"""
set -e
source /opt/ros/{ros_distro}/setup.bash
[ -f "{microros_ws}/install/local_setup.bash" ] && source "{microros_ws}/install/local_setup.bash"
[ -f "{install_setup}" ] && source "{install_setup}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
unset ROS_DISCOVERY_SERVER || true
{body}
"""


def _setup(context, *args, **kwargs):
    public_port = LaunchConfiguration('port').perform(context)  # ESP32 targets this
    agent_port = LaunchConfiguration('agent_port').perform(context)
    verbose = LaunchConfiguration('verbose').perform(context)
    start_agent = LaunchConfiguration('start_microros_agent').perform(context).lower() in (
        '1',
        'true',
        'yes',
    )
    enable_viewer = LaunchConfiguration('enable_viewer').perform(context).lower() in (
        '1',
        'true',
        'yes',
    )
    use_mock = LaunchConfiguration('use_mock_imu').perform(context).lower() in (
        '1',
        'true',
        'yes',
    )

    actions = []

    # 1) Agent on INTERNAL port (not seen by ESP32 directly)
    if start_agent:
        actions.append(
            ExecuteProcess(
                cmd=[
                    'bash',
                    '-c',
                    _overlay(
                        f"""
echo "[petcam] micro_ros_agent on INTERNAL udp4 :{agent_port}"
if ! ros2 pkg prefix micro_ros_agent >/dev/null; then
  echo "ERROR: install micro_ros_agent first" >&2; exit 1
fi
if command -v fuser >/dev/null 2>&1; then
  fuser -k {agent_port}/udp 2>/dev/null || true
  fuser -k {public_port}/udp 2>/dev/null || true
fi
exec ros2 run micro_ros_agent micro_ros_agent udp4 --port {agent_port} -v{verbose}
"""
                    ),
                ],
                output='screen',
                name='micro_ros_agent',
            )
        )

    # 2) Bridge on PUBLIC :8888 + ROS /imu/data publisher
    actions.append(
        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'bash',
                        '-c',
                        _overlay(
                            f"""
echo "[petcam] xrce_imu_bridge :{public_port} → agent :{agent_port}"
exec ros2 run create_map xrce_imu_bridge --ros-args \
  -r __node:=xrce_imu_bridge \
  -p listen_port:={public_port} \
  -p agent_port:={agent_port} \
  -p imu_topic:=/imu/data
"""
                        ),
                    ],
                    output='screen',
                    name='xrce_imu_bridge',
                )
            ],
        )
    )

    # 3) Odometry + map after bridge is up
    node_actions = []
    if use_mock:
        node_actions.append(
            ExecuteProcess(
                cmd=[
                    'bash',
                    '-c',
                    _overlay('exec ros2 run create_map mock_imu --ros-args -r __node:=mock_imu'),
                ],
                output='screen',
                name='mock_imu',
            )
        )
    node_actions.append(
        ExecuteProcess(
            cmd=[
                'bash',
                '-c',
                _overlay(
                    """
echo "[petcam] imu_odometry (subscribes /imu/data from xrce_imu_bridge)"
exec ros2 run create_map imu_odometry --ros-args -r __node:=imu_odometry
"""
                ),
            ],
            output='screen',
            name='imu_odometry',
        )
    )
    if enable_viewer:
        node_actions.append(
            ExecuteProcess(
                cmd=[
                    'bash',
                    '-c',
                    _overlay(
                        'exec ros2 run create_map map_viewer --ros-args -r __node:=map_viewer'
                    ),
                ],
                output='screen',
                name='map_viewer',
            )
        )

    actions.append(TimerAction(period=2.0, actions=node_actions))
    actions.append(
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'bash',
                        '-c',
                        f"""
echo "[petcam] port check:"
ss -ulnp 2>/dev/null | grep -E ":({public_port}|{agent_port})[[:space:]]" || true
""",
                    ],
                    output='screen',
                )
            ],
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    '[petcam] XRCE IMU bridge mode: ESP32→:8888→agent:8887 + /imu/data '
                    '(bypasses agent DDS discovery)'
                )
            ),
            DeclareLaunchArgument('use_mock_imu', default_value='false'),
            DeclareLaunchArgument('start_microros_agent', default_value='true'),
            DeclareLaunchArgument('port', default_value='8888'),
            DeclareLaunchArgument('agent_port', default_value='8887'),
            DeclareLaunchArgument('enable_viewer', default_value='true'),
            DeclareLaunchArgument('verbose', default_value='4'),
            DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
            ExecuteProcess(cmd=['ros2', 'daemon', 'stop'], output='screen'),
            OpaqueFunction(function=_setup),
        ]
    )
