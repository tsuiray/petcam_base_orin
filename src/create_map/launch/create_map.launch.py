"""Launch create_map: agent + nodes in the SAME sourced overlay (shared FastDDS)."""

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
    # install/create_map/share/create_map/launch -> ../../../../local_setup.bash
    guess = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'local_setup.bash')
    )
    return guess if os.path.isfile(guess) else ''


def _overlay_prefix(body: str) -> str:
    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    microros_ws = os.environ.get('MICROROS_WS', os.path.expanduser('~/microros_ws'))
    install_setup = _resolve_install_setup()
    fastdds_xml = os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE', '')
    return f"""
set -e
source /opt/ros/{ros_distro}/setup.bash
if [ -f "{microros_ws}/install/local_setup.bash" ]; then
  source "{microros_ws}/install/local_setup.bash"
fi
if [ -f "{install_setup}" ]; then
  source "{install_setup}"
fi
export LD_LIBRARY_PATH="{microros_ws}/install/lib:{microros_ws}/install/lib/aarch64-linux-gnu:/opt/ros/{ros_distro}/lib:/opt/ros/{ros_distro}/lib/aarch64-linux-gnu:${{LD_LIBRARY_PATH:-}}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
unset ROS_DISCOVERY_SERVER || true
if [ -n "{fastdds_xml}" ] && [ -f "{fastdds_xml}" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="{fastdds_xml}"
fi
{body}
"""


def _launch_setup(context, *args, **kwargs):
    port = LaunchConfiguration('port').perform(context)
    verbose = LaunchConfiguration('verbose').perform(context)
    start_agent = LaunchConfiguration('start_microros_agent').perform(context).lower() in (
        'true',
        '1',
        'yes',
    )
    use_mock = LaunchConfiguration('use_mock_imu').perform(context).lower() in (
        'true',
        '1',
        'yes',
    )
    enable_viewer = LaunchConfiguration('enable_viewer').perform(context).lower() in (
        'true',
        '1',
        'yes',
    )
    imu_topic = LaunchConfiguration('imu_topic').perform(context)

    actions = []

    if start_agent:
        actions.append(
            ExecuteProcess(
                cmd=[
                    'bash',
                    '-c',
                    _overlay_prefix(
                        f"""
echo "[petcam] === micro_ros_agent udp4 --port {port} ==="
if ! ros2 pkg prefix micro_ros_agent; then
  echo "[petcam] ERROR: micro_ros_agent missing — run ./scripts/install_microros_agent.sh" >&2
  exit 1
fi
if command -v fuser >/dev/null 2>&1; then fuser -k {port}/udp 2>/dev/null || true; fi
exec ros2 run micro_ros_agent micro_ros_agent udp4 --port {port} -v{verbose}
"""
                    ),
                ],
                output='screen',
                name='micro_ros_agent',
            )
        )
        actions.append(
            TimerAction(
                period=2.5,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            'bash',
                            '-c',
                            f"""
if ss -uln 2>/dev/null | grep -E ":{port}[[:space:]]" >/dev/null; then
  echo "[petcam] OK: UDP {port} listening"
else
  echo "[petcam] ERROR: UDP {port} NOT listening" >&2
fi
""",
                        ],
                        output='screen',
                    )
                ],
            )
        )

    node_actions = []
    if use_mock:
        node_actions.append(
            ExecuteProcess(
                cmd=[
                    'bash',
                    '-c',
                    _overlay_prefix('exec ros2 run create_map mock_imu --ros-args -r __node:=mock_imu'),
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
                _overlay_prefix(
                    f"""
echo "[petcam] === imu_odometry (same FastDDS overlay as agent) ==="
# Probe graph from THIS overlay
ros2 topic list 2>/dev/null | head -50 || true
exec ros2 run create_map imu_odometry --ros-args \
  -r __node:=imu_odometry \
  -p imu_topic:={imu_topic}
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
                    _overlay_prefix(
                        'exec ros2 run create_map map_viewer --ros-args -r __node:=map_viewer'
                    ),
                ],
                output='screen',
                name='map_viewer',
            )
        )

    # Start nodes after agent has a moment to create DDS writers
    actions.append(TimerAction(period=3.0, actions=node_actions))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            LogInfo(
                msg='[petcam] create_map unified-overlay launch (fixes agent XRCE without ROS /imu/data)'
            ),
            DeclareLaunchArgument('use_mock_imu', default_value='false'),
            DeclareLaunchArgument('start_microros_agent', default_value='true'),
            DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
            DeclareLaunchArgument('port', default_value='8888'),
            DeclareLaunchArgument('enable_viewer', default_value='true'),
            DeclareLaunchArgument('verbose', default_value='4'),
            ExecuteProcess(cmd=['ros2', 'daemon', 'stop'], output='screen'),
            OpaqueFunction(function=_launch_setup),
        ]
    )
