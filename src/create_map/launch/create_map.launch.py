"""Launch create_map: micro-ROS agent first (ESP32 reachability), then nodes."""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _find_fastdds_xml() -> str:
    share_env = os.environ.get('COLCON_PREFIX_PATH', '')
    for prefix in share_env.split(os.pathsep):
        cand = os.path.join(
            prefix, 'create_map', 'share', 'create_map', 'config', 'fastdds_localhost.xml'
        )
        if os.path.isfile(cand):
            return cand
    src_xml = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..', 'config', 'fastdds_localhost.xml')
    )
    return src_xml if os.path.isfile(src_xml) else ''


def generate_launch_description():
    pkg_share = FindPackageShare('create_map')
    bringup_share = FindPackageShare('petcam_bringup')
    config = PathJoinSubstitution([pkg_share, 'config', 'create_map.yaml'])
    fastdds_xml_path = _find_fastdds_xml()

    # Simple discovery + UDPv4. Discovery Server is NOT required for XRCE bind.
    dds_env = {
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4',
        'ROS_LOCALHOST_ONLY': '0',
    }
    if fastdds_xml_path:
        dds_env['FASTRTPS_DEFAULT_PROFILES_FILE'] = fastdds_xml_path

    env_actions = [SetEnvironmentVariable(name=k, value=v) for k, v in dds_env.items()]
    env_actions.append(
        LogInfo(
            msg=(
                '[petcam] Agent binds XRCE UDP first; DDS=UDPv4 simple discovery. '
                f'XML={fastdds_xml_path or "none"}'
            )
        )
    )

    return LaunchDescription(
        env_actions
        + [
            DeclareLaunchArgument('use_mock_imu', default_value='false'),
            DeclareLaunchArgument('start_microros_agent', default_value='true'),
            DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
            DeclareLaunchArgument('port', default_value='8888'),
            DeclareLaunchArgument('enable_viewer', default_value='true'),
            ExecuteProcess(cmd=['ros2', 'daemon', 'stop'], output='screen'),
            # Start agent IMMEDIATELY so ESP32 can ping :8888
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [bringup_share, 'launch', 'microros_agent.launch.py']
                    )
                ),
                condition=IfCondition(LaunchConfiguration('start_microros_agent')),
                launch_arguments={
                    'transport': 'udp4',
                    'port': LaunchConfiguration('port'),
                }.items(),
            ),
            # Start create_map nodes after agent has time to bind :8888
            TimerAction(
                period=3.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            'bash',
                            '-c',
                            'P="${PETCAM_AGENT_PORT:-8888}"; '
                            'if ss -uln 2>/dev/null | grep -E ":$P[[:space:]]" >/dev/null; then '
                            '  echo "[petcam] OK: UDP $P is listening — ESP32 can reach agent"; '
                            '  ss -ulnp 2>/dev/null | grep -E ":$P[[:space:]]" || true; '
                            'else '
                            '  echo "[petcam] ERROR: UDP $P NOT listening — agent failed (see micro_ros_agent log)" >&2; '
                            '  echo "[petcam] Try standalone: ./scripts/run_microros_agent.sh" >&2; '
                            '  pgrep -af micro_ros_agent || echo "[petcam] no micro_ros_agent process" >&2; '
                            'fi',
                        ],
                        additional_env={'PETCAM_AGENT_PORT': LaunchConfiguration('port')},
                        output='screen',
                    ),
                    Node(
                        package='create_map',
                        executable='mock_imu',
                        name='mock_imu',
                        output='screen',
                        parameters=[config],
                        condition=IfCondition(LaunchConfiguration('use_mock_imu')),
                        additional_env=dds_env,
                    ),
                    Node(
                        package='create_map',
                        executable='imu_odometry',
                        name='imu_odometry',
                        output='screen',
                        parameters=[
                            config,
                            {'imu_topic': LaunchConfiguration('imu_topic')},
                        ],
                        additional_env=dds_env,
                    ),
                    Node(
                        package='create_map',
                        executable='map_viewer',
                        name='map_viewer',
                        output='screen',
                        parameters=[config],
                        condition=IfCondition(LaunchConfiguration('enable_viewer')),
                        additional_env=dds_env,
                    ),
                ],
            ),
        ]
    )
