"""Launch create_map: discovery server + micro-ROS agent + odometry + map."""

import os
import shutil

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

    discovery_server = os.environ.get('ROS_DISCOVERY_SERVER', '127.0.0.1:11811')
    dds_env = {
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4',
        'ROS_DISCOVERY_SERVER': discovery_server,
        'ROS_LOCALHOST_ONLY': '0',
    }
    if fastdds_xml_path:
        dds_env['FASTRTPS_DEFAULT_PROFILES_FILE'] = fastdds_xml_path

    env_actions = [
        SetEnvironmentVariable(name=k, value=v) for k, v in dds_env.items()
    ]
    env_actions.append(
        LogInfo(
            msg=(
                '[petcam] DDS: Discovery Server '
                f'{discovery_server} + UDPv4 (no SHM). XML={fastdds_xml_path or "none"}'
            )
        )
    )

    # Start Fast DDS Discovery Server if available (Humble: `fastdds discovery`)
    fastdds_bin = shutil.which('fastdds')
    discovery_actions = []
    if fastdds_bin:
        discovery_actions = [
            LogInfo(msg=f'[petcam] Starting Fast DDS Discovery Server via {fastdds_bin}'),
            ExecuteProcess(
                cmd=[
                    fastdds_bin,
                    'discovery',
                    '--server-id',
                    '0',
                    '--ip-address',
                    '127.0.0.1',
                    '--port',
                    '11811',
                ],
                output='screen',
                name='fastdds_discovery',
            ),
        ]
    else:
        discovery_actions = [
            LogInfo(
                msg=(
                    '[petcam] WARNING: `fastdds` CLI not found. '
                    'Install ros-humble-fastdds-tools OR rely on ROS_DISCOVERY_SERVER only. '
                    'If /imu/data still missing, use: ./scripts/run_create_map_docker_agent.sh'
                )
            )
        ]

    return LaunchDescription(
        env_actions
        + discovery_actions
        + [
            DeclareLaunchArgument(
                'use_mock_imu',
                default_value='false',
                description='Publish synthetic IMU at 50 Hz for offline verify',
            ),
            DeclareLaunchArgument(
                'start_microros_agent',
                default_value='true',
                description='Also start UDP micro-ROS agent for ESP32-S3',
            ),
            DeclareLaunchArgument(
                'imu_topic',
                default_value='/imu/data',
                description='sensor_msgs/Imu topic from ESP32',
            ),
            DeclareLaunchArgument(
                'port',
                default_value='8888',
                description='micro-ROS UDP port',
            ),
            DeclareLaunchArgument(
                'enable_viewer',
                default_value='true',
                description='Open OpenCV live map window',
            ),
            ExecuteProcess(cmd=['ros2', 'daemon', 'stop'], output='screen'),
            # Delay agent slightly so discovery server is listening
            TimerAction(
                period=1.0,
                actions=[
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
                ],
            ),
            TimerAction(
                period=1.5,
                actions=[
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
