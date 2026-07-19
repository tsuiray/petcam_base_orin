"""Launch create_map: IMU odometry + live map viewer (+ optional agent / mock)."""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('create_map')
    bringup_share = FindPackageShare('petcam_bringup')
    config = PathJoinSubstitution([pkg_share, 'config', 'create_map.yaml'])
    fastdds_xml = PathJoinSubstitution(
        [pkg_share, 'config', 'fastdds_localhost.xml']
    )

    # Prefer install-space XML; fall back handled by env in run script.
    # Launch SetEnvironmentVariable needs a concrete path when possible.
    xml_candidates = []
    share_env = os.environ.get('COLCON_PREFIX_PATH', '')
    for prefix in share_env.split(os.pathsep):
        cand = os.path.join(prefix, 'create_map', 'share', 'create_map', 'config', 'fastdds_localhost.xml')
        if os.path.isfile(cand):
            xml_candidates.append(cand)
            break
    # Source-tree fallback for symlink-install mid-dev
    src_xml = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'fastdds_localhost.xml'
    )
    src_xml = os.path.abspath(src_xml)
    if os.path.isfile(src_xml):
        xml_candidates.append(src_xml)

    fastdds_xml_path = xml_candidates[0] if xml_candidates else ''

    env_actions = [
        SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_fastrtps_cpp'),
        SetEnvironmentVariable(name='FASTDDS_BUILTIN_TRANSPORTS', value='UDPv4'),
        SetEnvironmentVariable(name='ROS_LOCALHOST_ONLY', value='0'),
        LogInfo(
            msg=[
                '[petcam] FastDDS UDPv4-only (disable SHM) for micro_ros_agent <-> create_map'
            ]
        ),
    ]
    if fastdds_xml_path:
        env_actions.append(
            SetEnvironmentVariable(
                name='FASTRTPS_DEFAULT_PROFILES_FILE', value=fastdds_xml_path
            )
        )

    return LaunchDescription(
        env_actions
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
            # Clear stale DDS graph cache (common cause of empty topic list)
            ExecuteProcess(cmd=['ros2', 'daemon', 'stop'], output='screen'),
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
            Node(
                package='create_map',
                executable='mock_imu',
                name='mock_imu',
                output='screen',
                parameters=[config],
                condition=IfCondition(LaunchConfiguration('use_mock_imu')),
                additional_env={
                    'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4',
                    'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
                },
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
                additional_env={
                    'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4',
                    'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
                },
            ),
            Node(
                package='create_map',
                executable='map_viewer',
                name='map_viewer',
                output='screen',
                parameters=[config],
                condition=IfCondition(LaunchConfiguration('enable_viewer')),
                additional_env={
                    'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4',
                    'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
                },
            ),
        ]
    )
