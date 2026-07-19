"""Launch create_map: IMU odometry + live map viewer (+ optional agent / mock)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('create_map')
    bringup_share = FindPackageShare('petcam_bringup')
    config = PathJoinSubstitution([pkg_share, 'config', 'create_map.yaml'])

    return LaunchDescription(
        [
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
            ),
            Node(
                package='create_map',
                executable='map_viewer',
                name='map_viewer',
                output='screen',
                parameters=[config],
                condition=IfCondition(LaunchConfiguration('enable_viewer')),
            ),
        ]
    )
