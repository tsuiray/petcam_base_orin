"""Launch create_map: ESP32 IMU → path map over micro-ROS TCP.

Default: ESP32 TCP → micro_ros_agent tcp4 :8888 → /imu/data → imu_odometry
(No UDP XRCE bridge — incompatible with TCP.)
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
    public_port = LaunchConfiguration('port').perform(context)
    agent_port = LaunchConfiguration('agent_port').perform(context)
    verbose = LaunchConfiguration('verbose').perform(context)
    transport = LaunchConfiguration('transport').perform(context).strip().lower() or 'tcp4'
    imu_mode = LaunchConfiguration('imu_mode').perform(context) or 'sim'
    use_bridge = LaunchConfiguration('use_xrce_bridge').perform(context).lower() in (
        '1',
        'true',
        'yes',
    )
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

    if transport in ('tcp', 'tcp4'):
        transport = 'tcp4'
    elif transport in ('udp', 'udp4'):
        transport = 'udp4'
    else:
        transport = 'tcp4'

    # UDP XRCE byte-proxy cannot front a TCP agent.
    if transport == 'tcp4' and use_bridge:
        use_bridge = False

    if imu_mode.lower() in ('real', 'body', 'mpu', 'mpu6050'):
        accel_frame = 'body'
        use_fixed_dt = 'false'
        calib_sec = '1.0'
    else:
        imu_mode = 'sim'
        accel_frame = 'world'
        use_fixed_dt = 'true'
        calib_sec = '0.0'

    bind_port = agent_port if use_bridge else public_port
    actions = []

    if start_agent:
        kill_proto = 'tcp' if transport == 'tcp4' else 'udp'
        actions.append(
            ExecuteProcess(
                cmd=[
                    'bash',
                    '-c',
                    _overlay(
                        f"""
echo "[petcam] micro_ros_agent {transport} :{bind_port}"
if ! ros2 pkg prefix micro_ros_agent >/dev/null; then
  echo "ERROR: install micro_ros_agent first" >&2; exit 1
fi
if command -v fuser >/dev/null 2>&1; then
  fuser -k {public_port}/{kill_proto} 2>/dev/null || true
  fuser -k {agent_port}/udp 2>/dev/null || true
  fuser -k {agent_port}/tcp 2>/dev/null || true
fi
exec ros2 run micro_ros_agent micro_ros_agent {transport} --port {bind_port} -v{verbose}
"""
                    ),
                ],
                output='screen',
                name='micro_ros_agent',
            )
        )

    if use_bridge:
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
echo "[petcam] xrce_imu_bridge :{public_port} → agent :{agent_port} + publish /imu/data"
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
                    f"""
echo "[petcam] imu_odometry mode={imu_mode} accel_frame={accel_frame} fixed_dt={use_fixed_dt} agent={transport}"
exec ros2 run create_map imu_odometry --ros-args \
  -r __node:=imu_odometry \
  -p imu_mode:={imu_mode} \
  -p accel_frame:={accel_frame} \
  -p use_fixed_dt:={use_fixed_dt} \
  -p calibrate_on_start_sec:={calib_sec} \
  -p default_dt_sec:=0.02
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

    delay = 2.0 if use_bridge else 1.5
    actions.append(TimerAction(period=delay, actions=node_actions))
    actions.append(
        TimerAction(
            period=delay + 1.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'bash',
                        '-c',
                        f"""
echo "[petcam] port check ({transport}):"
ss -tlnp 2>/dev/null | grep -E ":{public_port}[[:space:]]" || true
ss -ulnp 2>/dev/null | grep -E ":({public_port}|{agent_port})[[:space:]]" || true
""",
                    ],
                    output='screen',
                )
            ],
        )
    )
    actions.append(
        TimerAction(
            period=delay + 5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'bash',
                        '-c',
                        _overlay(
                            """
echo "[petcam] /imu/data check (5s):"
ros2 topic info /imu/data -v 2>/dev/null | head -40 || true
timeout 3 ros2 topic hz /imu/data 2>/dev/null || echo "[petcam] WARN: no /imu/data hz — check ESP32 MICROROS_TRANSPORT matches agent tcp4/udp4"
"""
                        ),
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
                    '[petcam] create_map default: micro_ros_agent tcp4 :8888 '
                    '(ESP32 MICROROS_TRANSPORT_TCP). UDP: transport:=udp4'
                )
            ),
            DeclareLaunchArgument('use_mock_imu', default_value='false'),
            DeclareLaunchArgument(
                'transport',
                default_value='tcp4',
                description='micro_ros_agent transport: tcp4 (default) | udp4',
            ),
            # Bridge is UDP-only; ignored when transport:=tcp4
            DeclareLaunchArgument('use_xrce_bridge', default_value='false'),
            DeclareLaunchArgument(
                'imu_mode',
                default_value='sim',
                description='sim=ESP32 L-home world-frame+fixed 20ms; real=MPU6050 body',
            ),
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
