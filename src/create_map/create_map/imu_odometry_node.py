#!/usr/bin/env python3
"""Subscribe to ESP32 IMU and publish dead-reckoned path/pose."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float64MultiArray
from tf2_ros import TransformBroadcaster

from create_map.dead_reckon import ImuDeadReckoner, ImuSample


def _qos_best_effort(depth: int = 50) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _resolve_imu_mode(mode: str) -> str:
    m = (mode or 'sim').strip().lower()
    if m in ('sim', 'simulation', 'world'):
        return 'sim'
    if m in ('real', 'body', 'mpu', 'mpu6050'):
        return 'real'
    if m == 'auto':
        return 'auto'
    return 'sim'


class ImuOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__('imu_odometry')

        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('raw_imu_topic', '/imu/raw')
        self.declare_parameter('prefer_raw', False)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')
        # imu_mode: sim (ESP32 L-home world-frame) | real (MPU6050 body) | auto
        self.declare_parameter('imu_mode', 'sim')
        self.declare_parameter('use_receive_time', True)
        self.declare_parameter('use_fixed_dt', True)
        self.declare_parameter('accel_frame', 'world')
        self.declare_parameter('max_dt_sec', 0.05)
        self.declare_parameter('min_dt_sec', 0.0001)
        self.declare_parameter('default_dt_sec', 0.02)
        self.declare_parameter('accel_in_g', False)
        self.declare_parameter('gravity', 9.80665)
        self.declare_parameter('enable_zupt', True)
        self.declare_parameter('zupt_accel_epsilon', 0.45)
        self.declare_parameter('zupt_gyro_epsilon', 0.15)
        self.declare_parameter('zupt_hold_sec', 0.35)
        self.declare_parameter('calibrate_on_start_sec', 0.0)
        self.declare_parameter('path_max_poses', 5000)
        self.declare_parameter('path_min_step_m', 0.0005)
        self.declare_parameter('publish_tf', True)
        # RX diagnostics: log first N samples fully, then every Nth sample.
        self.declare_parameter('log_rx_first_n', 5)
        self.declare_parameter('log_rx_every_n', 25)
        # SIM helpers: wait for a≈0 settle before integrating (avoid mid-lap scribble),
        # and snap pose to origin when a lap nearly closes (masks small UDP gaps).
        self.declare_parameter('sim_wait_settle', True)
        self.declare_parameter('sim_snap_lap', True)
        self.declare_parameter('sim_lap_min_dist_m', 20.0)
        self.declare_parameter('sim_origin_eps_m', 0.35)

        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.use_receive_time = bool(self.get_parameter('use_receive_time').value)
        prefer_raw = bool(self.get_parameter('prefer_raw').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.imu_mode = _resolve_imu_mode(str(self.get_parameter('imu_mode').value))

        accel_frame, use_fixed_dt, calib_sec, zupt_eps = self._mode_defaults()
        # Explicit params override mode defaults when set via launch/CLI.
        if self.has_parameter('accel_frame'):
            af = str(self.get_parameter('accel_frame').value).lower()
            if af in ('body', 'world'):
                accel_frame = af
        use_fixed_dt = bool(self.get_parameter('use_fixed_dt').value)
        calib_sec = float(self.get_parameter('calibrate_on_start_sec').value)

        self.reckoner = ImuDeadReckoner(
            gravity=float(self.get_parameter('gravity').value),
            accel_in_g=bool(self.get_parameter('accel_in_g').value),
            accel_frame=accel_frame,
            use_fixed_dt=use_fixed_dt,
            enable_zupt=bool(self.get_parameter('enable_zupt').value),
            zupt_accel_epsilon=float(self.get_parameter('zupt_accel_epsilon').value),
            zupt_gyro_epsilon=float(self.get_parameter('zupt_gyro_epsilon').value),
            zupt_hold_sec=float(self.get_parameter('zupt_hold_sec').value),
            max_dt_sec=float(self.get_parameter('max_dt_sec').value),
            min_dt_sec=float(self.get_parameter('min_dt_sec').value),
            default_dt_sec=float(self.get_parameter('default_dt_sec').value),
            path_max_poses=int(self.get_parameter('path_max_poses').value),
            path_min_step_m=float(self.get_parameter('path_min_step_m').value),
            calibrate_on_start_sec=calib_sec,
        )

        self.path_pub = self.create_publisher(Path, '/create_map/path', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/create_map/pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/create_map/odom', 10)
        self.dist_pub = self.create_publisher(Float32, '/create_map/distance', 10)
        self.debug_pub = self.create_publisher(Float64MultiArray, '/create_map/debug', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self._imu_subs = []
        self._imu_via = ''
        self._msg_count = 0
        self._rate_count = 0
        self._rate_t0 = time.monotonic()
        self._last_hz = 0.0
        self._last_rx: ImuSample | None = None
        self._log_rx_first_n = int(self.get_parameter('log_rx_first_n').value)
        self._log_rx_every_n = max(1, int(self.get_parameter('log_rx_every_n').value))
        self._auto_az: list[float] = []
        self._mode_locked = self.imu_mode != 'auto'
        self._sim_wait_settle = bool(self.get_parameter('sim_wait_settle').value) and (
            self.imu_mode == 'sim'
        )
        self._sim_snap_lap = bool(self.get_parameter('sim_snap_lap').value) and (
            self.imu_mode == 'sim'
        )
        self._sim_armed = not self._sim_wait_settle
        self._dist_at_lap = 0.0
        self._lost_est_total = 0.0
        self._expect_hz = 1.0 / max(
            1e-6, float(self.get_parameter('default_dt_sec').value)
        )

        if prefer_raw:
            raw_topic = self.get_parameter('raw_imu_topic').value
            self._imu_subs.append(
                self.create_subscription(
                    Float64MultiArray, raw_topic, self._on_raw, _qos_best_effort()
                )
            )
            self.get_logger().info(f'Subscribing raw IMU on {raw_topic}')
        else:
            self._imu_subs.append(
                self.create_subscription(
                    Imu, self.imu_topic, self._on_imu, _qos_best_effort()
                )
            )

        self._path_msg = Path()
        self._path_msg.header.frame_id = self.frame_id
        self._last_state = None
        self._calibrating = True
        self.create_timer(0.5, self._heartbeat)
        self.create_timer(1.0, self._log_rate)
        self.create_timer(2.0, self._discover_imu)

        dt_ms = float(self.get_parameter('default_dt_sec').value) * 1000.0
        self.get_logger().info(
            f'imu_odometry mode={self.imu_mode} accel_frame={self.reckoner.accel_frame} '
            f'fixed_dt={self.reckoner.use_fixed_dt} (expect {dt_ms:.0f} ms/sample, ~'
            f'{1.0 / float(self.get_parameter("default_dt_sec").value):.0f} Hz) '
            f'calib={calib_sec:.2f}s'
        )
        self.get_logger().info(
            'Waiting for /imu/data — ESP32 SIM needs world-frame+fixed 20ms; '
            'REAL MPU needs imu_mode:=real'
        )
        if self._sim_wait_settle:
            self.get_logger().info(
                'SIM: holding integration until first a≈0 settle sample '
                '(avoids mid-lap scribble at connect)'
            )

    def _mode_defaults(self):
        if self.imu_mode == 'real':
            return 'body', False, 1.0, 0.45
        if self.imu_mode == 'sim':
            return 'world', True, 0.0, 0.08
        # auto: start as sim until az seen
        return 'world', True, 0.0, 0.08

    def _maybe_autodetect(self, az: float) -> None:
        if self._mode_locked:
            return
        self._auto_az.append(abs(az))
        if len(self._auto_az) < 25:
            return
        med = sorted(self._auto_az)[len(self._auto_az) // 2]
        if med < 1.0:
            self.imu_mode = 'sim'
            self.reckoner.accel_frame = 'world'
            self.reckoner.use_fixed_dt = True
            self.reckoner.calibrate_on_start_sec = 0.0
            self.reckoner._calibrated = True
            self.get_logger().info(
                f'imu_mode auto→sim (|az| median={med:.2f}); world-frame + fixed 20ms'
            )
        else:
            self.imu_mode = 'real'
            self.reckoner.accel_frame = 'body'
            self.reckoner.use_fixed_dt = False
            self.reckoner.calibrate_on_start_sec = 1.0
            self.reckoner._calibrated = False
            self.reckoner._calib_samples.clear()
            self.reckoner._calib_start = None
            self.get_logger().info(
                f'imu_mode auto→real (|az| median={med:.2f}); body-frame + calib'
            )
        self._mode_locked = True

    def _log_rate(self) -> None:
        now = time.monotonic()
        elapsed = now - self._rate_t0
        if elapsed < 0.5:
            return
        hz = self._rate_count / elapsed
        self._last_hz = hz
        expect = self._expect_hz
        if self._msg_count == 0:
            self.get_logger().warn(
                f'create_map RX: 0 msgs on {self.imu_topic} in last {elapsed:.1f}s — '
                'ESP32 Serial OK does not mean Orin received /imu/data'
            )
        else:
            # Loss estimate vs ideal 50 Hz (UDP best-effort often drops some).
            expect_n = expect * elapsed
            got_n = float(self._rate_count)
            lost_n = max(0.0, expect_n - got_n)
            self._lost_est_total += lost_n
            loss_pct = 100.0 * lost_n / expect_n if expect_n > 1.0 else 0.0
            rx = self._last_rx
            st = self.reckoner.state
            rx_s = (
                f'a=({rx.ax:.3f},{rx.ay:.3f},{rx.az:.3f}) '
                f'g=({rx.gx:.3f},{rx.gy:.3f},{rx.gz:.3f})'
                if rx is not None
                else 'a=(?,?,?) g=(?,?,?)'
            )
            armed = 'armed' if self._sim_armed else 'WAIT_SETTLE'
            self.get_logger().info(
                f'create_map RX: {hz:.1f}/{expect:.0f} Hz '
                f'loss~{loss_pct:.0f}% (est lost {lost_n:.0f} this s, '
                f'{self._lost_est_total:.0f} total) [{armed}] '
                f'total={self._msg_count} integ={self.reckoner.samples_integrated} '
                f'last[{rx_s}] → pose=({st.x:.3f},{st.y:.3f}) '
                f'v=({st.vx:.3f},{st.vy:.3f}) dist={st.distance_m:.3f}m '
                f'path_pts={len(self._path_msg.poses)} '
                f'dt={st.dt * 1000:.1f}ms frame={self.reckoner.accel_frame}'
            )
            if loss_pct > 15.0:
                self.get_logger().warn(
                    'High packet loss estimate — L-laps will not overlap 100%. '
                    'UDP BEST_EFFORT cannot retransmit. Options: micro-ROS '
                    'RELIABLE publisher, or agent tcp4 (needs ESP32 TCP transport). '
                    'See docs/esp32_imu_contract.md#reliable-delivery'
                )
        self._rate_count = 0
        self._rate_t0 = now

    def _log_rx_sample(self, sample: ImuSample, frame_id: str) -> None:
        """Log what create_map actually got on /imu/data."""
        upcoming = self._msg_count + 1  # called before _process increments
        if upcoming <= self._log_rx_first_n or upcoming % self._log_rx_every_n == 0:
            self.get_logger().info(
                f'create_map GOT #{upcoming} frame_id={frame_id!r} '
                f'a=({sample.ax:.4f},{sample.ay:.4f},{sample.az:.4f}) '
                f'gyro=({sample.gx:.4f},{sample.gy:.4f},{sample.gz:.4f}) '
                f'mode={self.imu_mode}/{self.reckoner.accel_frame}'
            )

    def _discover_imu(self) -> None:
        if self._msg_count > 0:
            return
        try:
            pubs = self.get_publishers_info_by_topic(self.imu_topic)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'topic discovery failed: {exc}')
            return
        if not pubs:
            self.get_logger().warn(
                f'No DDS publisher on {self.imu_topic} yet. '
                'Check: ./scripts/run_create_map.sh and ESP32 agent IP:8888'
            )
            return
        parts = []
        for p in pubs:
            rel = str(p.qos_profile.reliability).split('.')[-1]
            dur = str(p.qos_profile.durability).split('.')[-1]
            parts.append(f'{p.node_name}[{rel}/{dur}]')
        self.get_logger().warn(
            f'{self.imu_topic} has {len(pubs)} publisher(s): {", ".join(parts)} '
            f'but imu_odometry got 0 msgs'
        )

    def _publish_debug(
        self,
        *,
        dt=0.0,
        ax_body=0.0,
        ay_body=0.0,
        ax_world=0.0,
        ay_world=0.0,
        vx=0.0,
        vy=0.0,
        zupt=False,
        still_sec=0.0,
        distance=0.0,
        x=0.0,
        y=0.0,
        dt_raw=0.0,
        dt_clamped=False,
        ax_raw=0.0,
        ay_raw=0.0,
        bias_ax=0.0,
        bias_ay=0.0,
        path_pts=0.0,
        integ=0.0,
        status=0.0,
    ) -> None:
        dbg = Float64MultiArray()
        dbg.data = [
            float(dt),
            float(ax_body),
            float(ay_body),
            float(ax_world),
            float(ay_world),
            float(vx),
            float(vy),
            1.0 if zupt else 0.0,
            float(still_sec),
            float(distance),
            float(x),
            float(y),
            float(dt_raw),
            1.0 if dt_clamped else 0.0,
            float(ax_raw),
            float(ay_raw),
            float(bias_ax),
            float(bias_ay),
            float(path_pts),
            float(integ),
            float(status),
            float(self._msg_count),
            float(self._last_hz),
        ]
        self.debug_pub.publish(dbg)

    def _heartbeat(self) -> None:
        if self._last_state is not None:
            return
        bias_ax, bias_ay = self.reckoner.bias_xy
        if self._msg_count == 0:
            status = 0.0
        elif self._calibrating:
            status = 1.0
        else:
            status = 2.0
        self._publish_debug(
            bias_ax=bias_ax,
            bias_ay=bias_ay,
            path_pts=len(self._path_msg.poses),
            integ=self.reckoner.samples_integrated,
            status=status,
        )

    def _integration_stamp_sec(self, msg_stamp) -> float:
        # Even with fixed_dt, keep a monotonic clock for ordering / debug dt_raw.
        if self.use_receive_time or self.reckoner.use_fixed_dt:
            return self.get_clock().now().nanoseconds * 1e-9
        sec = float(msg_stamp.sec) + float(msg_stamp.nanosec) * 1e-9
        if sec <= 0.0:
            return self.get_clock().now().nanoseconds * 1e-9
        return sec

    def _on_imu(self, msg: Imu, src: str = '') -> None:
        # Do NOT drop back-to-back samples: Wi-Fi/DDS often bursts packets.
        # Each sample is 20 ms of SIM motion regardless of receive clustering.
        via = src or 'best_effort'
        if via != self._imu_via:
            self._imu_via = via
            self.get_logger().info(
                f'create_map FIRST /imu/data via QoS={via} '
                f'frame_id={msg.header.frame_id!r}'
            )

        az = float(msg.linear_acceleration.z)
        self._maybe_autodetect(az)

        sample = ImuSample(
            stamp_sec=self._integration_stamp_sec(msg.header.stamp),
            ax=float(msg.linear_acceleration.x),
            ay=float(msg.linear_acceleration.y),
            az=az,
            gx=float(msg.angular_velocity.x),
            gy=float(msg.angular_velocity.y),
            gz=float(msg.angular_velocity.z),
        )
        self._last_rx = sample
        self._log_rx_sample(sample, str(msg.header.frame_id))
        self._process(sample, self.get_clock().now().to_msg())

    def _on_raw(self, msg: Float64MultiArray) -> None:
        data = list(msg.data)
        if len(data) < 6:
            self.get_logger().warn(
                f'raw IMU needs 6 values [ax,ay,az,gx,gy,gz], got {len(data)}',
                throttle_duration_sec=2.0,
            )
            return
        now = self.get_clock().now().to_msg()
        sample = ImuSample(
            stamp_sec=self.get_clock().now().nanoseconds * 1e-9,
            ax=float(data[0]),
            ay=float(data[1]),
            az=float(data[2]),
            gx=float(data[3]),
            gy=float(data[4]),
            gz=float(data[5]),
        )
        self._last_rx = sample
        self._log_rx_sample(sample, 'raw')
        self._process(sample, now)

    def _maybe_sim_gate(self, sample: ImuSample) -> bool:
        """Return False to skip integration (still counts as RX)."""
        if not self._sim_wait_settle or self._sim_armed:
            return True
        if math.hypot(sample.ax, sample.ay) < 1e-3 and abs(sample.gz) < 1e-3:
            self._sim_armed = True
            self.reckoner.reset()
            self._path_msg.poses = []
            self._dist_at_lap = 0.0
            self.get_logger().info(
                'SIM settle seen — start integrating from this corner (clean L)'
            )
            return True
        return False

    def _maybe_sim_snap_lap(self, state) -> None:
        if not self._sim_snap_lap or state is None:
            return
        min_dist = float(self.get_parameter('sim_lap_min_dist_m').value)
        eps = float(self.get_parameter('sim_origin_eps_m').value)
        traveled = state.distance_m - self._dist_at_lap
        if traveled < min_dist:
            return
        if math.hypot(state.x, state.y) > eps:
            return
        # Soft loop-closure for demo: UDP gaps accumulate; snap back to origin.
        self.get_logger().info(
            f'SIM lap snap: dist={state.distance_m:.2f}m near origin — '
            f'reset pose for overlap (est lost samples ~{self._lost_est_total:.0f})'
        )
        self.reckoner.state.x = 0.0
        self.reckoner.state.y = 0.0
        self.reckoner.state.vx = 0.0
        self.reckoner.state.vy = 0.0
        self.reckoner.state.yaw = 0.0
        self._dist_at_lap = state.distance_m
        # Keep path history so previous laps stay drawn; next points overlay.

    def _process(self, sample: ImuSample, stamp) -> None:
        self._rate_count += 1
        self._msg_count += 1
        if not self._maybe_sim_gate(sample):
            self._publish_debug(
                ax_raw=sample.ax,
                ay_raw=sample.ay,
                path_pts=len(self._path_msg.poses),
                integ=self.reckoner.samples_integrated,
                status=0.0,
            )
            return
        state = self.reckoner.update(sample)
        if state is None:
            self._calibrating = True
            if self._msg_count % 25 == 0:
                self.get_logger().info('Calibrating IMU bias at rest…')
            bias_ax, bias_ay = self.reckoner.bias_xy
            self._publish_debug(
                ax_raw=sample.ax,
                ay_raw=sample.ay,
                bias_ax=bias_ax,
                bias_ay=bias_ay,
                path_pts=len(self._path_msg.poses),
                integ=0.0,
                status=1.0,
            )
            return

        self._calibrating = False
        self._maybe_sim_snap_lap(state)
        self._last_state = state

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = state.x
        pose.pose.position.y = state.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quat(state.yaw)
        self.pose_pub.publish(pose)

        if state.path_appended or not self._path_msg.poses:
            self._path_msg.header.stamp = stamp
            if not self._path_msg.poses:
                self._path_msg.poses.append(pose)
            elif state.path_appended:
                self._path_msg.poses.append(pose)
            max_poses = int(self.get_parameter('path_max_poses').value)
            if len(self._path_msg.poses) > max_poses:
                self._path_msg.poses = self._path_msg.poses[-max_poses:]
        else:
            if self._path_msg.poses:
                self._path_msg.poses[-1] = pose
                self._path_msg.header.stamp = stamp
        self.path_pub.publish(self._path_msg)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose = pose.pose
        twist = Twist()
        twist.linear.x = state.vx
        twist.linear.y = state.vy
        twist.angular.z = sample.gz
        odom.twist.twist = twist
        self.odom_pub.publish(odom)

        dist = Float32()
        dist.data = float(state.distance_m)
        self.dist_pub.publish(dist)

        bias_ax, bias_ay = self.reckoner.bias_xy
        self._publish_debug(
            dt=state.dt,
            ax_body=state.ax_body,
            ay_body=state.ay_body,
            ax_world=state.ax_world,
            ay_world=state.ay_world,
            vx=state.vx,
            vy=state.vy,
            zupt=state.zupt_active,
            still_sec=state.still_sec,
            distance=state.distance_m,
            x=state.x,
            y=state.y,
            dt_raw=state.dt_raw,
            dt_clamped=state.dt_clamped,
            ax_raw=state.ax_raw,
            ay_raw=state.ay_raw,
            bias_ax=bias_ax,
            bias_ay=bias_ay,
            path_pts=len(self._path_msg.poses),
            integ=self.reckoner.samples_integrated,
            status=2.0,
        )

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.frame_id
            tf.child_frame_id = self.child_frame_id
            tf.transform.translation.x = state.x
            tf.transform.translation.y = state.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)

        if self._msg_count % 50 == 0:
            self.get_logger().info(
                f'pose=({state.x:.3f},{state.y:.3f}) dist={state.distance_m:.3f}m '
                f'v=({state.vx:.3f},{state.vy:.3f}) '
                f'dt={state.dt*1000:.1f}ms raw={state.dt_raw*1000:.1f}ms '
                f'hz={self._last_hz:.1f} frame={self.reckoner.accel_frame} '
                f'{"CLAMP " if state.dt_clamped else ""}'
                f'a_raw=({state.ax_raw:.2f},{state.ay_raw:.2f}) '
                f'path_pts={len(self._path_msg.poses)} integ={self.reckoner.samples_integrated}'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
