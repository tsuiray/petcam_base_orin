#!/usr/bin/env python3
"""Subscribe to ESP32 MPU6050 IMU and publish dead-reckoned path/pose."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
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


def _qos_reliable(depth: int = 50) -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class ImuOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__('imu_odometry')

        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('raw_imu_topic', '/imu/raw')
        self.declare_parameter('prefer_raw', False)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('use_receive_time', True)
        self.declare_parameter('max_dt_sec', 0.05)
        self.declare_parameter('min_dt_sec', 0.0001)
        self.declare_parameter('default_dt_sec', 0.02)
        self.declare_parameter('accel_in_g', False)
        self.declare_parameter('gravity', 9.80665)
        self.declare_parameter('enable_zupt', True)
        self.declare_parameter('zupt_accel_epsilon', 0.45)
        self.declare_parameter('zupt_gyro_epsilon', 0.15)
        self.declare_parameter('zupt_hold_sec', 0.35)
        self.declare_parameter('calibrate_on_start_sec', 1.0)
        self.declare_parameter('path_max_poses', 5000)
        self.declare_parameter('path_min_step_m', 0.0005)
        self.declare_parameter('publish_tf', True)

        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.use_receive_time = bool(self.get_parameter('use_receive_time').value)
        prefer_raw = bool(self.get_parameter('prefer_raw').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)

        self.reckoner = ImuDeadReckoner(
            gravity=float(self.get_parameter('gravity').value),
            accel_in_g=bool(self.get_parameter('accel_in_g').value),
            enable_zupt=bool(self.get_parameter('enable_zupt').value),
            zupt_accel_epsilon=float(self.get_parameter('zupt_accel_epsilon').value),
            zupt_gyro_epsilon=float(self.get_parameter('zupt_gyro_epsilon').value),
            zupt_hold_sec=float(self.get_parameter('zupt_hold_sec').value),
            max_dt_sec=float(self.get_parameter('max_dt_sec').value),
            min_dt_sec=float(self.get_parameter('min_dt_sec').value),
            default_dt_sec=float(self.get_parameter('default_dt_sec').value),
            path_max_poses=int(self.get_parameter('path_max_poses').value),
            path_min_step_m=float(self.get_parameter('path_min_step_m').value),
            calibrate_on_start_sec=float(self.get_parameter('calibrate_on_start_sec').value),
        )

        self.path_pub = self.create_publisher(Path, '/create_map/path', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/create_map/pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/create_map/odom', 10)
        self.dist_pub = self.create_publisher(Float32, '/create_map/distance', 10)
        self.debug_pub = self.create_publisher(Float64MultiArray, '/create_map/debug', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self._imu_subs = []
        self._last_cb_ns = 0
        self._imu_via = ''

        if prefer_raw:
            raw_topic = self.get_parameter('raw_imu_topic').value
            self._imu_subs.append(
                self.create_subscription(
                    Float64MultiArray, raw_topic, self._on_raw, _qos_best_effort()
                )
            )
            self.get_logger().info(f'Subscribing raw IMU on {raw_topic}')
        else:
            # Dual QoS: micro-ROS agent QoS varies by build; incompatible QoS
            # yields ZERO callbacks while agent still prints XRCE hex.
            for label, qos in (
                ('best_effort', _qos_best_effort()),
                ('sensor_data', qos_profile_sensor_data),
                ('reliable', _qos_reliable()),
            ):
                sub = self.create_subscription(
                    Imu,
                    self.imu_topic,
                    lambda msg, src=label: self._on_imu(msg, src),
                    qos,
                )
                self._imu_subs.append(sub)
            self.get_logger().info(
                f'Subscribing {self.imu_topic} with best_effort+sensor_data+reliable '
                f'(dt from {"receive time" if self.use_receive_time else "msg stamp"})'
            )

        self._path_msg = Path()
        self._path_msg.header.frame_id = self.frame_id
        self._msg_count = 0
        self._last_state = None
        self._calibrating = True
        self.create_timer(0.5, self._heartbeat)
        self.create_timer(2.0, self._discover_imu)
        self.get_logger().info(
            'create_map imu_odometry ready — /create_map/debug heartbeat on. '
            'Waiting for /imu/data from micro-ROS agent.'
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
                'If agent shows XRCE hex but this persists, agent/create_map '
                'are on different FastDDS. Use: ./scripts/run_create_map.sh '
                '(unified overlay). Check: ros2 topic list | grep imu'
            )
            return
        parts = []
        for p in pubs:
            rel = str(p.qos_profile.reliability).split('.')[-1]
            dur = str(p.qos_profile.durability).split('.')[-1]
            parts.append(f'{p.node_name}[{rel}/{dur}]')
        self.get_logger().warn(
            f'{self.imu_topic} has {len(pubs)} publisher(s): {", ".join(parts)} '
            f'but imu_odometry got 0 msgs — QoS/discovery mismatch; dual-sub active'
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
        """status: 0=waiting_imu 1=calibrating 2=running"""
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
        if self.use_receive_time:
            return self.get_clock().now().nanoseconds * 1e-9
        sec = float(msg_stamp.sec) + float(msg_stamp.nanosec) * 1e-9
        if sec <= 0.0:
            return self.get_clock().now().nanoseconds * 1e-9
        return sec

    def _on_imu(self, msg: Imu, src: str = '') -> None:
        # Deduplicate when multiple QoS subscriptions deliver the same sample
        now_ns = self.get_clock().now().nanoseconds
        if self._last_cb_ns and (now_ns - self._last_cb_ns) < 1_000_000:
            return
        self._last_cb_ns = now_ns
        if src and src != self._imu_via:
            self._imu_via = src
            self.get_logger().info(f'Receiving /imu/data via QoS={src}')

        sample = ImuSample(
            stamp_sec=self._integration_stamp_sec(msg.header.stamp),
            ax=float(msg.linear_acceleration.x),
            ay=float(msg.linear_acceleration.y),
            az=float(msg.linear_acceleration.z),
            gx=float(msg.angular_velocity.x),
            gy=float(msg.angular_velocity.y),
            gz=float(msg.angular_velocity.z),
        )
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
        self._process(sample, now)

    def _process(self, sample: ImuSample, stamp) -> None:
        state = self.reckoner.update(sample)
        self._msg_count += 1
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
                f'{"CLAMP " if state.dt_clamped else ""}'
                f'a_raw=({state.ax_raw:.2f},{state.ay_raw:.2f}) '
                f'a_xy=({state.ax_body:.2f},{state.ay_body:.2f}) '
                f'bias=({bias_ax:.2f},{bias_ay:.2f}) '
                f'zupt={"ON" if state.zupt_active else "off"} '
                f'via={self._imu_via or "?"} '
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
