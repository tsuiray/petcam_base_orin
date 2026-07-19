#!/usr/bin/env python3
"""Subscribe to ESP32 MPU6050 IMU and publish dead-reckoned path/pose."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float64MultiArray
from tf2_ros import TransformBroadcaster

from create_map.dead_reckon import ImuDeadReckoner, ImuSample


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
        self.declare_parameter('max_dt_sec', 0.1)
        self.declare_parameter('min_dt_sec', 0.001)
        self.declare_parameter('accel_in_g', False)
        self.declare_parameter('gravity', 9.80665)
        self.declare_parameter('enable_zupt', True)
        self.declare_parameter('zupt_accel_epsilon', 0.35)
        self.declare_parameter('zupt_gyro_epsilon', 0.08)
        self.declare_parameter('calibrate_on_start_sec', 1.0)
        self.declare_parameter('path_max_poses', 5000)
        self.declare_parameter('publish_tf', True)

        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        prefer_raw = bool(self.get_parameter('prefer_raw').value)

        self.reckoner = ImuDeadReckoner(
            gravity=float(self.get_parameter('gravity').value),
            accel_in_g=bool(self.get_parameter('accel_in_g').value),
            enable_zupt=bool(self.get_parameter('enable_zupt').value),
            zupt_accel_epsilon=float(self.get_parameter('zupt_accel_epsilon').value),
            zupt_gyro_epsilon=float(self.get_parameter('zupt_gyro_epsilon').value),
            max_dt_sec=float(self.get_parameter('max_dt_sec').value),
            min_dt_sec=float(self.get_parameter('min_dt_sec').value),
            path_max_poses=int(self.get_parameter('path_max_poses').value),
            calibrate_on_start_sec=float(self.get_parameter('calibrate_on_start_sec').value),
        )

        self.path_pub = self.create_publisher(Path, '/create_map/path', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/create_map/pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/create_map/odom', 10)
        self.dist_pub = self.create_publisher(Float32, '/create_map/distance', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        imu_topic = self.get_parameter('imu_topic').value
        raw_topic = self.get_parameter('raw_imu_topic').value

        if prefer_raw:
            self.create_subscription(
                Float64MultiArray, raw_topic, self._on_raw, qos_profile_sensor_data
            )
            self.get_logger().info(f'Subscribing raw IMU Float64MultiArray on {raw_topic}')
        else:
            self.create_subscription(Imu, imu_topic, self._on_imu, qos_profile_sensor_data)
            self.get_logger().info(f'Subscribing sensor_msgs/Imu on {imu_topic}')

        self._path_msg = Path()
        self._path_msg.header.frame_id = self.frame_id
        self._msg_count = 0
        self.get_logger().info(
            'create_map imu_odometry ready — waiting for ESP32 MPU6050 packets (~20 ms)'
        )

    def _stamp_to_sec(self, stamp) -> float:
        if stamp.sec == 0 and stamp.nanosec == 0:
            now = self.get_clock().now().nanoseconds * 1e-9
            return now
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _on_imu(self, msg: Imu) -> None:
        sample = ImuSample(
            stamp_sec=self._stamp_to_sec(msg.header.stamp),
            ax=float(msg.linear_acceleration.x),
            ay=float(msg.linear_acceleration.y),
            az=float(msg.linear_acceleration.z),
            gx=float(msg.angular_velocity.x),
            gy=float(msg.angular_velocity.y),
            gz=float(msg.angular_velocity.z),
        )
        self._process(sample, msg.header.stamp)

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
            stamp_sec=self._stamp_to_sec(now),
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
            if self._msg_count % 25 == 0:
                self.get_logger().info('Calibrating IMU bias at rest…')
            return

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = state.x
        pose.pose.position.y = state.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quat(state.yaw)
        self.pose_pub.publish(pose)

        self._path_msg.header.stamp = stamp
        self._path_msg.poses.append(pose)
        max_poses = int(self.get_parameter('path_max_poses').value)
        if len(self._path_msg.poses) > max_poses:
            self._path_msg.poses = self._path_msg.poses[-max_poses:]
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
                f'pose=({state.x:.3f},{state.y:.3f}) yaw={math.degrees(state.yaw):.1f}deg '
                f'distance={state.distance_m:.3f}m packets={self._msg_count}'
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
