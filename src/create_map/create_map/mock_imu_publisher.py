#!/usr/bin/env python3
"""Publish synthetic MPU6050-like Imu at 50 Hz (20 ms) for offline verify."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


# Match ESP32 best-effort publisher for end-to-end QoS testing
MOCK_IMU_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class MockImuPublisher(Node):
    """Simulates a short accelerate / coast / turn square path."""

    def __init__(self) -> None:
        super().__init__('mock_imu')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('accel_peak', 0.4)
        self.declare_parameter('gravity', 9.80665)

        topic = self.get_parameter('imu_topic').value
        rate = float(self.get_parameter('rate_hz').value)
        self.accel_peak = float(self.get_parameter('accel_peak').value)
        self.gravity = float(self.get_parameter('gravity').value)

        self.pub = self.create_publisher(Imu, topic, MOCK_IMU_QOS)
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'mock IMU publishing on {topic} at {rate:.1f} Hz (dt≈{1000.0/rate:.1f} ms)'
        )

    def _tick(self) -> None:
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        # 8-second repeating segments: +x accel, coast, +yaw, +y accel, ...
        phase = t % 8.0
        ax = ay = 0.0
        gz = 0.0
        if phase < 1.0:
            ax = self.accel_peak
        elif 2.0 <= phase < 2.5:
            gz = math.radians(90.0) / 0.5  # 90 deg turn in 0.5 s
        elif 3.0 <= phase < 4.0:
            ax = self.accel_peak  # forward in new heading after yaw integrate
        elif 5.0 <= phase < 5.5:
            gz = math.radians(90.0) / 0.5
        elif 6.0 <= phase < 7.0:
            ax = self.accel_peak

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = self.gravity
        msg.angular_velocity.z = gz
        # Covariance unknown
        msg.orientation_covariance[0] = -1.0
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockImuPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
