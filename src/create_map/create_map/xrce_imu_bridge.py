#!/usr/bin/env python3
"""
Bidirectional UDP proxy + IMU extractor for PetCam.

ESP32  --> UDP :8888 (this bridge) --> micro_ros_agent :8887
                 |
                 +--> parse XRCE DATA containing sensor_msgs/Imu
                 +--> publish /imu/data on ROS 2 (bypasses agent DDS graph)

This fixes the common Orin issue where micro_ros_agent receives XRCE hex
but create_map never sees a DDS publisher on /imu/data.
"""

from __future__ import annotations

import select
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from create_map.xrce_imu_cdr import try_parse_imu_cdr


IMU_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
)


def _to_imu_msg(parsed, stamp_msg) -> Imu:
    msg = Imu()
    msg.header.stamp = stamp_msg
    msg.header.frame_id = parsed.frame_id
    msg.orientation.x = parsed.qx
    msg.orientation.y = parsed.qy
    msg.orientation.z = parsed.qz
    msg.orientation.w = parsed.qw
    msg.orientation_covariance[0] = -1.0
    msg.angular_velocity.x = parsed.gx
    msg.angular_velocity.y = parsed.gy
    msg.angular_velocity.z = parsed.gz
    msg.linear_acceleration.x = parsed.ax
    msg.linear_acceleration.y = parsed.ay
    msg.linear_acceleration.z = parsed.az
    return msg


class XrceImuBridge(Node):
    def __init__(self) -> None:
        super().__init__('xrce_imu_bridge')
        self.declare_parameter('listen_port', 8888)
        self.declare_parameter('agent_host', '127.0.0.1')
        self.declare_parameter('agent_port', 8887)
        self.declare_parameter('imu_topic', '/imu/data')

        self.listen_port = int(self.get_parameter('listen_port').value)
        self.agent_host = str(self.get_parameter('agent_host').value)
        self.agent_port = int(self.get_parameter('agent_port').value)
        imu_topic = str(self.get_parameter('imu_topic').value)

        self.pub = self.create_publisher(Imu, imu_topic, IMU_QOS)
        self._client_addr = None
        self._pub_count = 0
        self._last_log = 0.0

        self.sock_ext = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_ext.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_ext.bind(('0.0.0.0', self.listen_port))
        self.sock_ext.setblocking(False)

        self.sock_agent = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_agent.setblocking(False)
        # Bind ephemeral local port for agent replies
        self.sock_agent.bind(('0.0.0.0', 0))

        self.create_timer(0.001, self._poll)  # 1 kHz poll is fine for 50 Hz IMU
        self.get_logger().info(
            f'XRCE IMU bridge: ESP32 → :{self.listen_port} → agent '
            f'{self.agent_host}:{self.agent_port}, publish {imu_topic}'
        )

    def _poll(self) -> None:
        ready, _, _ = select.select([self.sock_ext, self.sock_agent], [], [], 0.0)
        for sock in ready:
            if sock is self.sock_ext:
                self._from_esp32()
            elif sock is self.sock_agent:
                self._from_agent()

    def _from_esp32(self) -> None:
        try:
            data, addr = self.sock_ext.recvfrom(4096)
        except BlockingIOError:
            return
        self._client_addr = addr
        try:
            self.sock_agent.sendto(data, (self.agent_host, self.agent_port))
        except OSError as exc:
            self.get_logger().warn(f'forward to agent failed: {exc}', throttle_duration_sec=2.0)

        imu_parsed = try_parse_imu_cdr(data)
        if imu_parsed is None:
            return
        # Prefer receive-time stamp for odometry dt stability
        imu = _to_imu_msg(imu_parsed, self.get_clock().now().to_msg())
        self.pub.publish(imu)
        self._pub_count += 1
        now = time.time()
        if now - self._last_log >= 2.0:
            self._last_log = now
            self.get_logger().info(
                f'Published /imu/data x{self._pub_count} '
                f'a=({imu.linear_acceleration.x:.2f},{imu.linear_acceleration.y:.2f},'
                f'{imu.linear_acceleration.z:.2f}) from {addr}'
            )

    def _from_agent(self) -> None:
        try:
            data, _ = self.sock_agent.recvfrom(4096)
        except BlockingIOError:
            return
        if self._client_addr is None:
            return
        try:
            self.sock_ext.sendto(data, self._client_addr)
        except OSError:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = XrceImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock_ext.close()
        node.sock_agent.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
