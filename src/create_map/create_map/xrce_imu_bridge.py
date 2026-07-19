#!/usr/bin/env python3
"""
Bidirectional UDP proxy + IMU extractor for PetCam (optional fallback).

ESP32  --> UDP :8888 (this bridge) --> micro_ros_agent :8887
                 |
                 +--> parse XRCE DATA containing sensor_msgs/Imu
                 +--> publish /imu/data on ROS 2 (bypasses agent DDS graph)

Prefer the default create_map launch (agent directly on :8888). Use this
bridge only when agent DDS discovery still fails on the Orin.
"""

from __future__ import annotations

import select
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from create_map.xrce_imu_cdr import try_parse_imu_cdr, xrce_summary


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
        self._client_lock = threading.Lock()
        self._pub_count = 0
        self._udp_count = 0
        self._agent_rx = 0
        self._parse_miss = 0
        self._last_log = 0.0
        self._saw_client = False
        self._hex_dumps_left = 24
        self._stop = threading.Event()

        # Larger buffers so XRCE ping/CREATE bursts are not dropped.
        self.sock_ext = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_ext.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_ext.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock_ext.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
        self.sock_ext.bind(('0.0.0.0', self.listen_port))
        self.sock_ext.setblocking(False)

        self.sock_agent = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_agent.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock_agent.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
        self.sock_agent.setblocking(False)
        self.sock_agent.bind(('0.0.0.0', 0))

        # Dedicated thread: ROS timers are too slow/jittery for XRCE ping RTT.
        self._thread = threading.Thread(target=self._proxy_loop, name='xrce_udp', daemon=True)
        self._thread.start()
        self.create_timer(3.0, self._status)
        self.get_logger().info(
            f'XRCE IMU bridge (threaded): ESP32 → :{self.listen_port} → agent '
            f'{self.agent_host}:{self.agent_port}, publish {imu_topic}'
        )

    def _status(self) -> None:
        if self._pub_count > 0:
            return
        if self._udp_count == 0:
            self.get_logger().warn(
                f'Waiting for ESP32 XRCE UDP on :{self.listen_port} '
                f'(MICROROS_AGENT_IP = this Orin Wi-Fi IP, port {self.listen_port})'
            )
            return
        self.get_logger().warn(
            f'ESP32 UDP ok (esp={self._udp_count} agent_rx={self._agent_rx} '
            f'from {self._client_addr}) but no Imu CDR yet '
            f'(parse_miss={self._parse_miss}). '
            'If counters freeze after ~14, XRCE session stalled — prefer agent on :8888.'
        )

    def _proxy_loop(self) -> None:
        while not self._stop.is_set():
            try:
                ready, _, _ = select.select(
                    [self.sock_ext, self.sock_agent], [], [], 0.01
                )
            except (OSError, ValueError):
                break
            if self.sock_ext in ready:
                self._drain_esp32()
            if self.sock_agent in ready:
                self._drain_agent()

    def _drain_esp32(self) -> None:
        while True:
            try:
                data, addr = self.sock_ext.recvfrom(8192)
            except BlockingIOError:
                return
            except OSError:
                return
            with self._client_lock:
                self._client_addr = addr
            self._udp_count += 1
            if not self._saw_client:
                self._saw_client = True
                self.get_logger().info(
                    f'ESP32 XRCE peer {addr} first UDP {len(data)} bytes'
                )
            if self._hex_dumps_left > 0:
                self._hex_dumps_left -= 1
                self.get_logger().info(
                    f'ESP32 pkt#{self._udp_count} {xrce_summary(data)} '
                    f'hex={data[:48].hex()}'
                )
            try:
                self.sock_agent.sendto(data, (self.agent_host, self.agent_port))
            except OSError as exc:
                self.get_logger().warn(
                    f'forward to agent failed: {exc}', throttle_duration_sec=2.0
                )

            imu_parsed = try_parse_imu_cdr(data)
            if imu_parsed is None:
                self._parse_miss += 1
                continue
            stamp = self.get_clock().now().to_msg()
            imu = _to_imu_msg(imu_parsed, stamp)
            self.pub.publish(imu)
            self._pub_count += 1
            now = time.time()
            if self._pub_count == 1 or now - self._last_log >= 2.0:
                self._last_log = now
                self.get_logger().info(
                    f'Published /imu/data x{self._pub_count} '
                    f'a=({imu.linear_acceleration.x:.2f},{imu.linear_acceleration.y:.2f},'
                    f'{imu.linear_acceleration.z:.2f}) from {addr}'
                )

    def _drain_agent(self) -> None:
        while True:
            try:
                data, _ = self.sock_agent.recvfrom(8192)
            except BlockingIOError:
                return
            except OSError:
                return
            self._agent_rx += 1
            with self._client_lock:
                client = self._client_addr
            if client is None:
                continue
            try:
                self.sock_ext.sendto(data, client)
            except OSError:
                pass

    def destroy_node(self) -> bool:
        self._stop.set()
        try:
            self._thread.join(timeout=1.0)
        except RuntimeError:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = XrceImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop.set()
        try:
            node.sock_ext.close()
            node.sock_agent.close()
        except OSError:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
