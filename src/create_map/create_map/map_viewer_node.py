#!/usr/bin/env python3
"""Live 2D map window showing the robot path from create_map odometry."""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray


class MapViewerNode(Node):
    def __init__(self) -> None:
        super().__init__('map_viewer')

        self.declare_parameter('path_topic', '/create_map/path')
        self.declare_parameter('pose_topic', '/create_map/pose')
        self.declare_parameter('distance_topic', '/create_map/distance')
        self.declare_parameter('debug_topic', '/create_map/debug')
        self.declare_parameter('window_name', 'PetCam create_map')
        self.declare_parameter('pixels_per_meter', 200.0)
        self.declare_parameter('trail_thickness', 3)
        self.declare_parameter('show_grid', True)
        self.declare_parameter('grid_spacing_m', 0.25)
        self.declare_parameter('canvas_size', 800)

        self.window_name = str(self.get_parameter('window_name').value)
        self.ppm = float(self.get_parameter('pixels_per_meter').value)
        self.base_ppm = self.ppm
        self.trail_thickness = int(self.get_parameter('trail_thickness').value)
        self.show_grid = bool(self.get_parameter('show_grid').value)
        self.grid_spacing_m = float(self.get_parameter('grid_spacing_m').value)
        self.canvas_size = int(self.get_parameter('canvas_size').value)

        self._path_xy: List[Tuple[float, float]] = []
        self._pose: Optional[PoseStamped] = None
        self._distance_m = 0.0
        self._debug = [0.0] * 12

        self.create_subscription(
            Path, self.get_parameter('path_topic').value, self._on_path, 10
        )
        self.create_subscription(
            PoseStamped, self.get_parameter('pose_topic').value, self._on_pose, 10
        )
        self.create_subscription(
            Float32, self.get_parameter('distance_topic').value, self._on_distance, 10
        )
        self.create_subscription(
            Float64MultiArray, self.get_parameter('debug_topic').value, self._on_debug, 10
        )

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.canvas_size, self.canvas_size)
        self.create_timer(1.0 / 30.0, self._draw)
        self.get_logger().info(f'Map viewer window: {self.window_name}')

    def _on_path(self, msg: Path) -> None:
        self._path_xy = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _on_pose(self, msg: PoseStamped) -> None:
        self._pose = msg

    def _on_distance(self, msg: Float32) -> None:
        self._distance_m = float(msg.data)

    def _on_debug(self, msg: Float64MultiArray) -> None:
        if msg.data:
            self._debug = list(msg.data)

    def _world_to_pixel(self, x: float, y: float, origin: Tuple[int, int]) -> Tuple[int, int]:
        ox, oy = origin
        px = int(ox + x * self.ppm)
        py = int(oy - y * self.ppm)  # image y down
        return px, py

    def _draw(self) -> None:
        img = np.full((self.canvas_size, self.canvas_size, 3), 30, dtype=np.uint8)
        origin = (self.canvas_size // 2, self.canvas_size // 2)

        # Auto zoom/center so small IMU paths are visible
        if self._path_xy:
            xs = [p[0] for p in self._path_xy]
            ys = [p[1] for p in self._path_xy]
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)
            cx = 0.5 * (min_x + max_x)
            cy = 0.5 * (min_y + max_y)
            span = max(max_x - min_x, max_y - min_y, 0.05)
            # Fit path into ~70% of canvas; allow zoom-in for cm-scale motion
            self.ppm = float(np.clip((self.canvas_size * 0.7) / span, 40.0, 2500.0))
            origin = (
                int(self.canvas_size / 2 - cx * self.ppm),
                int(self.canvas_size / 2 + cy * self.ppm),
            )

        if self.show_grid:
            spacing_px = max(int(self.grid_spacing_m * self.ppm), 8)
            for x in range(origin[0] % spacing_px, self.canvas_size, spacing_px):
                cv2.line(img, (x, 0), (x, self.canvas_size), (45, 45, 45), 1)
            for y in range(origin[1] % spacing_px, self.canvas_size, spacing_px):
                cv2.line(img, (0, y), (self.canvas_size, y), (45, 45, 45), 1)

        cv2.arrowedLine(
            img, origin, (origin[0] + 40, origin[1]), (80, 80, 200), 2, tipLength=0.3
        )
        cv2.arrowedLine(
            img, origin, (origin[0], origin[1] - 40), (80, 200, 80), 2, tipLength=0.3
        )
        cv2.putText(img, 'X', (origin[0] + 44, origin[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 80, 200), 1)
        cv2.putText(img, 'Y', (origin[0] + 4, origin[1] - 44), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 200, 80), 1)

        unique_span = 0.0
        if self._path_xy:
            xs = [p[0] for p in self._path_xy]
            ys = [p[1] for p in self._path_xy]
            unique_span = max(max(xs) - min(xs), max(ys) - min(ys))

        if len(self._path_xy) >= 2 and unique_span > 1e-4:
            pts = np.array(
                [self._world_to_pixel(x, y, origin) for x, y in self._path_xy],
                dtype=np.int32,
            )
            cv2.polylines(img, [pts], False, (0, 200, 255), self.trail_thickness, cv2.LINE_AA)
            # Mark start
            cv2.circle(img, tuple(pts[0]), 5, (255, 180, 80), -1, cv2.LINE_AA)

        if self._pose is not None:
            x = self._pose.pose.position.x
            y = self._pose.pose.position.y
            q = self._pose.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            px, py = self._world_to_pixel(x, y, origin)
            cv2.circle(img, (px, py), 7, (0, 255, 120), -1, cv2.LINE_AA)
            hx = int(px + 20 * math.cos(yaw))
            hy = int(py - 20 * math.sin(yaw))
            cv2.arrowedLine(img, (px, py), (hx, hy), (0, 255, 120), 2, tipLength=0.35)

        dt_ms = self._debug[0] * 1000.0 if len(self._debug) > 0 else 0.0
        ax_b = self._debug[1] if len(self._debug) > 1 else 0.0
        ay_b = self._debug[2] if len(self._debug) > 2 else 0.0
        vx = self._debug[5] if len(self._debug) > 5 else 0.0
        vy = self._debug[6] if len(self._debug) > 6 else 0.0
        zupt = self._debug[7] >= 0.5 if len(self._debug) > 7 else False
        still = self._debug[8] if len(self._debug) > 8 else 0.0
        dt_raw_ms = self._debug[12] * 1000.0 if len(self._debug) > 12 else 0.0
        clamped = self._debug[13] >= 0.5 if len(self._debug) > 13 else False
        ax_raw = self._debug[14] if len(self._debug) > 14 else 0.0
        ay_raw = self._debug[15] if len(self._debug) > 15 else 0.0
        integ = int(self._debug[19]) if len(self._debug) > 19 else 0

        hint = 'MOVE / shake robot to draw path'
        if self._distance_m < 0.005 and unique_span < 0.005:
            hint = 'No motion yet — push robot (still 1s at start for calib)'
        elif zupt:
            hint = 'ZUPT on (stationary) — move to continue path'
        elif clamped:
            hint = 'dt clamped (packet gap) — check CPU / WiFi'

        hud = [
            'PetCam create_map',
            f'distance: {self._distance_m:.3f} m',
            f'path pts: {len(self._path_xy)}  span: {unique_span*100:.1f} cm  integ: {integ}',
            f'dt: {dt_ms:.1f} ms (raw {dt_raw_ms:.1f})  v: ({vx:.2f},{vy:.2f})',
            f'a_raw: ({ax_raw:.2f},{ay_raw:.2f})  a_xy: ({ax_b:.2f},{ay_b:.2f})',
            f'zupt: {"ON" if zupt else "off"} still:{still:.2f}s  scale: {self.ppm:.0f} px/m',
            hint,
        ]
        for i, line in enumerate(hud):
            color = (80, 220, 255) if i == len(hud) - 1 else (220, 220, 220)
            cv2.putText(
                img,
                line,
                (12, 24 + i * 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                color,
                1,
                cv2.LINE_AA,
            )

        cv2.imshow(self.window_name, img)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27):
            self.get_logger().info('Map viewer closed by user')
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
