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
from std_msgs.msg import Float32


class MapViewerNode(Node):
    def __init__(self) -> None:
        super().__init__('map_viewer')

        self.declare_parameter('path_topic', '/create_map/path')
        self.declare_parameter('pose_topic', '/create_map/pose')
        self.declare_parameter('distance_topic', '/create_map/distance')
        self.declare_parameter('window_name', 'PetCam create_map')
        self.declare_parameter('pixels_per_meter', 80.0)
        self.declare_parameter('trail_thickness', 2)
        self.declare_parameter('show_grid', True)
        self.declare_parameter('grid_spacing_m', 0.5)
        self.declare_parameter('canvas_size', 800)

        self.window_name = str(self.get_parameter('window_name').value)
        self.ppm = float(self.get_parameter('pixels_per_meter').value)
        self.trail_thickness = int(self.get_parameter('trail_thickness').value)
        self.show_grid = bool(self.get_parameter('show_grid').value)
        self.grid_spacing_m = float(self.get_parameter('grid_spacing_m').value)
        self.canvas_size = int(self.get_parameter('canvas_size').value)

        self._path_xy: List[Tuple[float, float]] = []
        self._pose: Optional[PoseStamped] = None
        self._distance_m = 0.0

        self.create_subscription(
            Path, self.get_parameter('path_topic').value, self._on_path, 10
        )
        self.create_subscription(
            PoseStamped, self.get_parameter('pose_topic').value, self._on_pose, 10
        )
        self.create_subscription(
            Float32, self.get_parameter('distance_topic').value, self._on_distance, 10
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

    def _world_to_pixel(self, x: float, y: float, origin: Tuple[int, int]) -> Tuple[int, int]:
        ox, oy = origin
        px = int(ox + x * self.ppm)
        py = int(oy - y * self.ppm)  # image y down
        return px, py

    def _draw(self) -> None:
        img = np.full((self.canvas_size, self.canvas_size, 3), 30, dtype=np.uint8)
        origin = (self.canvas_size // 2, self.canvas_size // 2)

        # Auto-center on path centroid if far from origin
        if self._path_xy:
            xs = [p[0] for p in self._path_xy]
            ys = [p[1] for p in self._path_xy]
            cx = 0.5 * (min(xs) + max(xs))
            cy = 0.5 * (min(ys) + max(ys))
            span = max(max(xs) - min(xs), max(ys) - min(ys), 1.0)
            # Keep ppm unless path is huge
            if span * self.ppm > self.canvas_size * 0.85:
                self.ppm = (self.canvas_size * 0.8) / span
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

        # Axes
        cv2.arrowedLine(
            img, origin, (origin[0] + 40, origin[1]), (80, 80, 200), 2, tipLength=0.3
        )
        cv2.arrowedLine(
            img, origin, (origin[0], origin[1] - 40), (80, 200, 80), 2, tipLength=0.3
        )
        cv2.putText(img, 'X', (origin[0] + 44, origin[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 80, 200), 1)
        cv2.putText(img, 'Y', (origin[0] + 4, origin[1] - 44), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 200, 80), 1)

        if len(self._path_xy) >= 2:
            pts = np.array(
                [self._world_to_pixel(x, y, origin) for x, y in self._path_xy],
                dtype=np.int32,
            )
            cv2.polylines(img, [pts], False, (0, 200, 255), self.trail_thickness, cv2.LINE_AA)

        if self._pose is not None:
            x = self._pose.pose.position.x
            y = self._pose.pose.position.y
            q = self._pose.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            px, py = self._world_to_pixel(x, y, origin)
            cv2.circle(img, (px, py), 6, (0, 255, 120), -1, cv2.LINE_AA)
            hx = int(px + 18 * math.cos(yaw))
            hy = int(py - 18 * math.sin(yaw))
            cv2.arrowedLine(img, (px, py), (hx, hy), (0, 255, 120), 2, tipLength=0.35)

        hud = [
            'PetCam create_map',
            f'distance: {self._distance_m:.3f} m',
            f'points: {len(self._path_xy)}',
            f'scale: {self.ppm:.0f} px/m',
            'q = quit window focus',
        ]
        for i, line in enumerate(hud):
            cv2.putText(
                img,
                line,
                (12, 24 + i * 22),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (220, 220, 220),
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
