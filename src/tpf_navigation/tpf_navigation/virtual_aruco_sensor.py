"""Virtual ArUco landmark sensor for Gazebo simulation.

Reads landmark positions from a YAML config, listens to /odom for the robot
pose, and publishes synthetic range/bearing detections in the same JSON format
as the real aruco_detector_node.  Includes LOS raycasting against /map so that
landmarks behind walls are correctly occluded.
"""

from __future__ import annotations

import json
import math
from typing import Any

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def _yaw_from_quaternion(q: Any) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _bresenham_clear(grid: np.ndarray, x0: int, y0: int, x1: int, y1: int,
                     occupied_thresh: int = 50) -> bool:
    """Return True if the Bresenham line from (x0,y0) to (x1,y1) is free."""
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    h, w = grid.shape
    x, y = x0, y0
    while True:
        if 0 <= y < h and 0 <= x < w:
            if grid[y, x] >= occupied_thresh:
                return False
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return True


class VirtualArucoSensor(Node):
    def __init__(self) -> None:
        super().__init__('virtual_aruco_sensor')

        self.declare_parameter('landmarks_file', '')
        self.declare_parameter('detections_topic', '/aruco/detections')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('markers_viz_topic', '/virtual_landmarks_viz')
        self.declare_parameter('max_range_m', 3.5)
        self.declare_parameter('sigma_range', 0.05)
        self.declare_parameter('sigma_bearing', 0.02)
        self.declare_parameter('publish_rate_hz', 10.0)

        lm_file = str(self.get_parameter('landmarks_file').value)
        if not lm_file:
            raise RuntimeError('landmarks_file parameter is required')

        with open(lm_file) as f:
            data = yaml.safe_load(f)
        self._landmarks: list[dict] = data['landmarks']
        self.get_logger().info(f'Loaded {len(self._landmarks)} virtual landmarks from {lm_file}')

        self._max_range = float(self.get_parameter('max_range_m').value)
        self._sigma_range = float(self.get_parameter('sigma_range').value)
        self._sigma_bearing = float(self.get_parameter('sigma_bearing').value)
        self._rng = np.random.default_rng()

        self._robot_x: float | None = None
        self._robot_y: float | None = None
        self._robot_yaw: float | None = None

        self._occ_grid: np.ndarray | None = None
        self._map_origin_x: float = 0.0
        self._map_origin_y: float = 0.0
        self._map_resolution: float = 0.05

        self._det_pub = self.create_publisher(
            String, str(self.get_parameter('detections_topic').value), 10)
        self._viz_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter('markers_viz_topic').value), 10)

        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._on_odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('map_topic').value),
            self._on_map,
            10,
        )

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self._publish_detections)
        self._publish_landmark_markers()

    def _on_odom(self, msg: Odometry) -> None:
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        self._robot_yaw = _yaw_from_quaternion(msg.pose.pose.orientation)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map_origin_x = msg.info.origin.position.x
        self._map_origin_y = msg.info.origin.position.y
        self._map_resolution = msg.info.resolution
        self._occ_grid = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))
        self.get_logger().info(
            f'Received occupancy map {msg.info.width}x{msg.info.height} @ {msg.info.resolution}m/cell',
            once=True,
        )

    def _world_to_cell(self, wx: float, wy: float) -> tuple[int, int]:
        col = int((wx - self._map_origin_x) / self._map_resolution)
        row = int((wy - self._map_origin_y) / self._map_resolution)
        return col, row

    def _has_los(self, lm_x: float, lm_y: float) -> bool:
        if self._occ_grid is None:
            return True  # no map yet — fail open
        rc, rr = self._world_to_cell(self._robot_x, self._robot_y)
        lc, lr = self._world_to_cell(lm_x, lm_y)
        return _bresenham_clear(self._occ_grid, rc, rr, lc, lr)

    def _publish_detections(self) -> None:
        if self._robot_x is None:
            return

        if self._occ_grid is None:
            self.get_logger().warn(
                'No occupancy map received — LOS check disabled', throttle_duration_sec=10.0)

        detections: list[dict[str, Any]] = []

        for lm in self._landmarks:
            lm_x, lm_y = float(lm['x']), float(lm['y'])
            dx = lm_x - self._robot_x
            dy = lm_y - self._robot_y
            dist = math.hypot(dx, dy)

            if dist > self._max_range or dist < 0.05:
                continue
            if not self._has_los(lm_x, lm_y):
                continue

            bearing_world = math.atan2(dy, dx)
            bearing_robot = math.atan2(
                math.sin(bearing_world - self._robot_yaw),
                math.cos(bearing_world - self._robot_yaw),
            )

            noisy_range = dist + self._rng.normal(0.0, self._sigma_range)
            noisy_bearing = bearing_robot + self._rng.normal(0.0, self._sigma_bearing)

            detections.append({
                'id': int(lm['id']),
                'base_link_approx': {
                    'range': round(float(noisy_range), 4),
                    'bearing': round(float(noisy_bearing), 4),
                },
            })

        msg = String()
        msg.data = json.dumps({
            'frame_id': 'base_link',
            'count': len(detections),
            'detections': detections,
        })
        self._det_pub.publish(msg)

    def _publish_landmark_markers(self) -> None:
        ma = MarkerArray()
        for lm in self._landmarks:
            m = Marker()
            m.header.frame_id = 'map'
            m.ns = 'virtual_landmarks'
            m.id = int(lm['id'])
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(lm['x'])
            m.pose.position.y = float(lm['y'])
            m.pose.position.z = 0.5
            m.pose.orientation.w = 1.0
            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 1.0
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.b = 0.0
            m.color.a = 0.9
            m.lifetime.sec = 0

            label = Marker()
            label.header.frame_id = 'map'
            label.ns = 'virtual_landmarks_labels'
            label.id = int(lm['id'])
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(lm['x'])
            label.pose.position.y = float(lm['y'])
            label.pose.position.z = 1.2
            label.pose.orientation.w = 1.0
            label.scale.z = 0.2
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f'L{lm["id"]}'
            label.lifetime.sec = 0

            ma.markers.extend([m, label])

        self._viz_pub.publish(ma)


def main() -> None:
    rclpy.init()
    node = VirtualArucoSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
