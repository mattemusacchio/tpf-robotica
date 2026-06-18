"""Progressive ROS demo for building the occupancy map over time.

This node is intentionally a visualization/demo surface: it reads the optimized
Graph SLAM trajectory plus the laberinto bag, integrates LaserScan messages in
order, and publishes an incrementally growing /map together with the optimized
path and landmarks.
"""

from __future__ import annotations

import json
import math
import sqlite3
import threading
import time
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .occupancy_grid_builder import MappingConfig, OccupancyGridBuilder


class ProgressiveMappingDemoNode(Node):
    """Publish an incrementally built map from bag scans and optimized poses."""

    def __init__(self) -> None:
        super().__init__('progressive_mapping_demo_node')

        self.declare_parameter('bag_path', 'data/rosbags/laberinto')
        self.declare_parameter('optimized_graph_path', 'log/laberinto_optimized_graph.json')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('path_topic', '/slam/demo_path')
        self.declare_parameter('landmarks_topic', '/slam/demo_landmarks')
        self.declare_parameter('status_topic', '/slam/demo_status')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('resolution', 0.08)
        self.declare_parameter('max_range_m', 5.0)
        self.declare_parameter('scan_stride', 20)
        self.declare_parameter('beam_stride', 10)
        self.declare_parameter('inflate_radius_m', 0.08)
        self.declare_parameter('min_occupied_component_cells', 4)
        self.declare_parameter('use_tf_static', False)
        self.declare_parameter('laser_x_m', -0.04)
        self.declare_parameter('laser_y_m', 0.0)
        self.declare_parameter('laser_yaw_rad', 1.5707963267948966)
        self.declare_parameter('publish_every_scans', 10)
        self.declare_parameter('playback_rate_hz', 8.0)
        self.declare_parameter('max_processed_scans', 0)
        self.declare_parameter('loop', False)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.publish_every_scans = max(1, int(self.get_parameter('publish_every_scans').value))
        self.playback_rate_hz = max(0.0, float(self.get_parameter('playback_rate_hz').value))
        self.max_processed_scans = max(0, int(self.get_parameter('max_processed_scans').value))
        self.loop = bool(self.get_parameter('loop').value)
        self.stop_event = threading.Event()
        self.worker: threading.Thread | None = None
        self.current_stamp = 0.0
        self.done = False

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_pub = self.create_publisher(OccupancyGrid, str(self.get_parameter('map_topic').value), map_qos)
        self.path_pub = self.create_publisher(PathMsg, str(self.get_parameter('path_topic').value), map_qos)
        self.landmarks_pub = self.create_publisher(MarkerArray, str(self.get_parameter('landmarks_topic').value), map_qos)
        self.status_pub = self.create_publisher(String, str(self.get_parameter('status_topic').value), map_qos)

        self.builder = self._create_builder()
        self.create_timer(1.0, self._publish_static_overlays)
        self.worker = threading.Thread(target=self._run_demo_loop, daemon=True)
        self.worker.start()

        self.get_logger().info(
            'Progressive mapping demo started: bag=%s graph=%s scan_stride=%d beam_stride=%d'
            % (
                self.get_parameter('bag_path').value,
                self.get_parameter('optimized_graph_path').value,
                self.builder.config.scan_stride,
                self.builder.config.beam_stride,
            )
        )

    def destroy_node(self) -> bool:
        self.stop_event.set()
        if self.worker and self.worker.is_alive():
            self.worker.join(timeout=2.0)
        return super().destroy_node()

    def _create_builder(self) -> OccupancyGridBuilder:
        config = MappingConfig(
            resolution=float(self.get_parameter('resolution').value),
            max_range_m=float(self.get_parameter('max_range_m').value),
            scan_stride=max(1, int(self.get_parameter('scan_stride').value)),
            beam_stride=max(1, int(self.get_parameter('beam_stride').value)),
            inflate_radius_m=max(0.0, float(self.get_parameter('inflate_radius_m').value)),
            min_occupied_component_cells=max(0, int(self.get_parameter('min_occupied_component_cells').value)),
            use_tf_static=bool(self.get_parameter('use_tf_static').value),
            laser_x_m=float(self.get_parameter('laser_x_m').value),
            laser_y_m=float(self.get_parameter('laser_y_m').value),
            laser_yaw_rad=float(self.get_parameter('laser_yaw_rad').value),
        )
        return OccupancyGridBuilder(
            Path(str(self.get_parameter('bag_path').value)),
            Path(str(self.get_parameter('optimized_graph_path').value)),
            config,
        )

    def _run_demo_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                self._run_once()
            except Exception as exc:  # pragma: no cover - demo robustness
                self.get_logger().error(f'Progressive mapping demo failed: {exc}')
                self._publish_status(error=str(exc))
                return
            if not self.loop:
                self.done = True
                self._publish_all(force=True)
                self._publish_status(done=True)
                return
            self.builder = self._create_builder()
            self.current_stamp = 0.0

    def _run_once(self) -> None:
        bag_path = Path(str(self.get_parameter('bag_path').value))
        db3_files = sorted(bag_path.glob('*.db3')) if bag_path.is_dir() else [bag_path]
        if not db3_files:
            raise FileNotFoundError(f'No .db3 files found in {bag_path}')
        processed_since_publish = 0
        for db3_file in db3_files:
            with sqlite3.connect(str(db3_file)) as connection:
                topic = connection.execute(
                    'SELECT id, type FROM topics WHERE name = ?',
                    (self.builder.config.scan_topic,),
                ).fetchone()
                if topic is None:
                    raise KeyError(f'Scan topic {self.builder.config.scan_topic!r} not found in {db3_file}')
                topic_id, topic_type = int(topic[0]), str(topic[1])
                scan_type = get_message(topic_type)
                self.builder._load_laser_transform_if_available(connection, scan_type, topic_id)
                query = '''
                    SELECT timestamp, data
                    FROM messages
                    WHERE topic_id = ?
                    ORDER BY timestamp, id
                '''
                for raw_index, (_timestamp, data) in enumerate(connection.execute(query, (topic_id,))):
                    if self.stop_event.is_set():
                        return
                    self.builder.stats['scan_messages'] += 1
                    if raw_index % self.builder.config.scan_stride != 0:
                        continue
                    scan = deserialize_message(data, scan_type)
                    self.current_stamp = float(scan.header.stamp.sec) + 1e-9 * float(scan.header.stamp.nanosec)
                    self.builder._process_scan(scan)
                    processed_since_publish += 1
                    if processed_since_publish >= self.publish_every_scans:
                        processed_since_publish = 0
                        self._publish_all()
                    if self.max_processed_scans and self.builder.stats['processed_scans'] >= self.max_processed_scans:
                        self._publish_all(force=True)
                        return
                    if self.playback_rate_hz > 0.0:
                        time.sleep(1.0 / self.playback_rate_hz)
        self._publish_all(force=True)

    def _publish_static_overlays(self) -> None:
        self._publish_path()
        self._publish_landmarks()
        self._publish_status(done=self.done)

    def _publish_all(self, force: bool = False) -> None:
        if self.builder.stats['processed_scans'] == 0 and not force:
            return
        self._publish_map()
        self._publish_path()
        self._publish_landmarks()
        self._publish_status(done=self.done)

    def _publish_map(self) -> None:
        occupancy = self.builder._occupancy_values()
        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = float(self.builder.config.resolution)
        msg.info.width = int(self.builder.width)
        msg.info.height = int(self.builder.height)
        msg.info.origin.position.x = float(self.builder.origin_x)
        msg.info.origin.position.y = float(self.builder.origin_y)
        msg.info.origin.orientation.w = 1.0
        msg.data = [int(value) for value in occupancy.reshape(-1)]
        self.map_pub.publish(msg)

    def _publish_path(self) -> None:
        msg = PathMsg()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.current_stamp <= 0.0:
            poses = self.builder.trajectory.poses[:1]
        elif self.done:
            poses = self.builder.trajectory.poses
        else:
            poses = [pose for pose in self.builder.trajectory.poses if pose.stamp <= self.current_stamp]
            if not poses:
                poses = self.builder.trajectory.poses[:1]
        for pose in poses:
            stamped = PoseStamped()
            stamped.header = msg.header
            stamped.pose.position.x = pose.x
            stamped.pose.position.y = pose.y
            half_yaw = 0.5 * pose.theta
            stamped.pose.orientation.z = float(math.sin(half_yaw))
            stamped.pose.orientation.w = float(math.cos(half_yaw))
            msg.poses.append(stamped)
        self.path_pub.publish(msg)

    def _publish_landmarks(self) -> None:
        marker_array = MarkerArray()
        delete_all = Marker()
        delete_all.header.frame_id = self.map_frame
        delete_all.header.stamp = self.get_clock().now().to_msg()
        delete_all.ns = 'demo_landmarks'
        delete_all.id = 0
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)
        for index, landmark in enumerate(self.builder.landmarks, start=1):
            sphere = Marker()
            sphere.header = delete_all.header
            sphere.ns = 'demo_landmark_sphere'
            sphere.id = index
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = float(landmark['x'])
            sphere.pose.position.y = float(landmark['y'])
            sphere.pose.position.z = 0.1
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.18
            sphere.scale.y = 0.18
            sphere.scale.z = 0.18
            sphere.color.r = 1.0
            sphere.color.g = 0.55
            sphere.color.b = 0.0
            sphere.color.a = 0.95
            marker_array.markers.append(sphere)

            label = Marker()
            label.header = delete_all.header
            label.ns = 'demo_landmark_label'
            label.id = 10_000 + index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(landmark['x'])
            label.pose.position.y = float(landmark['y'])
            label.pose.position.z = 0.35
            label.pose.orientation.w = 1.0
            label.scale.z = 0.22
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f"id={landmark['id']}"
            marker_array.markers.append(label)
        self.landmarks_pub.publish(marker_array)

    def _publish_status(self, *, done: bool = False, error: str | None = None) -> None:
        msg = String()
        payload: dict[str, Any] = {
            'done': done,
            'processed_scans': self.builder.stats['processed_scans'],
            'scan_messages': self.builder.stats['scan_messages'],
            'valid_rays': self.builder.stats['valid_rays'],
            'map_width': self.builder.width,
            'map_height': self.builder.height,
            'laser_transform': self.builder.summary()['laser_transform'],
        }
        if error:
            payload['error'] = error
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ProgressiveMappingDemoNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
