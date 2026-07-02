"""Graph SLAM front-end for odometry and ArUco landmark observations.

This node intentionally does not optimize the graph yet.  It converts the raw
runtime streams into an inspectable graph that the back-end can consume next:

- pose nodes from odometry keyframes;
- odometry edges between consecutive keyframes;
- landmark nodes keyed by ArUco ID;
- visual range/bearing edges from keyframes to landmarks.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from math import atan2, cos, hypot, sin
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""

    return atan2(sin(angle), cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from a ROS quaternion."""

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return atan2(siny_cosp, cosy_cosp)


def stamp_to_sec(stamp: Any) -> float:
    """Convert a ROS stamp-like object or JSON stamp dict to seconds."""

    if isinstance(stamp, dict):
        return float(stamp.get('sec', 0)) + 1e-9 * float(stamp.get('nanosec', 0))
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


@dataclass(frozen=True)
class RobotPose2D:
    x: float
    y: float
    theta: float
    stamp: float
    frame_id: str


@dataclass
class KeyframeNode:
    id: int
    x: float
    y: float
    theta: float
    stamp: float
    frame_id: str


@dataclass
class LandmarkNode:
    id: int
    x: float
    y: float
    observations: int


@dataclass
class OdometryEdge:
    from_id: int
    to_id: int
    dx: float
    dy: float
    dtheta: float
    distance: float


@dataclass
class VisualEdge:
    keyframe_id: int
    landmark_id: int
    range: float
    bearing: float
    stamp: float


class GraphSlamFrontendNode(Node):
    """Build an initial pose-landmark graph from odometry and ArUco detections."""

    def __init__(self) -> None:
        super().__init__('graph_slam_frontend_node')

        self.declare_parameter('odom_topic', '/tb4_0/odom')
        self.declare_parameter('detections_topic', '/aruco/detections')
        self.declare_parameter('graph_topic', '/slam/graph_snapshot')
        self.declare_parameter('keyframes_topic', '/slam/keyframes')
        self.declare_parameter('landmarks_topic', '/slam/landmarks')
        self.declare_parameter('graph_edges_topic', '/slam/graph_edges')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('min_keyframe_translation_m', 0.20)
        self.declare_parameter('min_keyframe_rotation_rad', 0.35)
        self.declare_parameter('max_keyframe_period_sec', 2.0)
        self.declare_parameter('min_observation_interval_sec', 0.25)
        self.declare_parameter('max_detection_sync_age_sec', 0.35)
        self.declare_parameter('max_odom_buffer_size', 500)
        self.declare_parameter('max_landmark_range_m', 4.0)
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('graph_output_path', 'log/slam_frontend_graph.json')

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.min_keyframe_translation_m = float(self.get_parameter('min_keyframe_translation_m').value)
        self.min_keyframe_rotation_rad = float(self.get_parameter('min_keyframe_rotation_rad').value)
        self.max_keyframe_period_sec = float(self.get_parameter('max_keyframe_period_sec').value)
        self.min_observation_interval_sec = float(self.get_parameter('min_observation_interval_sec').value)
        self.max_detection_sync_age_sec = float(self.get_parameter('max_detection_sync_age_sec').value)
        self.max_odom_buffer_size = int(self.get_parameter('max_odom_buffer_size').value)
        self.max_landmark_range_m = float(self.get_parameter('max_landmark_range_m').value)
        self.graph_output_path = str(self.get_parameter('graph_output_path').value)

        self.odom_buffer: list[RobotPose2D] = []
        self.keyframes: list[KeyframeNode] = []
        self.landmarks: dict[int, LandmarkNode] = {}
        self.odom_edges: list[OdometryEdge] = []
        self.visual_edges: list[VisualEdge] = []
        self.last_observation_by_key: dict[tuple[int, int], float] = {}
        self.latest_pose: RobotPose2D | None = None

        self.graph_pub = self.create_publisher(String, str(self.get_parameter('graph_topic').value), 10)
        self.keyframes_pub = self.create_publisher(PoseArray, str(self.get_parameter('keyframes_topic').value), 10)
        self.landmarks_pub = self.create_publisher(MarkerArray, str(self.get_parameter('landmarks_topic').value), 10)
        self.graph_edges_pub = self.create_publisher(MarkerArray, str(self.get_parameter('graph_edges_topic').value), 10)

        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._on_odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('detections_topic').value),
            self._on_detections,
            10,
        )
        self.create_timer(float(self.get_parameter('publish_period_sec').value), self._publish_graph)

        self.get_logger().info(
            'Graph SLAM front-end ready: odom=%s detections=%s output=%s'
            % (
                self.get_parameter('odom_topic').value,
                self.get_parameter('detections_topic').value,
                self.graph_output_path or '<disabled>',
            )
        )

    def _on_odom(self, msg: Odometry) -> None:
        pose = self._pose_from_odom(msg)
        self.latest_pose = pose
        self.odom_buffer.append(pose)
        if len(self.odom_buffer) > self.max_odom_buffer_size:
            self.odom_buffer.pop(0)

        if not self.keyframes:
            self._add_keyframe(pose)
            return

        if self._should_add_keyframe(pose):
            self._add_keyframe(pose)

    def _on_detections(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Ignoring malformed detection JSON: {exc}')
            return

        detections = payload.get('detections', [])
        if not detections:
            return

        detection_stamp = stamp_to_sec(payload.get('stamp', {}))
        pose = self._nearest_odom_pose(detection_stamp)
        if pose is None:
            self.get_logger().warn('Ignoring detections: no odometry pose is available yet', throttle_duration_sec=5.0)
            return

        if not self.keyframes or self._should_add_keyframe(pose):
            keyframe = self._add_keyframe(pose)
        else:
            keyframe = self.keyframes[-1]

        added = 0
        for detection in detections:
            planar = detection.get('base_link_approx', {})
            marker_id = detection.get('id')
            measured_range = planar.get('range')
            measured_bearing = planar.get('bearing')
            if marker_id is None or measured_range is None or measured_bearing is None:
                continue

            marker_id = int(marker_id)
            measured_range = float(measured_range)
            measured_bearing = float(measured_bearing)
            if measured_range <= 0.0 or measured_range > self.max_landmark_range_m:
                continue

            obs_key = (keyframe.id, marker_id)
            previous_obs_stamp = self.last_observation_by_key.get(obs_key)
            if previous_obs_stamp is not None and detection_stamp - previous_obs_stamp < self.min_observation_interval_sec:
                continue
            self.last_observation_by_key[obs_key] = detection_stamp

            landmark_x = keyframe.x + measured_range * cos(keyframe.theta + measured_bearing)
            landmark_y = keyframe.y + measured_range * sin(keyframe.theta + measured_bearing)
            self._upsert_landmark(marker_id, landmark_x, landmark_y)
            self.visual_edges.append(
                VisualEdge(
                    keyframe_id=keyframe.id,
                    landmark_id=marker_id,
                    range=measured_range,
                    bearing=measured_bearing,
                    stamp=detection_stamp,
                )
            )
            added += 1

        if added:
            self.get_logger().info(
                'Graph update: keyframes=%d landmarks=%d visual_edges=%d latest_added=%d'
                % (len(self.keyframes), len(self.landmarks), len(self.visual_edges), added),
                throttle_duration_sec=2.0,
            )

    def _pose_from_odom(self, msg: Odometry) -> RobotPose2D:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        return RobotPose2D(
            x=float(position.x),
            y=float(position.y),
            theta=yaw_from_quaternion(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ),
            stamp=stamp_to_sec(msg.header.stamp),
            frame_id=msg.header.frame_id or self.map_frame,
        )

    def _nearest_odom_pose(self, stamp: float) -> RobotPose2D | None:
        if not self.odom_buffer:
            return self.latest_pose
        nearest = min(self.odom_buffer, key=lambda pose: abs(pose.stamp - stamp))
        if abs(nearest.stamp - stamp) <= self.max_detection_sync_age_sec:
            return nearest
        return self.latest_pose

    def _should_add_keyframe(self, pose: RobotPose2D) -> bool:
        last = self.keyframes[-1]
        translation = hypot(pose.x - last.x, pose.y - last.y)
        rotation = abs(normalize_angle(pose.theta - last.theta))
        elapsed = pose.stamp - last.stamp
        return (
            translation >= self.min_keyframe_translation_m
            or rotation >= self.min_keyframe_rotation_rad
            or elapsed >= self.max_keyframe_period_sec
        )

    def _add_keyframe(self, pose: RobotPose2D) -> KeyframeNode:
        keyframe = KeyframeNode(
            id=len(self.keyframes),
            x=pose.x,
            y=pose.y,
            theta=pose.theta,
            stamp=pose.stamp,
            frame_id=pose.frame_id,
        )
        if self.keyframes:
            previous = self.keyframes[-1]
            global_dx = keyframe.x - previous.x
            global_dy = keyframe.y - previous.y
            dx = cos(previous.theta) * global_dx + sin(previous.theta) * global_dy
            dy = -sin(previous.theta) * global_dx + cos(previous.theta) * global_dy
            dtheta = normalize_angle(keyframe.theta - previous.theta)
            self.odom_edges.append(
                OdometryEdge(
                    from_id=previous.id,
                    to_id=keyframe.id,
                    dx=dx,
                    dy=dy,
                    dtheta=dtheta,
                    distance=hypot(global_dx, global_dy),
                )
            )
        self.keyframes.append(keyframe)
        return keyframe

    def _upsert_landmark(self, marker_id: int, x: float, y: float) -> None:
        landmark = self.landmarks.get(marker_id)
        if landmark is None:
            self.landmarks[marker_id] = LandmarkNode(id=marker_id, x=x, y=y, observations=1)
            return
        count = landmark.observations
        landmark.x = (landmark.x * count + x) / (count + 1)
        landmark.y = (landmark.y * count + y) / (count + 1)
        landmark.observations = count + 1

    def _publish_graph(self) -> None:
        snapshot = self._snapshot()

        graph_msg = String()
        graph_msg.data = json.dumps(snapshot, sort_keys=True)
        self.graph_pub.publish(graph_msg)

        self.keyframes_pub.publish(self._keyframes_pose_array())
        self.landmarks_pub.publish(self._landmark_markers())
        self.graph_edges_pub.publish(self._edge_markers())
        self._save_graph(snapshot)

    def _snapshot(self) -> dict[str, Any]:
        return {
            'frame_id': self.map_frame,
            'counts': {
                'keyframes': len(self.keyframes),
                'landmarks': len(self.landmarks),
                'odom_edges': len(self.odom_edges),
                'visual_edges': len(self.visual_edges),
            },
            'keyframes': [asdict(keyframe) for keyframe in self.keyframes],
            'landmarks': [asdict(landmark) for landmark in sorted(self.landmarks.values(), key=lambda item: item.id)],
            'odom_edges': [asdict(edge) for edge in self.odom_edges],
            'visual_edges': [asdict(edge) for edge in self.visual_edges],
        }

    def _keyframes_pose_array(self) -> PoseArray:
        msg = PoseArray()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for keyframe in self.keyframes:
            pose = Pose()
            pose.position.x = keyframe.x
            pose.position.y = keyframe.y
            pose.position.z = 0.0
            half_yaw = 0.5 * keyframe.theta
            pose.orientation.z = sin(half_yaw)
            pose.orientation.w = cos(half_yaw)
            msg.poses.append(pose)
        return msg

    def _landmark_markers(self) -> MarkerArray:
        marker_array = MarkerArray()
        marker_array.markers.append(self._delete_all_marker('slam_landmarks'))
        for index, landmark in enumerate(sorted(self.landmarks.values(), key=lambda item: item.id), start=1):
            sphere = Marker()
            sphere.header.frame_id = self.map_frame
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = 'slam_landmark_sphere'
            sphere.id = index
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = landmark.x
            sphere.pose.position.y = landmark.y
            sphere.pose.position.z = 0.05
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.14
            sphere.scale.y = 0.14
            sphere.scale.z = 0.14
            sphere.color.r = 1.0
            sphere.color.g = 0.7
            sphere.color.b = 0.0
            sphere.color.a = 0.95
            marker_array.markers.append(sphere)

            label = Marker()
            label.header = sphere.header
            label.ns = 'slam_landmark_label'
            label.id = 10_000 + index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = landmark.x
            label.pose.position.y = landmark.y
            label.pose.position.z = 0.25
            label.pose.orientation.w = 1.0
            label.scale.z = 0.18
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f'id={landmark.id} n={landmark.observations}'
            marker_array.markers.append(label)
        return marker_array

    def _edge_markers(self) -> MarkerArray:
        marker_array = MarkerArray()
        marker_array.markers.append(self._delete_all_marker('slam_graph_edges'))
        marker_array.markers.append(
            self._line_list_marker(
                ns='slam_odom_edges',
                marker_id=1,
                color=(0.0, 0.4, 1.0, 1.0),
                width=0.035,
                points=[
                    (self.keyframes[edge.from_id].x, self.keyframes[edge.from_id].y, 0.03,
                     self.keyframes[edge.to_id].x, self.keyframes[edge.to_id].y, 0.03)
                    for edge in self.odom_edges
                ],
            )
        )
        visual_segments: list[tuple[float, float, float, float, float, float]] = []
        for edge in self.visual_edges:
            if edge.keyframe_id >= len(self.keyframes):
                continue
            landmark = self.landmarks.get(edge.landmark_id)
            if landmark is None:
                continue
            keyframe = self.keyframes[edge.keyframe_id]
            visual_segments.append((keyframe.x, keyframe.y, 0.02, landmark.x, landmark.y, 0.02))
        marker_array.markers.append(
            self._line_list_marker(
                ns='slam_visual_edges',
                marker_id=2,
                color=(0.1, 1.0, 0.2, 0.45),
                width=0.015,
                points=visual_segments,
            )
        )
        return marker_array

    def _line_list_marker(
        self,
        ns: str,
        marker_id: int,
        color: tuple[float, float, float, float],
        width: float,
        points: list[tuple[float, float, float, float, float, float]],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = width
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        for x1, y1, z1, x2, y2, z2 in points:
            marker.points.append(Point(x=x1, y=y1, z=z1))
            marker.points.append(Point(x=x2, y=y2, z=z2))
        return marker

    def _delete_all_marker(self, ns: str) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = 0
        marker.action = Marker.DELETEALL
        return marker

    def _save_graph(self, snapshot: dict[str, Any] | None = None) -> None:
        if not self.graph_output_path:
            return
        path = Path(self.graph_output_path)
        if not path.is_absolute():
            path = Path.cwd() / path
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            data = snapshot if snapshot is not None else self._snapshot()
            path.write_text(json.dumps(data, indent=2, sort_keys=True), encoding='utf-8')
        except OSError as exc:
            self.get_logger().warn(f'Could not write graph snapshot to {path}: {exc}', throttle_duration_sec=5.0)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GraphSlamFrontendNode()
    try:
        rclpy.spin(node)
    finally:
        node._save_graph()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
