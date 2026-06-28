"""Detect red cones and publish planner goals for Parte C."""

from __future__ import annotations

import json
import math
from collections import deque
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float
    source: str


@dataclass(frozen=True)
class ConeCandidate:
    contour: Any
    area: float
    bbox: tuple[int, int, int, int]
    centroid: tuple[float, float]
    fill_ratio: float
    aspect_ratio: float
    triangularity: float


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def _angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


class RedConeDetectorNode(Node):
    """HSV red-cone detector with temporal validation and planner integration.

    The node estimates a cone position from the image using the known cone
    height, transforms it with the current probabilistic pose estimate, and
    publishes a ``/goal_pose`` only after several consistent observations.  This
    deliberately feeds the global planner instead of commanding the robot toward
    the pixel bearing, so walls and grates remain obstacles in the costmap.
    """

    def __init__(self) -> None:
        super().__init__('red_cone_detector_node')

        self.declare_parameter('image_topic', '/tb4_0/oakd/rgb/preview/image_raw')
        self.declare_parameter('camera_info_topic', '/tb4_0/oakd/rgb/preview/camera_info')
        self.declare_parameter('pose_topic', '/pose_estimate')
        self.declare_parameter('detections_topic', '/red_cone/detections')
        self.declare_parameter('debug_image_topic', '/red_cone/debug_image')
        self.declare_parameter('mask_image_topic', '/red_cone/mask')
        self.declare_parameter('markers_topic', '/red_cone/markers')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('use_static_calibration', True)
        self.declare_parameter('camera_matrix', [203.14, 0.0, 122.57, 0.0, 361.13, 123.33, 0.0, 0.0, 1.0])
        self.declare_parameter('cone_height_m', 0.30)
        self.declare_parameter('goal_standoff_m', 0.35)
        self.declare_parameter('camera_yaw_offset_rad', 0.0)
        self.declare_parameter('camera_x_offset_m', 0.05)
        self.declare_parameter('camera_y_offset_m', 0.0)
        self.declare_parameter('min_area_px', 120.0)
        self.declare_parameter('min_height_px', 16)
        self.declare_parameter('hue_low_1', 0)
        self.declare_parameter('hue_high_1', 28)
        self.declare_parameter('hue_low_2', 155)
        self.declare_parameter('hue_high_2', 179)
        self.declare_parameter('sat_low', 60)
        self.declare_parameter('value_low', 35)
        self.declare_parameter('min_fill_ratio', 0.22)
        self.declare_parameter('max_fill_ratio', 0.92)
        self.declare_parameter('min_aspect_ratio', 0.25)
        self.declare_parameter('max_aspect_ratio', 1.35)
        self.declare_parameter('min_triangularity', 0.38)
        self.declare_parameter('max_range_m', 4.0)
        self.declare_parameter('min_range_m', 0.20)
        self.declare_parameter('history_size', 8)
        self.declare_parameter('confirmations_required', 4)
        self.declare_parameter('stable_radius_m', 0.35)
        self.declare_parameter('republish_period_s', 5.0)
        self.declare_parameter('auto_publish_goal', True)
        self.declare_parameter('publish_debug_image', True)

        self._bridge = CvBridge()
        self._latest_camera_model: CameraModel | None = None
        self._static_camera_model = self._load_static_camera_model()

        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._have_pose = False

        self._history: deque[tuple[float, float, float]] = deque(
            maxlen=int(self.get_parameter('history_size').value))
        self._last_goal_publish_time = -1.0e9

        self._detections_pub = self.create_publisher(
            String, str(self.get_parameter('detections_topic').value), 10)
        self._goal_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter('goal_topic').value), 10)
        self._markers_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter('markers_topic').value), 10)
        self._debug_pub = self.create_publisher(
            Image,
            str(self.get_parameter('debug_image_topic').value),
            QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE),
        )
        self._mask_pub = self.create_publisher(
            Image,
            str(self.get_parameter('mask_image_topic').value),
            QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE),
        )

        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter('pose_topic').value),
            self._on_pose,
            10,
        )

        self.get_logger().info(
            'Red cone detector ready: image=%s pose=%s goal=%s'
            % (
                self.get_parameter('image_topic').value,
                self.get_parameter('pose_topic').value,
                self.get_parameter('goal_topic').value,
            )
        )

    def _load_static_camera_model(self) -> CameraModel:
        values = [float(value) for value in self.get_parameter('camera_matrix').value]
        if len(values) != 9:
            raise ValueError('camera_matrix must contain 9 values')
        return CameraModel(
            fx=values[0],
            fy=values[4],
            cx=values[2],
            cy=values[5],
            source='static_parameters',
        )

    def _camera_model(self) -> CameraModel | None:
        if bool(self.get_parameter('use_static_calibration').value):
            return self._static_camera_model
        return self._latest_camera_model

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if not any(msg.k):
            return
        self._latest_camera_model = CameraModel(
            fx=float(msg.k[0]),
            fy=float(msg.k[4]),
            cx=float(msg.k[2]),
            cy=float(msg.k[5]),
            source='camera_info',
        )

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._robot_x = float(p.x)
        self._robot_y = float(p.y)
        self._robot_yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
        self._have_pose = True

    def _on_image(self, msg: Image) -> None:
        model = self._camera_model()
        if model is None:
            self.get_logger().warn('Skipping image: no camera calibration yet', throttle_duration_sec=5.0)
            return

        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f'Could not convert image: {exc}')
            return

        mask = self._red_mask(image)
        candidates = self._find_candidates(mask)
        best = max(candidates, key=lambda c: c.area, default=None)

        annotated = image.copy()
        detections: list[dict[str, Any]] = []
        goal_msg: PoseStamped | None = None

        if best is not None:
            relative = self._estimate_relative(best, model)
            if relative is not None:
                rel_x, rel_y, rng, bearing = relative
                map_point = self._relative_to_map(rel_x, rel_y) if self._have_pose else None
                stable = False
                if map_point is not None:
                    stable = self._update_history(map_point[0], map_point[1])
                    if stable and bool(self.get_parameter('auto_publish_goal').value):
                        goal_msg = self._make_goal(map_point[0], map_point[1])
                        self._maybe_publish_goal(goal_msg)

                detections.append({
                    'range_m': rng,
                    'bearing_rad': bearing,
                    'base_link_approx': {'x': rel_x, 'y': rel_y},
                    'map': None if map_point is None else {'x': map_point[0], 'y': map_point[1]},
                    'stable': stable,
                    'bbox': list(best.bbox),
                    'area_px': best.area,
                    'fill_ratio': best.fill_ratio,
                    'aspect_ratio': best.aspect_ratio,
                    'triangularity': best.triangularity,
                })
                self._draw_detection(annotated, best, rng, bearing, stable)

        self._publish_detection_payload(msg, model, detections, len(candidates))
        self._publish_markers(msg, detections, goal_msg)

        if bool(self.get_parameter('publish_debug_image').value):
            debug = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            debug.header = msg.header
            self._debug_pub.publish(debug)
            mask_msg = self._bridge.cv2_to_imgmsg(mask, encoding='mono8')
            mask_msg.header = msg.header
            self._mask_pub.publish(mask_msg)

    def _red_mask(self, image: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        sat_low = int(self.get_parameter('sat_low').value)
        value_low = int(self.get_parameter('value_low').value)
        lower1 = np.array([int(self.get_parameter('hue_low_1').value), sat_low, value_low], dtype=np.uint8)
        upper1 = np.array([int(self.get_parameter('hue_high_1').value), 255, 255], dtype=np.uint8)
        lower2 = np.array([int(self.get_parameter('hue_low_2').value), sat_low, value_low], dtype=np.uint8)
        upper2 = np.array([int(self.get_parameter('hue_high_2').value), 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _find_candidates(self, mask: np.ndarray) -> list[ConeCandidate]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates: list[ConeCandidate] = []
        min_area = float(self.get_parameter('min_area_px').value)
        min_height = int(self.get_parameter('min_height_px').value)
        min_fill = float(self.get_parameter('min_fill_ratio').value)
        max_fill = float(self.get_parameter('max_fill_ratio').value)
        min_aspect = float(self.get_parameter('min_aspect_ratio').value)
        max_aspect = float(self.get_parameter('max_aspect_ratio').value)
        min_tri = float(self.get_parameter('min_triangularity').value)

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if h < min_height or w <= 0:
                continue
            fill_ratio = area / float(w * h)
            aspect_ratio = w / float(h)
            hull = cv2.convexHull(contour)
            hull_area = max(float(cv2.contourArea(hull)), 1.0)
            triangularity = area / hull_area
            if not (min_fill <= fill_ratio <= max_fill):
                continue
            if not (min_aspect <= aspect_ratio <= max_aspect):
                continue
            if triangularity < min_tri:
                continue
            moments = cv2.moments(contour)
            if abs(moments['m00']) < 1.0e-6:
                continue
            cx = float(moments['m10'] / moments['m00'])
            cy = float(moments['m01'] / moments['m00'])
            candidates.append(ConeCandidate(
                contour=contour,
                area=area,
                bbox=(x, y, w, h),
                centroid=(cx, cy),
                fill_ratio=fill_ratio,
                aspect_ratio=aspect_ratio,
                triangularity=triangularity,
            ))
        return candidates

    def _estimate_relative(self, candidate: ConeCandidate, model: CameraModel
                           ) -> tuple[float, float, float, float] | None:
        x, _y, w, h = candidate.bbox
        cone_height = float(self.get_parameter('cone_height_m').value)
        rng = cone_height * model.fy / max(float(h), 1.0)
        if not (float(self.get_parameter('min_range_m').value) <= rng <= float(self.get_parameter('max_range_m').value)):
            return None
        u = x + 0.5 * w
        bearing = math.atan2(u - model.cx, model.fx) + float(self.get_parameter('camera_yaw_offset_rad').value)
        rel_x = rng * math.cos(bearing) + float(self.get_parameter('camera_x_offset_m').value)
        rel_y = rng * math.sin(bearing) + float(self.get_parameter('camera_y_offset_m').value)
        return rel_x, rel_y, rng, bearing

    def _relative_to_map(self, rel_x: float, rel_y: float) -> tuple[float, float]:
        c = math.cos(self._robot_yaw)
        s = math.sin(self._robot_yaw)
        return (
            self._robot_x + rel_x * c - rel_y * s,
            self._robot_y + rel_x * s + rel_y * c,
        )

    def _update_history(self, mx: float, my: float) -> bool:
        now = self.get_clock().now().nanoseconds * 1.0e-9
        self._history.append((mx, my, now))
        radius = float(self.get_parameter('stable_radius_m').value)
        confirmations = int(self.get_parameter('confirmations_required').value)
        close = [(x, y) for x, y, _t in self._history if math.hypot(x - mx, y - my) <= radius]
        return len(close) >= confirmations

    def _make_goal(self, cone_x: float, cone_y: float) -> PoseStamped:
        standoff = float(self.get_parameter('goal_standoff_m').value)
        dx = cone_x - self._robot_x
        dy = cone_y - self._robot_y
        dist = max(math.hypot(dx, dy), 1.0e-6)
        goal_x = cone_x - standoff * dx / dist
        goal_y = cone_y - standoff * dy / dist
        yaw = math.atan2(cone_y - goal_y, cone_x - goal_x)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = goal_x
        msg.pose.position.y = goal_y
        qx, qy, qz, qw = _quat_from_yaw(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def _maybe_publish_goal(self, goal: PoseStamped) -> None:
        now = self.get_clock().now().nanoseconds * 1.0e-9
        if now - self._last_goal_publish_time < float(self.get_parameter('republish_period_s').value):
            return
        self._last_goal_publish_time = now
        self._goal_pub.publish(goal)
        self.get_logger().info(
            'Published red-cone goal at map=(%.2f, %.2f)'
            % (goal.pose.position.x, goal.pose.position.y)
        )

    def _publish_detection_payload(self, image_msg: Image, model: CameraModel,
                                   detections: list[dict[str, Any]],
                                   candidate_count: int) -> None:
        stamp = image_msg.header.stamp
        payload = {
            'stamp': {'sec': int(stamp.sec), 'nanosec': int(stamp.nanosec)},
            'frame_id': image_msg.header.frame_id,
            'camera_model_source': model.source,
            'count': len(detections),
            'candidate_count': candidate_count,
            'detections': detections,
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self._detections_pub.publish(msg)

    def _publish_markers(self, image_msg: Image, detections: list[dict[str, Any]],
                         goal: PoseStamped | None) -> None:
        arr = MarkerArray()
        delete = Marker()
        delete.header.frame_id = 'map'
        delete.header.stamp = self.get_clock().now().to_msg()
        delete.ns = 'red_cone'
        delete.id = 0
        delete.action = Marker.DELETEALL
        arr.markers.append(delete)

        for idx, detection in enumerate(detections):
            point = detection.get('map')
            if point is None:
                continue
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'red_cone'
            m.id = idx + 1
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(point['x'])
            m.pose.position.y = float(point['y'])
            m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0
            m.scale.x = 0.18
            m.scale.y = 0.18
            m.scale.z = float(self.get_parameter('cone_height_m').value)
            m.color.r = 1.0
            m.color.g = 0.05
            m.color.b = 0.02
            m.color.a = 0.85
            m.lifetime.sec = 1
            arr.markers.append(m)

        if goal is not None:
            gm = Marker()
            gm.header = goal.header
            gm.ns = 'red_cone_goal'
            gm.id = 100
            gm.type = Marker.ARROW
            gm.action = Marker.ADD
            gm.pose = goal.pose
            gm.scale.x = 0.35
            gm.scale.y = 0.06
            gm.scale.z = 0.06
            gm.color.r = 1.0
            gm.color.g = 0.8
            gm.color.b = 0.0
            gm.color.a = 1.0
            gm.lifetime.sec = 2
            arr.markers.append(gm)

        self._markers_pub.publish(arr)

    def _draw_detection(self, image: np.ndarray, candidate: ConeCandidate,
                        rng: float, bearing: float, stable: bool) -> None:
        x, y, w, h = candidate.bbox
        color = (0, 255, 0) if stable else (0, 165, 255)
        cv2.drawContours(image, [candidate.contour], -1, color, 2)
        cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
        text = 'red cone %.2fm %.1fdeg' % (rng, math.degrees(_angle_diff(bearing, 0.0)))
        if stable:
            text += ' stable'
        cv2.putText(image, text, (x, max(18, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RedConeDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
