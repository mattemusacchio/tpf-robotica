"""ROS 2 node that detects ArUco markers and publishes landmark observations."""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .aruco_geometry import optical_tvec_to_planar_observation, rotation_matrix_to_quaternion


@dataclass(frozen=True)
class CameraCalibration:
    """Camera intrinsics used by OpenCV pose estimation."""

    matrix: np.ndarray
    dist_coeffs: np.ndarray
    source: str


@dataclass(frozen=True)
class ArucoDetectionBackend:
    """OpenCV ArUco detector objects for both old and new Python APIs."""

    dictionary: Any
    parameters: Any
    detector: Any | None


class ArucoDetectorNode(Node):
    """Detect ArUco tags in TurtleBot camera frames.

    Outputs are intentionally simple and dependency-light for this first phase:

    - ``/aruco/detections``: JSON with IDs, optical-frame pose vectors and planar
      range/bearing observations for the SLAM front-end.
    - ``/aruco/poses``: PoseArray in the image optical frame for quick RViz checks.
    - ``/aruco/markers``: MarkerArray with tag IDs as text labels.
    - ``/aruco/debug_image``: annotated camera image.
    """

    def __init__(self) -> None:
        super().__init__('aruco_detector_node')

        self.declare_parameter('image_topic', '/tb4_0/oakd/rgb/preview/image_raw')
        self.declare_parameter('camera_info_topic', '/tb4_0/oakd/rgb/preview/camera_info')
        self.declare_parameter('detections_topic', '/aruco/detections')
        self.declare_parameter('poses_topic', '/aruco/poses')
        self.declare_parameter('markers_topic', '/aruco/markers')
        self.declare_parameter('debug_image_topic', '/aruco/debug_image')
        self.declare_parameter('aruco_dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_size_m', 0.0889)
        self.declare_parameter('use_static_calibration', True)
        self.declare_parameter('camera_matrix', [203.14, 0.0, 122.57, 0.0, 361.13, 123.33, 0.0, 0.0, 1.0])
        self.declare_parameter(
            'dist_coeffs',
            [-0.9904393553733826, -47.16939926147461, -0.0007601691759191453, -0.00031758102704770863, 306.0343933105469],
        )
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('draw_axes', True)
        self.declare_parameter('axis_length_m', 0.05)
        self.declare_parameter('min_marker_perimeter_rate', 0.02)
        self.declare_parameter('max_marker_perimeter_rate', 4.0)
        self.declare_parameter('adaptive_thresh_win_size_min', 3)
        self.declare_parameter('adaptive_thresh_win_size_max', 23)
        self.declare_parameter('adaptive_thresh_win_size_step', 10)
        self.declare_parameter('corner_refinement', True)

        self.marker_size_m = float(self.get_parameter('marker_size_m').value)
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.draw_axes = bool(self.get_parameter('draw_axes').value)
        self.axis_length_m = float(self.get_parameter('axis_length_m').value)
        self.use_static_calibration = bool(self.get_parameter('use_static_calibration').value)

        self.bridge = CvBridge()
        self.latest_camera_info_calibration: CameraCalibration | None = None
        self.static_calibration = self._load_static_calibration()
        self.detector = self._create_detector()

        self.detections_pub = self.create_publisher(
            String,
            str(self.get_parameter('detections_topic').value),
            10,
        )
        self.poses_pub = self.create_publisher(
            PoseArray,
            str(self.get_parameter('poses_topic').value),
            10,
        )
        self.markers_pub = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('markers_topic').value),
            10,
        )
        self.debug_image_pub = self.create_publisher(
            Image,
            str(self.get_parameter('debug_image_topic').value),
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

        self.get_logger().info(
            'Aruco detector ready: image=%s camera_info=%s dictionary=%s marker_size=%.4f m'
            % (
                self.get_parameter('image_topic').value,
                self.get_parameter('camera_info_topic').value,
                self.get_parameter('aruco_dictionary').value,
                self.marker_size_m,
            )
        )

    def _load_static_calibration(self) -> CameraCalibration:
        matrix_values = [float(value) for value in self.get_parameter('camera_matrix').value]
        if len(matrix_values) != 9:
            raise ValueError('camera_matrix must contain 9 values')
        dist_values = [float(value) for value in self.get_parameter('dist_coeffs').value]
        return CameraCalibration(
            matrix=np.array(matrix_values, dtype=np.float64).reshape((3, 3)),
            dist_coeffs=np.array(dist_values, dtype=np.float64),
            source='static_parameters',
        )

    def _create_detector(self) -> ArucoDetectionBackend:
        dictionary_name = str(self.get_parameter('aruco_dictionary').value)
        dictionary_id = getattr(cv2.aruco, dictionary_name, None)
        if dictionary_id is None:
            valid_names = sorted(name for name in dir(cv2.aruco) if name.startswith('DICT_'))
            raise ValueError(f'Unknown ArUco dictionary {dictionary_name!r}. Valid examples: {valid_names[:8]}')

        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        if hasattr(cv2.aruco, 'DetectorParameters'):
            parameters = cv2.aruco.DetectorParameters()
        else:
            parameters = cv2.aruco.DetectorParameters_create()
        parameters.minMarkerPerimeterRate = float(self.get_parameter('min_marker_perimeter_rate').value)
        parameters.maxMarkerPerimeterRate = float(self.get_parameter('max_marker_perimeter_rate').value)
        parameters.adaptiveThreshWinSizeMin = int(self.get_parameter('adaptive_thresh_win_size_min').value)
        parameters.adaptiveThreshWinSizeMax = int(self.get_parameter('adaptive_thresh_win_size_max').value)
        parameters.adaptiveThreshWinSizeStep = int(self.get_parameter('adaptive_thresh_win_size_step').value)
        if bool(self.get_parameter('corner_refinement').value):
            parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        detector_cls = getattr(cv2.aruco, 'ArucoDetector', None)
        detector = detector_cls(dictionary, parameters) if detector_cls is not None else None
        return ArucoDetectionBackend(dictionary=dictionary, parameters=parameters, detector=detector)

    def _detect_markers(self, gray_image: np.ndarray) -> tuple[Any, Any, Any]:
        """Detect markers using the OpenCV API available on this ROS install."""

        if self.detector.detector is not None:
            return self.detector.detector.detectMarkers(gray_image)
        return cv2.aruco.detectMarkers(
            gray_image,
            self.detector.dictionary,
            parameters=self.detector.parameters,
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if not any(msg.k):
            return
        dist_coeffs = np.array(msg.d, dtype=np.float64)
        if dist_coeffs.size == 0:
            dist_coeffs = self.static_calibration.dist_coeffs
        self.latest_camera_info_calibration = CameraCalibration(
            matrix=np.array(msg.k, dtype=np.float64).reshape((3, 3)),
            dist_coeffs=dist_coeffs,
            source='camera_info',
        )

    def _calibration(self) -> CameraCalibration | None:
        if self.use_static_calibration:
            return self.static_calibration
        if self.latest_camera_info_calibration is not None:
            return self.latest_camera_info_calibration
        return None

    def _on_image(self, msg: Image) -> None:
        calibration = self._calibration()
        if calibration is None:
            self.get_logger().warn('Skipping image: no camera calibration received yet', throttle_duration_sec=5.0)
            return

        try:
            bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # pragma: no cover - depends on ROS image encodings
            self.get_logger().error(f'Could not convert image: {exc}')
            return

        gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _rejected = self._detect_markers(gray)

        annotated = bgr_image.copy()
        detections: list[dict[str, Any]] = []
        pose_array = PoseArray()
        pose_array.header = msg.header
        marker_array = MarkerArray()
        marker_array.markers.append(self._delete_all_marker(msg))

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
            rvecs, tvecs, _obj_points = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.marker_size_m,
                calibration.matrix,
                calibration.dist_coeffs,
            )

            for index, marker_id_raw in enumerate(ids.flatten()):
                marker_id = int(marker_id_raw)
                rvec = np.asarray(rvecs[index][0], dtype=float)
                tvec = np.asarray(tvecs[index][0], dtype=float)
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                quaternion = rotation_matrix_to_quaternion(rotation_matrix)
                planar = optical_tvec_to_planar_observation(tvec)

                pose = Pose()
                pose.position.x = float(tvec[0])
                pose.position.y = float(tvec[1])
                pose.position.z = float(tvec[2])
                pose.orientation.x = quaternion[0]
                pose.orientation.y = quaternion[1]
                pose.orientation.z = quaternion[2]
                pose.orientation.w = quaternion[3]
                pose_array.poses.append(pose)

                marker_array.markers.extend(self._markers_for_detection(msg, marker_id, pose, index))

                if self.draw_axes:
                    cv2.drawFrameAxes(
                        annotated,
                        calibration.matrix,
                        calibration.dist_coeffs,
                        rvec,
                        tvec,
                        self.axis_length_m,
                    )

                detections.append(
                    {
                        'id': marker_id,
                        'camera_optical': {
                            'x': float(tvec[0]),
                            'y': float(tvec[1]),
                            'z': float(tvec[2]),
                            'rvec': [float(value) for value in rvec],
                            'quaternion_xyzw': [float(value) for value in quaternion],
                        },
                        'base_link_approx': planar,
                    }
                )

        stamp = msg.header.stamp
        payload = {
            'stamp': {'sec': int(stamp.sec), 'nanosec': int(stamp.nanosec)},
            'frame_id': msg.header.frame_id,
            'calibration_source': calibration.source,
            'marker_size_m': self.marker_size_m,
            'count': len(detections),
            'detections': detections,
        }
        detection_msg = String()
        detection_msg.data = json.dumps(payload, sort_keys=True)
        self.detections_pub.publish(detection_msg)
        self.poses_pub.publish(pose_array)
        self.markers_pub.publish(marker_array)

        if self.publish_debug_image:
            debug_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)

    def _delete_all_marker(self, image_msg: Image) -> Marker:
        marker = Marker()
        marker.header = image_msg.header
        marker.ns = 'aruco'
        marker.id = 0
        marker.action = Marker.DELETEALL
        return marker

    def _markers_for_detection(self, image_msg: Image, marker_id: int, pose: Pose, index: int) -> list[Marker]:
        cube = Marker()
        cube.header = image_msg.header
        cube.ns = 'aruco_pose'
        cube.id = marker_id
        cube.type = Marker.CUBE
        cube.action = Marker.ADD
        cube.pose = pose
        cube.scale.x = self.marker_size_m
        cube.scale.y = self.marker_size_m
        cube.scale.z = 0.01
        cube.color.r = 0.0
        cube.color.g = 0.8
        cube.color.b = 1.0
        cube.color.a = 0.7
        cube.lifetime.nanosec = 500_000_000

        label = Marker()
        label.header = image_msg.header
        label.ns = 'aruco_label'
        label.id = 10_000 + marker_id
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose = Pose()
        label.pose.position.x = pose.position.x
        label.pose.position.y = pose.position.y
        label.pose.position.z = pose.position.z + 0.08 + 0.01 * index
        label.pose.orientation.w = 1.0
        label.scale.z = 0.07
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        label.text = f'id={marker_id}'
        label.lifetime.nanosec = 500_000_000
        return [cube, label]


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
