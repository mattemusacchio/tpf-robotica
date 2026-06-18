"""Offline rosbag-to-graph extractor for the SLAM front-end.

This is the practical path for the 23-minute, multi-GB laberinto bag: it reads
rosbag2 directly, runs the same ArUco measurement model, and writes a graph JSON
without needing real-time ROS playback.
"""

from __future__ import annotations

import argparse
import json
import sqlite3
from dataclasses import asdict, dataclass
from math import atan2, cos, hypot, sin, sqrt
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from .graph_slam_frontend_node import (
    KeyframeNode,
    LandmarkNode,
    OdometryEdge,
    RobotPose2D,
    VisualEdge,
    normalize_angle,
    stamp_to_sec,
    yaw_from_quaternion,
)


@dataclass(frozen=True)
class OfflineBuilderConfig:
    odom_topic: str = '/tb4_0/odom'
    image_topic: str = '/tb4_0/oakd/rgb/preview/image_raw'
    aruco_dictionary: str = 'DICT_4X4_50'
    marker_size_m: float = 0.0889
    camera_matrix: tuple[float, ...] = (203.14, 0.0, 122.57, 0.0, 361.13, 123.33, 0.0, 0.0, 1.0)
    dist_coeffs: tuple[float, ...] = (-0.9904393553733826, -47.16939926147461, -0.0007601691759191453, -0.00031758102704770863, 306.0343933105469)
    min_keyframe_translation_m: float = 0.20
    min_keyframe_rotation_rad: float = 0.35
    max_keyframe_period_sec: float = 2.0
    min_observation_interval_sec: float = 0.25
    max_landmark_range_m: float = 4.0
    image_stride: int = 3
    max_images: int = 0
    progress_interval: int = 1000


class OfflineRosbagGraphBuilder:
    """Build a graph snapshot by reading odom and image messages from rosbag2."""

    def __init__(self, bag_path: Path, config: OfflineBuilderConfig) -> None:
        self.bag_path = bag_path
        self.config = config
        self.bridge = CvBridge()
        self.detector = self._create_detector()
        self.camera_matrix = np.asarray(config.camera_matrix, dtype=np.float64).reshape((3, 3))
        self.dist_coeffs = np.asarray(config.dist_coeffs, dtype=np.float64)

        self.latest_pose: RobotPose2D | None = None
        self.keyframes: list[KeyframeNode] = []
        self.landmarks: dict[int, LandmarkNode] = {}
        self.odom_edges: list[OdometryEdge] = []
        self.visual_edges: list[VisualEdge] = []
        self.last_observation_by_key: dict[tuple[int, int], float] = {}
        self.image_count = 0
        self.processed_image_count = 0
        self.detection_frame_count = 0
        self.odom_count = 0

    def build(self) -> dict[str, Any]:
        db3_files = sorted(self.bag_path.glob('*.db3')) if self.bag_path.is_dir() else [self.bag_path]
        if not db3_files:
            raise FileNotFoundError(f'No .db3 files found in {self.bag_path}')
        for db3_file in db3_files:
            self._build_from_sqlite_db(db3_file)
        return self.snapshot()

    def _build_from_sqlite_db(self, db3_file: Path) -> None:
        connection = sqlite3.connect(str(db3_file))
        try:
            topics = {
                name: {'id': topic_id, 'type': msg_type}
                for topic_id, name, msg_type, _serialization, _qos in connection.execute(
                    'SELECT id, name, type, serialization_format, offered_qos_profiles FROM topics'
                )
            }
            if self.config.odom_topic not in topics:
                raise KeyError(f'Odom topic {self.config.odom_topic!r} not found in {db3_file}')
            if self.config.image_topic not in topics:
                raise KeyError(f'Image topic {self.config.image_topic!r} not found in {db3_file}')

            odom_topic_id = int(topics[self.config.odom_topic]['id'])
            image_topic_id = int(topics[self.config.image_topic]['id'])
            odom_type = get_message(str(topics[self.config.odom_topic]['type']))
            image_type = get_message(str(topics[self.config.image_topic]['type']))
            self.image_count += int(
                connection.execute('SELECT COUNT(*) FROM messages WHERE topic_id = ?', (image_topic_id,)).fetchone()[0]
            )

            max_rn = self.config.max_images * self.config.image_stride if self.config.max_images else 0
            query = '''
                WITH sampled_images AS (
                    SELECT id
                    FROM (
                        SELECT
                            id,
                            ROW_NUMBER() OVER (ORDER BY timestamp, id) AS rn
                        FROM messages
                        WHERE topic_id = ?
                    )
                    WHERE ((rn - 1) % ?) = 0
                      AND (? = 0 OR rn <= ?)
                )
                SELECT timestamp, topic_id, data
                FROM messages
                WHERE topic_id = ?
                   OR id IN (SELECT id FROM sampled_images)
                ORDER BY timestamp, id
            '''
            for _timestamp, topic_id, data in connection.execute(
                query,
                (image_topic_id, self.config.image_stride, max_rn, max_rn, odom_topic_id),
            ):
                if int(topic_id) == odom_topic_id:
                    self.odom_count += 1
                    odom_msg = deserialize_message(data, odom_type)
                    self._on_odom(odom_msg)
                else:
                    image_msg = deserialize_message(data, image_type)
                    self._on_image(image_msg)
                    self._print_progress_if_needed()
        finally:
            connection.close()

    def _print_progress_if_needed(self) -> None:
        if (
            self.config.progress_interval
            and self.processed_image_count > 0
            and self.processed_image_count % self.config.progress_interval == 0
        ):
            print(
                'offline_graph progress: total_images=%d processed=%d detections=%d keyframes=%d landmarks=%d visual_edges=%d'
                % (
                    self.image_count,
                    self.processed_image_count,
                    self.detection_frame_count,
                    len(self.keyframes),
                    len(self.landmarks),
                    len(self.visual_edges),
                ),
                flush=True,
            )

    def snapshot(self) -> dict[str, Any]:
        return {
            'frame_id': 'map',
            'source_bag': str(self.bag_path),
            'offline_stats': {
                'odom_messages': self.odom_count,
                'image_messages': self.image_count,
                'processed_images': self.processed_image_count,
                'detection_frames': self.detection_frame_count,
                'image_stride': self.config.image_stride,
            },
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

    def _create_detector(self) -> tuple[Any, Any, Any | None]:
        dictionary_id = getattr(cv2.aruco, self.config.aruco_dictionary)
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        if hasattr(cv2.aruco, 'DetectorParameters'):
            parameters = cv2.aruco.DetectorParameters()
        else:
            parameters = cv2.aruco.DetectorParameters_create()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        detector_cls = getattr(cv2.aruco, 'ArucoDetector', None)
        detector = detector_cls(dictionary, parameters) if detector_cls is not None else None
        return dictionary, parameters, detector

    def _detect_markers(self, gray_image: np.ndarray) -> tuple[Any, Any, Any]:
        dictionary, parameters, detector = self.detector
        if detector is not None:
            return detector.detectMarkers(gray_image)
        return cv2.aruco.detectMarkers(gray_image, dictionary, parameters=parameters)

    def _on_odom(self, msg: Any) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        pose = RobotPose2D(
            x=float(position.x),
            y=float(position.y),
            theta=yaw_from_quaternion(float(orientation.x), float(orientation.y), float(orientation.z), float(orientation.w)),
            stamp=stamp_to_sec(msg.header.stamp),
            frame_id=msg.header.frame_id or 'odom',
        )
        self.latest_pose = pose
        if not self.keyframes or self._should_add_keyframe(pose):
            self._add_keyframe(pose)

    def _on_image(self, msg: Any) -> None:
        if self.latest_pose is None:
            return
        self.processed_image_count += 1
        try:
            bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _rejected = self._detect_markers(gray)
        if ids is None or len(ids) == 0:
            return

        self.detection_frame_count += 1
        pose = self.latest_pose
        if not self.keyframes or self._should_add_keyframe(pose):
            keyframe = self._add_keyframe(pose)
        else:
            keyframe = self.keyframes[-1]

        _rvecs, tvecs, _obj_points = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            self.config.marker_size_m,
            self.camera_matrix,
            self.dist_coeffs,
        )
        stamp = stamp_to_sec(msg.header.stamp)
        for index, marker_id_raw in enumerate(ids.flatten()):
            marker_id = int(marker_id_raw)
            tvec = np.asarray(tvecs[index][0], dtype=float)
            measured_range, measured_bearing = self._planar_observation(tvec)
            if measured_range <= 0.0 or measured_range > self.config.max_landmark_range_m:
                continue
            obs_key = (keyframe.id, marker_id)
            previous_stamp = self.last_observation_by_key.get(obs_key)
            if previous_stamp is not None and stamp - previous_stamp < self.config.min_observation_interval_sec:
                continue
            self.last_observation_by_key[obs_key] = stamp
            landmark_x = keyframe.x + measured_range * cos(keyframe.theta + measured_bearing)
            landmark_y = keyframe.y + measured_range * sin(keyframe.theta + measured_bearing)
            self._upsert_landmark(marker_id, landmark_x, landmark_y)
            self.visual_edges.append(
                VisualEdge(
                    keyframe_id=keyframe.id,
                    landmark_id=marker_id,
                    range=measured_range,
                    bearing=measured_bearing,
                    stamp=stamp,
                )
            )

    def _planar_observation(self, tvec: np.ndarray) -> tuple[float, float]:
        optical_x = float(tvec[0])
        optical_z = float(tvec[2])
        robot_x = optical_z
        robot_y = -optical_x
        return sqrt(robot_x * robot_x + robot_y * robot_y), atan2(robot_y, robot_x)

    def _should_add_keyframe(self, pose: RobotPose2D) -> bool:
        last = self.keyframes[-1]
        translation = hypot(pose.x - last.x, pose.y - last.y)
        rotation = abs(normalize_angle(pose.theta - last.theta))
        elapsed = pose.stamp - last.stamp
        return (
            translation >= self.config.min_keyframe_translation_m
            or rotation >= self.config.min_keyframe_rotation_rad
            or elapsed >= self.config.max_keyframe_period_sec
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


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Build a tpf_slam graph directly from a rosbag2 directory.')
    parser.add_argument('--bag', required=True, help='rosbag2 directory, e.g. data/rosbags/laberinto')
    parser.add_argument('--output', default='log/slam_frontend_graph.json')
    parser.add_argument('--image-stride', type=int, default=3, help='Process one out of N image frames.')
    parser.add_argument('--max-images', type=int, default=0, help='Optional cap on processed images; 0 means full bag.')
    parser.add_argument('--progress-interval', type=int, default=1000)
    parser.add_argument('--min-keyframe-translation-m', type=float, default=0.20)
    parser.add_argument('--min-keyframe-rotation-rad', type=float, default=0.35)
    parser.add_argument('--max-keyframe-period-sec', type=float, default=2.0)
    parser.add_argument('--min-observation-interval-sec', type=float, default=0.25)
    parser.add_argument('--max-landmark-range-m', type=float, default=4.0)
    return parser


def config_from_args(args: argparse.Namespace) -> OfflineBuilderConfig:
    return OfflineBuilderConfig(
        image_stride=max(1, int(args.image_stride)),
        max_images=max(0, int(args.max_images)),
        progress_interval=max(0, int(args.progress_interval)),
        min_keyframe_translation_m=args.min_keyframe_translation_m,
        min_keyframe_rotation_rad=args.min_keyframe_rotation_rad,
        max_keyframe_period_sec=args.max_keyframe_period_sec,
        min_observation_interval_sec=args.min_observation_interval_sec,
        max_landmark_range_m=args.max_landmark_range_m,
    )


def main(argv: list[str] | None = None) -> None:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    builder = OfflineRosbagGraphBuilder(Path(args.bag), config_from_args(args))
    snapshot = builder.build()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(snapshot, indent=2, sort_keys=True), encoding='utf-8')
    print(json.dumps({'output': str(output), 'counts': snapshot['counts'], 'offline_stats': snapshot['offline_stats']}, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
