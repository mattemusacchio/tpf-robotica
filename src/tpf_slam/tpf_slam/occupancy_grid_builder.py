"""Offline occupancy-grid builder from optimized Graph SLAM trajectory and LaserScan."""

from __future__ import annotations

import argparse
import json
import math
import sqlite3
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

try:  # pragma: no cover - exercised inside the ROS image
    from scipy.spatial import cKDTree
except Exception:  # pragma: no cover - fallback for minimal environments
    cKDTree = None

from .graph_slam_frontend_node import normalize_angle


@dataclass(frozen=True)
class MappingConfig:
    scan_topic: str = '/tb4_0/scan'
    tf_static_topic: str = '/tb4_0/tf_static'
    base_frame: str = 'base_link'
    use_tf_static: bool = True
    resolution: float = 0.05
    margin_m: float = 1.0
    max_range_m: float = 6.0
    min_range_m: float = 0.15
    scan_stride: int = 1
    beam_stride: int = 2
    # Distance-based scan selection: integrate a scan only after the robot has
    # moved/turned enough. Avoids re-stamping the same noisy returns while parked
    # (which thickens and biases walls) and gives even spatial coverage.
    min_scan_travel_m: float = 0.05
    min_scan_travel_rad: float = 0.05
    log_odds_free: float = -0.35
    log_odds_occupied: float = 0.85
    log_odds_min: float = -4.0
    log_odds_max: float = 4.0
    occupied_threshold: float = 0.65
    free_threshold: float = 0.35
    # A cell is only declared a wall once enough independent beams hit it, which
    # removes isolated speckle from single stray returns.
    min_hits_for_occupied: int = 2
    # Per-scan statistical outlier removal of endpoints (isolated LiDAR returns).
    outlier_filter: bool = True
    outlier_neighbors: int = 6
    outlier_std_mul: float = 2.0
    min_occupied_component_cells: int = 3
    inflate_radius_m: float = 0.15
    laser_x_m: float = 0.0
    laser_y_m: float = 0.0
    laser_yaw_rad: float = 0.0


@dataclass(frozen=True)
class Pose2D:
    stamp: float
    x: float
    y: float
    theta: float


class OptimizedTrajectory:
    """Timestamp-indexed optimized trajectory with linear interpolation."""

    def __init__(self, keyframes: list[dict[str, Any]]) -> None:
        if not keyframes:
            raise ValueError('optimized graph has no keyframes')
        self.poses = [
            Pose2D(
                stamp=float(row.get('stamp', 0.0)),
                x=float(row['x']),
                y=float(row['y']),
                theta=float(row['theta']),
            )
            for row in sorted(keyframes, key=lambda item: float(item.get('stamp', 0.0)))
        ]
        self.stamps = np.asarray([pose.stamp for pose in self.poses], dtype=float)

    def at(self, stamp: float) -> Pose2D:
        index = int(np.searchsorted(self.stamps, stamp))
        if index <= 0:
            return self.poses[0]
        if index >= len(self.poses):
            return self.poses[-1]
        before = self.poses[index - 1]
        after = self.poses[index]
        span = after.stamp - before.stamp
        if span <= 0.0:
            return before
        alpha = max(0.0, min(1.0, (stamp - before.stamp) / span))
        return Pose2D(
            stamp=stamp,
            x=before.x + alpha * (after.x - before.x),
            y=before.y + alpha * (after.y - before.y),
            theta=normalize_angle(before.theta + alpha * normalize_angle(after.theta - before.theta)),
        )

    def bounds(self) -> tuple[float, float, float, float]:
        xs = [pose.x for pose in self.poses]
        ys = [pose.y for pose in self.poses]
        return min(xs), max(xs), min(ys), max(ys)


class OccupancyGridBuilder:
    """Build a log-odds occupancy grid from optimized poses and scans."""

    def __init__(self, bag_path: Path, optimized_graph_path: Path, config: MappingConfig) -> None:
        self.bag_path = bag_path
        self.optimized_graph_path = optimized_graph_path
        self.config = config
        graph = json.loads(optimized_graph_path.read_text(encoding='utf-8'))
        keyframes = graph.get('optimized_keyframes') or graph.get('keyframes') or []
        self.trajectory = OptimizedTrajectory(keyframes)
        self.landmarks = graph.get('optimized_landmarks') or graph.get('landmarks') or []
        self.origin_x, self.origin_y, self.width, self.height = self._grid_geometry()
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.touched = np.zeros((self.height, self.width), dtype=bool)
        self.hit_count = np.zeros((self.height, self.width), dtype=np.int32)
        self._last_integrated: tuple[float, float, float] | None = None
        self.laser_x_m = config.laser_x_m
        self.laser_y_m = config.laser_y_m
        self.laser_yaw_rad = config.laser_yaw_rad
        self.laser_transform_source = 'parameters'
        self.stats = {
            'scan_messages': 0,
            'processed_scans': 0,
            'valid_rays': 0,
            'free_updates': 0,
            'occupied_updates': 0,
        }

    def build(self) -> dict[str, Any]:
        db3_files = sorted(self.bag_path.glob('*.db3')) if self.bag_path.is_dir() else [self.bag_path]
        if not db3_files:
            raise FileNotFoundError(f'No .db3 files found in {self.bag_path}')
        for db3_file in db3_files:
            self._process_db3(db3_file)
        return self.summary()

    def summary(self) -> dict[str, Any]:
        return {
            'bag': str(self.bag_path),
            'optimized_graph': str(self.optimized_graph_path),
            'resolution': self.config.resolution,
            'origin': {'x': self.origin_x, 'y': self.origin_y, 'theta': 0.0},
            'width': self.width,
            'height': self.height,
            'stats': self.stats,
            'trajectory_keyframes': len(self.trajectory.poses),
            'landmarks': len(self.landmarks),
            'laser_transform': {
                'source': self.laser_transform_source,
                'base_frame': self.config.base_frame,
                'x': self.laser_x_m,
                'y': self.laser_y_m,
                'yaw': self.laser_yaw_rad,
            },
        }

    def export(self, output_dir: Path, map_name: str) -> dict[str, str]:
        output_dir.mkdir(parents=True, exist_ok=True)
        occupancy = self._occupancy_values()
        pgm_image = self._pgm_image(occupancy)
        pgm_path = output_dir / f'{map_name}.pgm'
        png_path = output_dir / f'{map_name}.png'
        yaml_path = output_dir / f'{map_name}.yaml'
        summary_path = output_dir / f'{map_name}_summary.json'
        trajectory_path = output_dir / 'optimized_trajectory.csv'
        landmarks_path = output_dir / 'optimized_landmarks.json'

        cv2.imwrite(str(pgm_path), pgm_image)
        cv2.imwrite(str(png_path), self._debug_png(occupancy))
        yaml_path.write_text(
            '\n'.join([
                f'image: {pgm_path.name}',
                f'resolution: {self.config.resolution}',
                f'origin: [{self.origin_x}, {self.origin_y}, 0.0]',
                'negate: 0',
                'occupied_thresh: 0.65',
                'free_thresh: 0.19',
                '',
            ]),
            encoding='utf-8',
        )
        summary_path.write_text(json.dumps(self.summary(), indent=2, sort_keys=True), encoding='utf-8')
        self._write_trajectory_csv(trajectory_path)
        landmarks_path.write_text(json.dumps(self.landmarks, indent=2, sort_keys=True), encoding='utf-8')
        return {
            'pgm': str(pgm_path),
            'png': str(png_path),
            'yaml': str(yaml_path),
            'summary': str(summary_path),
            'trajectory_csv': str(trajectory_path),
            'landmarks_json': str(landmarks_path),
        }

    def _grid_geometry(self) -> tuple[float, float, int, int]:
        min_x, max_x, min_y, max_y = self.trajectory.bounds()
        pad = self.config.max_range_m + self.config.margin_m
        min_x -= pad
        max_x += pad
        min_y -= pad
        max_y += pad
        width = int(math.ceil((max_x - min_x) / self.config.resolution)) + 1
        height = int(math.ceil((max_y - min_y) / self.config.resolution)) + 1
        return min_x, min_y, width, height

    def _process_db3(self, db3_file: Path) -> None:
        connection = sqlite3.connect(str(db3_file))
        try:
            topic = connection.execute('SELECT id, type FROM topics WHERE name = ?', (self.config.scan_topic,)).fetchone()
            if topic is None:
                raise KeyError(f'Scan topic {self.config.scan_topic!r} not found in {db3_file}')
            topic_id, topic_type = int(topic[0]), str(topic[1])
            scan_type = get_message(topic_type)
            self._load_laser_transform_if_available(connection, scan_type, topic_id)
            self.stats['scan_messages'] += int(connection.execute('SELECT COUNT(*) FROM messages WHERE topic_id=?', (topic_id,)).fetchone()[0])
            query = '''
                SELECT timestamp, data
                FROM (
                    SELECT timestamp, data, ROW_NUMBER() OVER (ORDER BY timestamp, id) AS rn
                    FROM messages
                    WHERE topic_id = ?
                )
                WHERE ((rn - 1) % ?) = 0
                ORDER BY timestamp
            '''
            total = self.stats['scan_messages']
            seen = 0
            for _timestamp, data in connection.execute(query, (topic_id, self.config.scan_stride)):
                scan = deserialize_message(data, scan_type)
                self._process_scan(scan)
                seen += 1
                if seen % 500 == 0:
                    print(
                        'occupancy progress: read=%d/%d integrated=%d valid_rays=%d'
                        % (seen * self.config.scan_stride, total, self.stats['processed_scans'], self.stats['valid_rays']),
                        flush=True,
                    )
        finally:
            connection.close()

    def _process_scan(self, scan: Any) -> None:
        pose = self.trajectory.at(float(scan.header.stamp.sec) + 1e-9 * float(scan.header.stamp.nanosec))
        if self._skip_for_distance(pose):
            return
        laser_cos = math.cos(pose.theta)
        laser_sin = math.sin(pose.theta)
        laser_x = pose.x + laser_cos * self.laser_x_m - laser_sin * self.laser_y_m
        laser_y = pose.y + laser_sin * self.laser_x_m + laser_cos * self.laser_y_m
        start_cell = self._world_to_grid(laser_x, laser_y)
        if start_cell is None:
            return
        self._last_integrated = (pose.x, pose.y, pose.theta)
        self.stats['processed_scans'] += 1

        ranges = np.asarray(scan.ranges, dtype=np.float64)
        index = np.arange(0, ranges.shape[0], self.config.beam_stride)
        selected = ranges[index]
        finite = np.isfinite(selected) & (selected >= max(float(scan.range_min), self.config.min_range_m))
        index = index[finite]
        selected = selected[finite]
        if index.size == 0:
            return
        hit_is_valid = selected <= min(float(scan.range_max), self.config.max_range_m)
        ray_range = np.minimum(selected, self.config.max_range_m)
        angle = pose.theta + self.laser_yaw_rad + float(scan.angle_min) + index * float(scan.angle_increment)
        end_x = laser_x + ray_range * np.cos(angle)
        end_y = laser_y + ray_range * np.sin(angle)

        keep = self._outlier_mask(end_x[hit_is_valid], end_y[hit_is_valid])
        valid_keep = np.ones(index.size, dtype=bool)
        valid_keep[hit_is_valid] = keep

        for k in range(index.size):
            if not valid_keep[k]:
                continue
            end_cell = self._world_to_grid(end_x[k], end_y[k])
            if end_cell is None:
                continue
            self._trace_ray(start_cell, end_cell, bool(hit_is_valid[k]))

    def _skip_for_distance(self, pose: Pose2D) -> bool:
        if self._last_integrated is None:
            return False
        last_x, last_y, last_theta = self._last_integrated
        moved = math.hypot(pose.x - last_x, pose.y - last_y)
        turned = abs(normalize_angle(pose.theta - last_theta))
        return moved < self.config.min_scan_travel_m and turned < self.config.min_scan_travel_rad

    def _outlier_mask(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        """Statistical outlier removal: keep endpoints with typical neighbor spacing."""

        n = xs.shape[0]
        if not self.config.outlier_filter or cKDTree is None or n <= self.config.outlier_neighbors + 1:
            return np.ones(n, dtype=bool)
        points = np.column_stack([xs, ys])
        tree = cKDTree(points)
        k = self.config.outlier_neighbors + 1
        distances, _ = tree.query(points, k=k)
        mean_neighbor = distances[:, 1:].mean(axis=1)
        threshold = mean_neighbor.mean() + self.config.outlier_std_mul * mean_neighbor.std()
        return mean_neighbor <= threshold

    def _world_to_grid(self, x: float, y: float) -> tuple[int, int] | None:
        col = int(math.floor((x - self.origin_x) / self.config.resolution))
        row = int(math.floor((y - self.origin_y) / self.config.resolution))
        if col < 0 or row < 0 or col >= self.width or row >= self.height:
            return None
        return row, col

    def _trace_ray(self, start: tuple[int, int], end: tuple[int, int], hit_is_valid: bool) -> None:
        cells = bresenham(start[1], start[0], end[1], end[0])
        if not cells:
            return
        free_cells = cells[:-1] if hit_is_valid else cells
        for col, row in free_cells:
            self._update(row, col, self.config.log_odds_free)
            self.stats['free_updates'] += 1
        if hit_is_valid:
            col, row = cells[-1]
            self._update(row, col, self.config.log_odds_occupied)
            if 0 <= row < self.height and 0 <= col < self.width:
                self.hit_count[row, col] += 1
            self.stats['occupied_updates'] += 1
        self.stats['valid_rays'] += 1

    def _update(self, row: int, col: int, delta: float) -> None:
        if row < 0 or col < 0 or row >= self.height or col >= self.width:
            return
        self.log_odds[row, col] = np.clip(
            self.log_odds[row, col] + delta,
            self.config.log_odds_min,
            self.config.log_odds_max,
        )
        self.touched[row, col] = True

    def _occupancy_values(self) -> np.ndarray:
        probability = 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))
        occupancy = np.full((self.height, self.width), -1, dtype=np.int16)
        occupancy[self.touched & (probability <= self.config.free_threshold)] = 0
        # A wall needs both high occupancy probability and enough independent hits.
        occupied = (
            self.touched
            & (probability >= self.config.occupied_threshold)
            & (self.hit_count >= self.config.min_hits_for_occupied)
        )
        occupancy[occupied] = 100
        # Cells that look occupied but lack hit support fall back to free space.
        under_supported = (
            self.touched
            & (probability >= self.config.occupied_threshold)
            & (self.hit_count < self.config.min_hits_for_occupied)
        )
        occupancy[under_supported] = 0
        # Intermediate-probability cells are left as unknown (-1) instead of a gray
        # gradient: a crisp three-level map (free / occupied / unknown) reads much
        # cleaner in RViz and is what Nav2 expects.
        occupancy = self._clean_and_inflate(occupancy)
        return occupancy

    def _clean_and_inflate(self, occupancy: np.ndarray) -> np.ndarray:
        cleaned = occupancy.copy()
        occupied = (cleaned >= 100).astype(np.uint8)

        if self.config.min_occupied_component_cells > 1 and occupied.any():
            component_count, labels, stats, _centroids = cv2.connectedComponentsWithStats(occupied, connectivity=8)
            filtered = np.zeros_like(occupied)
            for label in range(1, component_count):
                area = int(stats[label, cv2.CC_STAT_AREA])
                if area >= self.config.min_occupied_component_cells:
                    filtered[labels == label] = 1
            removed = (occupied == 1) & (filtered == 0)
            cleaned[removed & self.touched] = 0
            occupied = filtered

        inflate_cells = int(math.ceil(self.config.inflate_radius_m / self.config.resolution))
        if inflate_cells > 0 and occupied.any():
            kernel_size = 2 * inflate_cells + 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
            inflated = cv2.dilate(occupied, kernel, iterations=1).astype(bool)
            cleaned[inflated] = 100
        return cleaned

    def _load_laser_transform_if_available(self, connection: sqlite3.Connection, scan_type: Any, scan_topic_id: int) -> None:
        if not self.config.use_tf_static or self.laser_transform_source == 'tf_static':
            return
        first_scan = connection.execute(
            'SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp, id LIMIT 1',
            (scan_topic_id,),
        ).fetchone()
        if first_scan is None:
            return
        scan_msg = deserialize_message(first_scan[0], scan_type)
        scan_frame = normalize_frame(scan_msg.header.frame_id)
        base_frame = normalize_frame(self.config.base_frame)

        tf_topic = connection.execute(
            'SELECT id, type FROM topics WHERE name = ?',
            (self.config.tf_static_topic,),
        ).fetchone()
        if tf_topic is None:
            return
        tf_topic_id, tf_type = int(tf_topic[0]), str(tf_topic[1])
        tf_msg_type = get_message(tf_type)
        transforms: dict[tuple[str, str], tuple[float, float, float]] = {}
        for (data,) in connection.execute(
            'SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp, id LIMIT 50',
            (tf_topic_id,),
        ):
            tf_msg = deserialize_message(data, tf_msg_type)
            for transform in tf_msg.transforms:
                parent = normalize_frame(transform.header.frame_id)
                child = normalize_frame(transform.child_frame_id)
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                yaw = quaternion_to_yaw(float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w))
                transforms[(parent, child)] = (float(translation.x), float(translation.y), yaw)

        resolved = resolve_transform_2d(transforms, base_frame, scan_frame)
        if resolved is None:
            return
        self.laser_x_m, self.laser_y_m, self.laser_yaw_rad = resolved
        self.laser_transform_source = f'tf_static:{base_frame}->{scan_frame}'

    def _pgm_image(self, occupancy: np.ndarray) -> np.ndarray:
        image = np.full((self.height, self.width), 205, dtype=np.uint8)
        image[occupancy == 0] = 254
        image[occupancy >= 100] = 0
        mid = (occupancy > 0) & (occupancy < 100)
        image[mid] = (254 - occupancy[mid] * 254 / 100).astype(np.uint8)
        return np.flipud(image)

    def _debug_png(self, occupancy: np.ndarray) -> np.ndarray:
        gray = self._pgm_image(occupancy)
        color = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        for pose in self.trajectory.poses[:: max(1, len(self.trajectory.poses) // 500)]:
            cell = self._world_to_grid(pose.x, pose.y)
            if cell is None:
                continue
            row, col = cell
            draw_row = self.height - 1 - row
            cv2.circle(color, (col, draw_row), 1, (0, 0, 255), -1)
        for landmark in self.landmarks:
            cell = self._world_to_grid(float(landmark['x']), float(landmark['y']))
            if cell is None:
                continue
            row, col = cell
            draw_row = self.height - 1 - row
            cv2.circle(color, (col, draw_row), 3, (0, 180, 255), -1)
        return color

    def _write_trajectory_csv(self, path: Path) -> None:
        with path.open('w', encoding='utf-8') as stream:
            stream.write('stamp,x,y,theta\n')
            for pose in self.trajectory.poses:
                stream.write(f'{pose.stamp},{pose.x},{pose.y},{pose.theta}\n')


def bresenham(x0: int, y0: int, x1: int, y1: int) -> list[tuple[int, int]]:
    """Integer Bresenham line from (x0, y0) to (x1, y1), inclusive."""

    cells: list[tuple[int, int]] = []
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    x, y = x0, y0
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * error
        if e2 >= dy:
            error += dy
            x += sx
        if e2 <= dx:
            error += dx
            y += sy
    return cells


def normalize_frame(frame: str) -> str:
    """Normalize TF frame names for bag lookups."""

    return frame.strip().lstrip('/')


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from a quaternion."""

    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def compose_transform_2d(
    first: tuple[float, float, float],
    second: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Compose 2D transforms T_ac = T_ab * T_bc."""

    x1, y1, yaw1 = first
    x2, y2, yaw2 = second
    c1 = math.cos(yaw1)
    s1 = math.sin(yaw1)
    return (
        x1 + c1 * x2 - s1 * y2,
        y1 + s1 * x2 + c1 * y2,
        normalize_angle(yaw1 + yaw2),
    )


def invert_transform_2d(transform: tuple[float, float, float]) -> tuple[float, float, float]:
    """Invert a 2D transform."""

    x, y, yaw = transform
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (-c * x - s * y, s * x - c * y, normalize_angle(-yaw))


def resolve_transform_2d(
    transforms: dict[tuple[str, str], tuple[float, float, float]],
    source: str,
    target: str,
) -> tuple[float, float, float] | None:
    """Resolve a source->target transform through a static TF tree."""

    if source == target:
        return (0.0, 0.0, 0.0)

    adjacency: dict[str, list[tuple[str, tuple[float, float, float]]]] = {}
    for (parent, child), transform in transforms.items():
        adjacency.setdefault(parent, []).append((child, transform))
        adjacency.setdefault(child, []).append((parent, invert_transform_2d(transform)))

    queue: list[tuple[str, tuple[float, float, float]]] = [(source, (0.0, 0.0, 0.0))]
    visited = {source}
    while queue:
        frame, accumulated = queue.pop(0)
        for next_frame, edge_transform in adjacency.get(frame, []):
            if next_frame in visited:
                continue
            next_transform = compose_transform_2d(accumulated, edge_transform)
            if next_frame == target:
                return next_transform
            visited.add(next_frame)
            queue.append((next_frame, next_transform))
    return None


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Build an occupancy grid from optimized SLAM graph and rosbag2 LaserScan.')
    parser.add_argument('--bag', required=True)
    parser.add_argument('--optimized-graph', required=True)
    parser.add_argument('--output-dir', default='log/maps')
    parser.add_argument('--map-name', default='laberinto_map')
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--max-range-m', type=float, default=6.0)
    parser.add_argument('--scan-stride', type=int, default=1)
    parser.add_argument('--beam-stride', type=int, default=2)
    parser.add_argument('--margin-m', type=float, default=1.0)
    parser.add_argument('--inflate-radius-m', type=float, default=0.15)
    parser.add_argument('--min-occupied-component-cells', type=int, default=3)
    parser.add_argument('--min-scan-travel-m', type=float, default=0.05)
    parser.add_argument('--min-scan-travel-rad', type=float, default=0.05)
    parser.add_argument('--min-hits-for-occupied', type=int, default=2)
    parser.add_argument('--no-outlier-filter', action='store_true', help='Disable per-scan statistical outlier removal.')
    parser.add_argument('--no-tf-static', action='store_true', help='Disable static TF lookup and use laser_* parameters.')
    parser.add_argument('--laser-x-m', type=float, default=-0.04, help='Laser x offset in base frame (used when TF static is off).')
    parser.add_argument('--laser-y-m', type=float, default=0.0)
    parser.add_argument('--laser-yaw-rad', type=float, default=1.5707963267948966)
    parser.add_argument('--scan-topic', type=str, default='/tb4_0/scan', help='LaserScan topic name in the bag.')
    parser.add_argument('--tf-static-topic', type=str, default='/tb4_0/tf_static', help='tf_static topic name in the bag.')
    parser.add_argument('--base-frame', type=str, default='base_link', help='Robot base frame to resolve the laser transform from.')
    return parser


def config_from_args(args: argparse.Namespace) -> MappingConfig:
    return MappingConfig(
        resolution=args.resolution,
        max_range_m=args.max_range_m,
        scan_stride=max(1, int(args.scan_stride)),
        beam_stride=max(1, int(args.beam_stride)),
        margin_m=args.margin_m,
        inflate_radius_m=max(0.0, args.inflate_radius_m),
        min_occupied_component_cells=max(0, int(args.min_occupied_component_cells)),
        min_scan_travel_m=max(0.0, args.min_scan_travel_m),
        min_scan_travel_rad=max(0.0, args.min_scan_travel_rad),
        min_hits_for_occupied=max(1, int(args.min_hits_for_occupied)),
        outlier_filter=not args.no_outlier_filter,
        use_tf_static=not args.no_tf_static,
        laser_x_m=args.laser_x_m,
        laser_y_m=args.laser_y_m,
        laser_yaw_rad=args.laser_yaw_rad,
        scan_topic=args.scan_topic,
        tf_static_topic=args.tf_static_topic,
        base_frame=args.base_frame,
    )


def main(argv: list[str] | None = None) -> None:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    builder = OccupancyGridBuilder(Path(args.bag), Path(args.optimized_graph), config_from_args(args))
    summary = builder.build()
    outputs = builder.export(Path(args.output_dir), args.map_name)
    print(json.dumps({'summary': summary, 'outputs': outputs}, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
