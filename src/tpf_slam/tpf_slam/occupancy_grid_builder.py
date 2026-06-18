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

from .graph_slam_frontend_node import normalize_angle


@dataclass(frozen=True)
class MappingConfig:
    scan_topic: str = '/tb4_0/scan'
    resolution: float = 0.05
    margin_m: float = 1.0
    max_range_m: float = 6.0
    min_range_m: float = 0.15
    scan_stride: int = 1
    beam_stride: int = 2
    log_odds_free: float = -0.35
    log_odds_occupied: float = 0.85
    log_odds_min: float = -4.0
    log_odds_max: float = 4.0
    occupied_threshold: float = 0.65
    free_threshold: float = 0.35
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
                'free_thresh: 0.196',
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
            for _timestamp, data in connection.execute(query, (topic_id, self.config.scan_stride)):
                scan = deserialize_message(data, scan_type)
                self._process_scan(scan)
        finally:
            connection.close()

    def _process_scan(self, scan: Any) -> None:
        pose = self.trajectory.at(float(scan.header.stamp.sec) + 1e-9 * float(scan.header.stamp.nanosec))
        laser_cos = math.cos(pose.theta)
        laser_sin = math.sin(pose.theta)
        laser_x = pose.x + laser_cos * self.config.laser_x_m - laser_sin * self.config.laser_y_m
        laser_y = pose.y + laser_sin * self.config.laser_x_m + laser_cos * self.config.laser_y_m
        start_cell = self._world_to_grid(laser_x, laser_y)
        if start_cell is None:
            return
        self.stats['processed_scans'] += 1

        for index in range(0, len(scan.ranges), self.config.beam_stride):
            raw_range = float(scan.ranges[index])
            if not math.isfinite(raw_range):
                continue
            if raw_range < max(float(scan.range_min), self.config.min_range_m):
                continue
            hit_is_valid = raw_range <= min(float(scan.range_max), self.config.max_range_m)
            ray_range = min(raw_range, self.config.max_range_m)
            angle = pose.theta + self.config.laser_yaw_rad + float(scan.angle_min) + index * float(scan.angle_increment)
            end_x = laser_x + ray_range * math.cos(angle)
            end_y = laser_y + ray_range * math.sin(angle)
            end_cell = self._world_to_grid(end_x, end_y)
            if end_cell is None:
                continue
            self._trace_ray(start_cell, end_cell, hit_is_valid)

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
        occupancy[self.touched & (probability >= self.config.occupied_threshold)] = 100
        known_mid = self.touched & (occupancy < 0)
        occupancy[known_mid] = np.clip((probability[known_mid] * 100.0).astype(np.int16), 1, 99)
        return occupancy

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
    return parser


def config_from_args(args: argparse.Namespace) -> MappingConfig:
    return MappingConfig(
        resolution=args.resolution,
        max_range_m=args.max_range_m,
        scan_stride=max(1, int(args.scan_stride)),
        beam_stride=max(1, int(args.beam_stride)),
        margin_m=args.margin_m,
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