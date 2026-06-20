"""2D LiDAR scan matching (point-to-point ICP) for Graph SLAM constraints.

The pose-landmark graph from ArUco alone is sparse and noisy, so the trajectory
is mostly carried by wheel/IMU odometry. ICP between LaserScans adds reliable
geometric pose-pose constraints: consecutive-scan refinement sharpens local
alignment, and revisit (loop-closure) matches pull the global map back together.

This module is intentionally dependency-light (numpy + scipy KD-tree) so it can
run in the same offline batch as the rest of Parte A.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, sin

import numpy as np

try:  # pragma: no cover - exercised inside the ROS image
    from scipy.spatial import cKDTree
except Exception:  # pragma: no cover - fallback for minimal environments
    cKDTree = None


PI = 3.141592653589793


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""

    return atan2(sin(angle), cos(angle))


def scan_to_points(
    ranges: np.ndarray,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    max_range: float,
    laser_x: float = 0.0,
    laser_y: float = 0.0,
    laser_yaw: float = 0.0,
    beam_stride: int = 1,
) -> np.ndarray:
    """Convert LaserScan ranges to (N, 2) points in the robot base frame."""

    ranges = np.asarray(ranges, dtype=np.float64)
    count = ranges.shape[0]
    index = np.arange(0, count, max(1, int(beam_stride)))
    selected = ranges[index]
    angles = angle_min + index * angle_increment
    upper = min(float(range_max), float(max_range))
    valid = np.isfinite(selected) & (selected >= float(range_min)) & (selected <= upper)
    selected = selected[valid]
    angles = angles[valid]
    if selected.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    laser_px = selected * np.cos(angles)
    laser_py = selected * np.sin(angles)
    cos_yaw = cos(laser_yaw)
    sin_yaw = sin(laser_yaw)
    base_x = laser_x + cos_yaw * laser_px - sin_yaw * laser_py
    base_y = laser_y + sin_yaw * laser_px + cos_yaw * laser_py
    return np.column_stack([base_x, base_y])


@dataclass(frozen=True)
class ICPResult:
    dx: float
    dy: float
    dtheta: float
    fitness: float       # inlier fraction of the source cloud
    mean_error: float    # mean inlier correspondence distance (m)
    iterations: int
    converged: bool


def _apply(points: np.ndarray, dx: float, dy: float, dtheta: float) -> np.ndarray:
    cos_t = cos(dtheta)
    sin_t = sin(dtheta)
    out = np.empty_like(points)
    out[:, 0] = cos_t * points[:, 0] - sin_t * points[:, 1] + dx
    out[:, 1] = sin_t * points[:, 0] + cos_t * points[:, 1] + dy
    return out


def icp_match(
    source: np.ndarray,
    target: np.ndarray,
    init: tuple[float, float, float] = (0.0, 0.0, 0.0),
    max_iterations: int = 40,
    tolerance: float = 1e-4,
    trim_ratio: float = 0.8,
    max_correspondence_m: float = 0.5,
    min_points: int = 25,
) -> ICPResult:
    """Estimate the rigid transform mapping ``source`` onto ``target``.

    The returned (dx, dy, dtheta) is expressed in the target frame, i.e. applying
    it to a source point yields the corresponding target-frame point. ``init`` is
    the initial guess, normally the odometry-relative motion.
    """

    dx, dy, dtheta = init
    if cKDTree is None or len(source) < min_points or len(target) < min_points:
        return ICPResult(dx, dy, dtheta, 0.0, float('inf'), 0, False)

    tree = cKDTree(target)
    previous_error = float('inf')
    fitness = 0.0
    mean_error = float('inf')
    iteration = 0
    converged = False
    for iteration in range(1, max_iterations + 1):
        moved = _apply(source, dx, dy, dtheta)
        distances, indices = tree.query(moved, k=1)
        order = np.argsort(distances)
        keep_count = max(min_points // 2, int(trim_ratio * len(order)))
        keep = order[:keep_count]
        gate = max(max_correspondence_m, float(np.median(distances[keep])) * 3.0)
        keep = keep[distances[keep] < gate]
        if len(keep) < min_points // 2:
            break
        matched_source = moved[keep]
        matched_target = target[indices[keep]]
        mean_source = matched_source.mean(axis=0)
        mean_target = matched_target.mean(axis=0)
        covariance = (matched_source - mean_source).T @ (matched_target - mean_target)
        u_mat, _s, vt_mat = np.linalg.svd(covariance)
        rotation = vt_mat.T @ u_mat.T
        if np.linalg.det(rotation) < 0.0:
            vt_mat[1] *= -1.0
            rotation = vt_mat.T @ u_mat.T
        step_theta = atan2(rotation[1, 0], rotation[0, 0])
        step_translation = mean_target - rotation @ mean_source
        cos_t = cos(step_theta)
        sin_t = sin(step_theta)
        dx, dy = (
            cos_t * dx - sin_t * dy + step_translation[0],
            sin_t * dx + cos_t * dy + step_translation[1],
        )
        dtheta = normalize_angle(dtheta + step_theta)
        mean_error = float(distances[keep].mean())
        fitness = len(keep) / len(source)
        if abs(previous_error - mean_error) < tolerance:
            converged = True
            break
        previous_error = mean_error
    return ICPResult(dx, dy, dtheta, fitness, mean_error, iteration, converged)
