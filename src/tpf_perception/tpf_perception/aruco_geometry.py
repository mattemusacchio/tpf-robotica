"""Geometry helpers for ArUco detections.

OpenCV reports marker poses in the camera optical frame:

- +x: image right
- +y: image down
- +z: forward from the camera

For the first SLAM front-end we mostly need a planar observation.  Until we add a
proper TF lookup for camera->base_link, we expose a documented approximation in
base-like axes:

- robot x ~= optical z, forward
- robot y ~= -optical x, left
- bearing = atan2(robot_y, robot_x)
"""

from __future__ import annotations

from math import atan2, sqrt
from typing import Iterable

import numpy as np


def rotation_matrix_to_quaternion(rotation_matrix: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to an ``(x, y, z, w)`` quaternion."""

    m = np.asarray(rotation_matrix, dtype=float)
    trace = float(np.trace(m))

    if trace > 0.0:
        scale = sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (m[2, 1] - m[1, 2]) / scale
        qy = (m[0, 2] - m[2, 0]) / scale
        qz = (m[1, 0] - m[0, 1]) / scale
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        scale = sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        qw = (m[2, 1] - m[1, 2]) / scale
        qx = 0.25 * scale
        qy = (m[0, 1] + m[1, 0]) / scale
        qz = (m[0, 2] + m[2, 0]) / scale
    elif m[1, 1] > m[2, 2]:
        scale = sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        qw = (m[0, 2] - m[2, 0]) / scale
        qx = (m[0, 1] + m[1, 0]) / scale
        qy = 0.25 * scale
        qz = (m[1, 2] + m[2, 1]) / scale
    else:
        scale = sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        qw = (m[1, 0] - m[0, 1]) / scale
        qx = (m[0, 2] + m[2, 0]) / scale
        qy = (m[1, 2] + m[2, 1]) / scale
        qz = 0.25 * scale

    norm = sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (qx / norm, qy / norm, qz / norm, qw / norm)


def optical_tvec_to_planar_observation(tvec: Iterable[float]) -> dict[str, float]:
    """Return range/bearing and approximate base-frame coordinates for a tag tvec."""

    optical_x, optical_y, optical_z = [float(value) for value in tvec]
    robot_x = optical_z
    robot_y = -optical_x
    robot_z = -optical_y
    planar_range = sqrt(robot_x * robot_x + robot_y * robot_y)
    bearing = atan2(robot_y, robot_x)

    return {
        "x": robot_x,
        "y": robot_y,
        "z": robot_z,
        "range": planar_range,
        "bearing": bearing,
    }
