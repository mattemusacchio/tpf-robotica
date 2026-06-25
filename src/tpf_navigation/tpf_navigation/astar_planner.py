"""A* path planner over an inflated occupancy costmap.

Subscribes to /map for the occupancy grid, inflates obstacles by a configurable
radius, then plans 8-directional A* paths from the current pose estimate to any
goal published on /goal_pose.  Replanning triggers immediately on new goals.
"""

from __future__ import annotations

import heapq
import json
import math
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                        ReliabilityPolicy)
from scipy.ndimage import binary_dilation, generate_binary_structure
from std_msgs.msg import String
from visualization_msgs.msg import Marker


_SQRT2 = math.sqrt(2.0)

_MAP_QOS = QoSProfile(
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
_DIRS = [(1, 0), (-1, 0), (0, 1), (0, -1),
         (1, 1), (1, -1), (-1, 1), (-1, -1)]
_COSTS = [1.0, 1.0, 1.0, 1.0, _SQRT2, _SQRT2, _SQRT2, _SQRT2]


def _astar(blocked: np.ndarray,
           start: tuple[int, int],
           goal: tuple[int, int]) -> list[tuple[int, int]] | None:
    """Return list of (col, row) from start to goal, or None if unreachable."""
    h, w = blocked.shape
    g_score: dict[tuple[int, int], float] = {start: 0.0}
    came_from: dict[tuple[int, int], tuple[int, int] | None] = {}
    heap: list[tuple[float, float, tuple[int, int]]] = []

    def heur(c: tuple[int, int]) -> float:
        return math.hypot(c[0] - goal[0], c[1] - goal[1])

    heapq.heappush(heap, (heur(start), 0.0, start))

    while heap:
        _, g, cur = heapq.heappop(heap)
        if cur in came_from:
            continue
        came_from[cur] = None if cur == start else came_from.get(cur)
        if cur == goal:
            path: list[tuple[int, int]] = []
            node: tuple[int, int] | None = goal
            while node is not None:
                path.append(node)
                node = came_from.get(node)
            return list(reversed(path))
        for (dc, dr), cost in zip(_DIRS, _COSTS):
            nc, nr = cur[0] + dc, cur[1] + dr
            if not (0 <= nc < w and 0 <= nr < h):
                continue
            if blocked[nr, nc]:
                continue
            ng = g + cost
            if ng < g_score.get((nc, nr), float('inf')):
                g_score[(nc, nr)] = ng
                came_from[(nc, nr)] = cur
                heapq.heappush(heap, (ng + heur((nc, nr)), ng, (nc, nr)))
    return None


def _snap_to_free(blocked: np.ndarray,
                  col: int, row: int,
                  max_search: int = 30) -> tuple[int, int] | None:
    """Return nearest free cell to (col, row) via BFS, or None if not found."""
    h, w = blocked.shape
    if 0 <= row < h and 0 <= col < w and not blocked[row, col]:
        return col, row
    visited: set[tuple[int, int]] = set()
    queue = [(col, row)]
    while queue:
        next_q: list[tuple[int, int]] = []
        for c, r in queue:
            for dc, dr in _DIRS:
                nc, nr = c + dc, r + dr
                if (nc, nr) in visited:
                    continue
                visited.add((nc, nr))
                if not (0 <= nc < w and 0 <= nr < h):
                    continue
                if not blocked[nr, nc]:
                    return nc, nr
                if abs(nc - col) <= max_search and abs(nr - row) <= max_search:
                    next_q.append((nc, nr))
        queue = next_q
    return None


def _smooth(xs: list[float], ys: list[float], window: int = 5
            ) -> tuple[list[float], list[float]]:
    if len(xs) <= window:
        return xs, ys
    k = np.ones(window) / window
    xs_s = np.convolve(xs, k, mode='valid').tolist()
    ys_s = np.convolve(ys, k, mode='valid').tolist()
    return [xs[0]] + xs_s + [xs[-1]], [ys[0]] + ys_s + [ys[-1]]


class AStarPlanner(Node):
    def __init__(self) -> None:
        super().__init__('astar_planner')

        self.declare_parameter('inflation_radius_m', 0.20)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('costmap_topic', '/costmap')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('pose_topic', '/pose_estimate')
        self.declare_parameter('nav_status_topic', '/nav_status')
        self.declare_parameter('smooth_window', 5)

        self._inflate_r = float(self.get_parameter('inflation_radius_m').value)
        self._smooth_w = int(self.get_parameter('smooth_window').value)

        self._map_info: Any = None
        self._blocked: np.ndarray | None = None   # raw obstacle mask
        self._costmap: np.ndarray | None = None   # inflated mask (True=blocked)

        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._robot_yaw: float = 0.0

        self._goal: PoseStamped | None = None

        self._costmap_pub = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter('costmap_topic').value), 10)
        self._plan_pub = self.create_publisher(
            Path,
            str(self.get_parameter('plan_topic').value), 10)
        self._status_pub = self.create_publisher(
            String,
            str(self.get_parameter('nav_status_topic').value), 10)
        self._goal_marker_pub = self.create_publisher(
            Marker, '/goal_marker', 10)

        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('map_topic').value),
            self._on_map, _MAP_QOS)
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter('pose_topic').value),
            self._on_pose, 10)
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('goal_topic').value),
            self._on_goal, 10)

        self.get_logger().info('A* planner ready')

    # ------------------------------------------------------------------
    # Map + costmap
    # ------------------------------------------------------------------

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map_info = msg.info
        grid = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))
        self._blocked = (grid >= 50)
        self._build_costmap()
        self._publish_costmap()

    def _build_costmap(self) -> None:
        if self._blocked is None or self._map_info is None:
            return
        radius_cells = int(math.ceil(self._inflate_r / self._map_info.resolution))
        struct = np.zeros((2 * radius_cells + 1, 2 * radius_cells + 1), dtype=bool)
        cy = cx = radius_cells
        for r in range(struct.shape[0]):
            for c in range(struct.shape[1]):
                if math.hypot(r - cy, c - cx) <= radius_cells:
                    struct[r, c] = True
        self._costmap = binary_dilation(self._blocked, structure=struct)

    def _publish_costmap(self) -> None:
        if self._costmap is None or self._map_info is None:
            return
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info = self._map_info
        flat = self._costmap.astype(np.int8) * 100
        msg.data = flat.flatten().tolist()
        self._costmap_pub.publish(msg)

    # ------------------------------------------------------------------
    # Pose + goal
    # ------------------------------------------------------------------

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._robot_yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z))

    def _on_goal(self, msg: PoseStamped) -> None:
        self._goal = msg
        self._publish_goal_marker(msg)
        self._do_plan()

    # ------------------------------------------------------------------
    # Planning
    # ------------------------------------------------------------------

    def _world_to_cell(self, wx: float, wy: float) -> tuple[int, int]:
        res = self._map_info.resolution
        ox = self._map_info.origin.position.x
        oy = self._map_info.origin.position.y
        return int((wx - ox) / res), int((wy - oy) / res)

    def _cell_to_world(self, col: int, row: int) -> tuple[float, float]:
        res = self._map_info.resolution
        ox = self._map_info.origin.position.x
        oy = self._map_info.origin.position.y
        return ox + (col + 0.5) * res, oy + (row + 0.5) * res

    def _do_plan(self) -> None:
        if self._costmap is None or self._goal is None:
            return

        start_cell = self._world_to_cell(self._robot_x, self._robot_y)
        goal_cell = self._world_to_cell(
            self._goal.pose.position.x, self._goal.pose.position.y)

        start_free = _snap_to_free(self._costmap, *start_cell)
        goal_free = _snap_to_free(self._costmap, *goal_cell)

        if start_free is None or goal_free is None:
            self._publish_status('PLANNING_FAILED', 'start or goal unreachable')
            return

        path_cells = _astar(self._costmap, start_free, goal_free)

        if path_cells is None:
            self._publish_status('PLANNING_FAILED', 'no path found')
            self.get_logger().warn('A*: no path found')
            return

        xs = [self._cell_to_world(c, r)[0] for c, r in path_cells]
        ys = [self._cell_to_world(c, r)[1] for c, r in path_cells]
        xs, ys = _smooth(xs, ys, self._smooth_w)

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        for wx, wy in zip(xs, ys):
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            path_msg.poses.append(ps)

        self._plan_pub.publish(path_msg)
        self._publish_status('PLANNING_OK', f'{len(path_msg.poses)} waypoints')
        self.get_logger().info(f'A* plan: {len(path_msg.poses)} waypoints')

    def _publish_status(self, status: str, detail: str = '') -> None:
        msg = String()
        msg.data = json.dumps({'status': status, 'detail': detail})
        self._status_pub.publish(msg)

    def _publish_goal_marker(self, goal: PoseStamped) -> None:
        m = Marker()
        m.header = goal.header
        m.ns = 'goal'
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose = goal.pose
        m.scale.x = 0.5
        m.scale.y = 0.08
        m.scale.z = 0.08
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = 1.0
        self._goal_marker_pub.publish(m)


def main() -> None:
    rclpy.init()
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
