"""A* path planner over an inflated occupancy costmap.

Subscribes to /map for the occupancy grid, inflates obstacles by a configurable
radius, then plans 8-directional A* paths from the current pose estimate to any
goal published on /goal_pose.  Replanning triggers immediately on new goals.

Dynamic obstacle layer: laser scan hits are projected to map frame and merged
with the static map before inflation so uncharted obstacles (e.g. table legs)
are avoided.  The layer is refreshed every scan; replanning is throttled to
dyn_replan_hz to avoid replanning at full scan rate.
"""

from __future__ import annotations

import heapq
import json
import math
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                        ReliabilityPolicy)
from scipy.ndimage import distance_transform_edt
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker


_SQRT2 = math.sqrt(2.0)
_MAX_TERRAIN_COST = 5.0  # max A* step penalty at lethal boundary

_MAP_QOS = QoSProfile(
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
_VIS_QOS = QoSProfile(
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
_DIRS = [(1, 0), (-1, 0), (0, 1), (0, -1),
         (1, 1), (1, -1), (-1, 1), (-1, -1)]
_COSTS = [1.0, 1.0, 1.0, 1.0, _SQRT2, _SQRT2, _SQRT2, _SQRT2]
# Heading angle for each direction in _DIRS (radians)
_DIR_ANGLES = [0.0, math.pi, math.pi / 2, -math.pi / 2,
               math.pi / 4, -math.pi / 4, 3 * math.pi / 4, -3 * math.pi / 4]


def _angle_diff(a: float, b: float) -> float:
    """Shortest angular distance between two angles, in [0, π]."""
    d = abs(a - b) % (2 * math.pi)
    return min(d, 2 * math.pi - d)


def _astar(costmap: np.ndarray,
           start: tuple[int, int],
           goal: tuple[int, int],
           start_heading: float = 0.0,
           turn_weight: float = 1.0) -> list[tuple[int, int]] | None:
    """Return list of (col, row) from start to goal, or None if unreachable.

    State: (col, row, dir_idx) — heading-aware so A* penalises sharp turns.
    This models the non-holonomic constraint: paths with gentle curves are
    preferred over geometrically shorter paths that require U-turns.

    costmap: float array — inf=lethal, 0=free, gradient in between.
    turn_weight: cost per radian of heading change (1.0 ≈ 1 cell per 57°).
    """
    h, w = costmap.shape
    start_dir = min(range(8), key=lambda i: _angle_diff(_DIR_ANGLES[i], start_heading))
    start_state = (start[0], start[1], start_dir)

    g_score: dict[tuple[int, int, int], float] = {start_state: 0.0}
    came_from: dict[tuple[int, int, int], tuple[int, int, int] | None] = {start_state: None}
    closed: set[tuple[int, int, int]] = set()
    heap: list[tuple[float, float, tuple[int, int, int]]] = []

    def heur(c: int, r: int) -> float:
        return math.hypot(c - goal[0], r - goal[1])

    heapq.heappush(heap, (heur(*start), 0.0, start_state))

    while heap:
        _, g, cur = heapq.heappop(heap)
        if cur in closed:
            continue
        closed.add(cur)
        c, r, d = cur
        if (c, r) == goal:
            path: list[tuple[int, int]] = []
            node: tuple[int, int, int] | None = cur
            while node is not None:
                path.append((node[0], node[1]))
                node = came_from.get(node)
            return list(reversed(path))
        for nd, ((dc, dr), step) in enumerate(zip(_DIRS, _COSTS)):
            nc, nr = c + dc, r + dr
            if not (0 <= nc < w and 0 <= nr < h):
                continue
            cell_cost = costmap[nr, nc]
            if not math.isfinite(cell_cost):
                continue
            turn = _angle_diff(_DIR_ANGLES[d], _DIR_ANGLES[nd])
            ng = g + step + cell_cost + turn_weight * turn
            nstate = (nc, nr, nd)
            if ng < g_score.get(nstate, float('inf')):
                g_score[nstate] = ng
                came_from[nstate] = cur
                heapq.heappush(heap, (ng + heur(nc, nr), ng, nstate))
    return None


def _snap_to_free(costmap: np.ndarray,
                  col: int, row: int,
                  max_search: int = 30) -> tuple[int, int] | None:
    """Return nearest traversable cell to (col, row) via BFS, or None if not found."""
    h, w = costmap.shape
    if 0 <= row < h and 0 <= col < w and math.isfinite(costmap[row, col]):
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
                if math.isfinite(costmap[nr, nc]):
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

        self.declare_parameter('inflation_radius_m', 0.28)
        self.declare_parameter('robot_radius_m', 0.13)
        self.declare_parameter('dyn_inflation_radius_m', 0.10)
        self.declare_parameter('turn_weight', 1.0)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('costmap_topic', '/costmap')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('pose_topic', '/pose_estimate')
        self.declare_parameter('nav_status_topic', '/nav_status')
        self.declare_parameter('smooth_window', 5)
        # dynamic obstacle layer
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('laser_x_m', -0.032)
        self.declare_parameter('laser_yaw_rad', 0.0)
        self.declare_parameter('dyn_replan_hz', 1.0)

        self._inflate_r = float(self.get_parameter('inflation_radius_m').value)
        self._robot_r = float(self.get_parameter('robot_radius_m').value)
        self._dyn_inflate_r = float(self.get_parameter('dyn_inflation_radius_m').value)
        self._turn_weight = float(self.get_parameter('turn_weight').value)
        self._smooth_w = int(self.get_parameter('smooth_window').value)
        self._laser_x_m = float(self.get_parameter('laser_x_m').value)
        self._laser_yaw_rad = float(self.get_parameter('laser_yaw_rad').value)

        self._map_info: Any = None
        self._blocked: np.ndarray | None = None      # static obstacle mask
        self._dynamic_blocked: np.ndarray | None = None  # laser-hit mask (confirmed)
        self._dyn_scan_history: list[np.ndarray] = []    # last 3 raw scan masks
        self._costmap: np.ndarray | None = None      # inflated combined mask
        self._last_path_cells: list[tuple[int, int]] = []

        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._robot_yaw: float = 0.0

        self._goal: PoseStamped | None = None
        self._dyn_changed: bool = False

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
        self._inflation_marker_pub = self.create_publisher(
            Marker, '/inflation_gradient', _VIS_QOS)
        self._map_obstacles_pub = self.create_publisher(
            Marker, '/map_obstacles', _VIS_QOS)

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
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self._on_scan, 10)

        dyn_period = 1.0 / max(0.1, float(self.get_parameter('dyn_replan_hz').value))
        self.create_timer(dyn_period, self._dyn_replan_cb)
        self.create_timer(1.0, self._publish_inflation_marker)

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
        self._publish_map_obstacles_marker()

    def _publish_map_obstacles_marker(self) -> None:
        """Publish the raw occupied map cells as a robust RViz marker.

        RViz's OccupancyGrid display can render poorly on some WSL/OpenGL
        combinations.  This marker is intentionally just the real occupied
        cells from /map, with no inflation, so the presentation can clearly
        separate physical walls from the planner's safety margin.
        """
        if self._blocked is None or self._map_info is None:
            return

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'raw_map_obstacles'
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        res = self._map_info.resolution
        ox = self._map_info.origin.position.x
        oy = self._map_info.origin.position.y
        marker.scale.x = res
        marker.scale.y = res
        marker.scale.z = 0.02
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        rows, cols = np.where(self._blocked)
        for row, col in zip(rows, cols):
            point = Point()
            point.x = float(ox + (col + 0.5) * res)
            point.y = float(oy + (row + 0.5) * res)
            point.z = 0.0
            marker.points.append(point)

        self._map_obstacles_pub.publish(marker)

    def _build_costmap(self) -> None:
        if self._blocked is None or self._map_info is None:
            return
        res = self._map_info.resolution

        # Distance from nearest static obstacle in metres
        dist = distance_transform_edt(~self._blocked) * res

        costmap = np.zeros(self._blocked.shape, dtype=float)

        # Lethal zone: within robot_radius → inf
        costmap[dist < self._robot_r] = math.inf

        # Gradient zone: exponential decay from robot_r to inflate_r
        span = self._inflate_r - self._robot_r
        if span > 0:
            grad_mask = (dist >= self._robot_r) & (dist < self._inflate_r)
            d = dist[grad_mask]
            costmap[grad_mask] = _MAX_TERRAIN_COST * np.exp(-3.0 * (d - self._robot_r) / span)

        # Dynamic obstacles: lethal within dyn_inflate_r
        if self._dynamic_blocked is not None and self._dynamic_blocked.any():
            dist_dyn = distance_transform_edt(~self._dynamic_blocked) * res
            costmap[dist_dyn < self._dyn_inflate_r] = math.inf

        self._costmap = costmap

    def _publish_costmap(self) -> None:
        if self._costmap is None or self._map_info is None:
            return
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info = self._map_info
        # inf → 100, gradient → 1-99 proportional, free → 0
        vis = np.where(
            np.isinf(self._costmap), 100,
            np.clip(self._costmap / _MAX_TERRAIN_COST * 99, 0, 99)
        ).astype(np.int8)
        msg.data = vis.flatten().tolist()
        self._costmap_pub.publish(msg)
        self._publish_inflation_marker()

    def _publish_inflation_marker(self) -> None:
        """Publish only the soft inflation gradient as a colored overlay.

        ``/costmap`` is still the planner's truth and includes lethal inflated
        cells.  This marker intentionally shows just finite cost cells, so RViz
        can display the real occupancy map separately and avoid making walls
        look physically thicker than they are.
        """
        if self._costmap is None or self._map_info is None:
            return

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'inflation_gradient'
        marker.id = 0
        marker.pose.orientation.w = 1.0

        grad_mask = np.isfinite(self._costmap) & (self._costmap > 1.0e-6)
        if not grad_mask.any():
            marker.action = Marker.DELETE
            self._inflation_marker_pub.publish(marker)
            return

        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        res = self._map_info.resolution
        ox = self._map_info.origin.position.x
        oy = self._map_info.origin.position.y
        marker.scale.x = res
        marker.scale.y = res
        marker.scale.z = 0.01

        rows, cols = np.where(grad_mask)
        costs = self._costmap[rows, cols]
        norm = np.clip(costs / _MAX_TERRAIN_COST, 0.0, 1.0)

        for row, col, value in zip(rows, cols, norm):
            point = Point()
            point.x = float(ox + (col + 0.5) * res)
            point.y = float(oy + (row + 0.5) * res)
            point.z = 0.01
            marker.points.append(point)

            color = ColorRGBA()
            color.r = 1.0
            color.g = float(0.85 - 0.55 * value)  # yellow -> orange
            color.b = 0.0
            color.a = float(0.18 + 0.42 * value)
            marker.colors.append(color)

        self._inflation_marker_pub.publish(marker)

    # ------------------------------------------------------------------
    # Dynamic obstacle layer
    # ------------------------------------------------------------------

    def _on_scan(self, msg: LaserScan) -> None:
        if self._map_info is None or self._blocked is None:
            return
        h, w = self._blocked.shape
        new_dyn = np.zeros((h, w), dtype=bool)

        cos_r = math.cos(self._robot_yaw)
        sin_r = math.sin(self._robot_yaw)
        # laser origin in world frame
        lx = self._robot_x + self._laser_x_m * cos_r
        ly = self._robot_y + self._laser_x_m * sin_r

        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                world_angle = angle + self._robot_yaw + self._laser_yaw_rad
                hx = lx + r * math.cos(world_angle)
                hy = ly + r * math.sin(world_angle)
                col, row = self._world_to_cell(hx, hy)
                if 0 <= col < w and 0 <= row < h:
                    new_dyn[row, col] = True
            angle += msg.angle_increment

        # Require hit in 2 consecutive scans to filter single-frame pose glitches
        # Majority vote over last 3 scans: cell confirmed if hit in ≥2 of 3.
        # Filters single-frame and double-frame MCL pose-jump artifacts.
        self._dyn_scan_history.append(new_dyn)
        if len(self._dyn_scan_history) > 3:
            self._dyn_scan_history.pop(0)

        if len(self._dyn_scan_history) >= 2:
            a, b = self._dyn_scan_history[-1], self._dyn_scan_history[-2]
            if len(self._dyn_scan_history) >= 3:
                c = self._dyn_scan_history[-3]
                confirmed = (a & b) | (b & c) | (a & c)
            else:
                confirmed = a & b
        else:
            confirmed = np.zeros_like(new_dyn)

        if self._dynamic_blocked is None or not np.array_equal(confirmed, self._dynamic_blocked):
            self._dynamic_blocked = confirmed
            self._dyn_changed = True

    def _path_still_valid(self) -> bool:
        """Return True if every cell in the last known path is traversable."""
        if self._costmap is None or not self._last_path_cells:
            return False
        return all(math.isfinite(self._costmap[r, c]) for c, r in self._last_path_cells)

    def _dyn_replan_cb(self) -> None:
        if not self._dyn_changed or self._goal is None or self._blocked is None:
            return
        self._build_costmap()
        self._publish_costmap()
        self._dyn_changed = False
        if not self._path_still_valid():
            self._do_plan()

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

        path_cells = _astar(self._costmap, start_free, goal_free,
                            start_heading=self._robot_yaw,
                            turn_weight=self._turn_weight)

        if path_cells is None:
            self._publish_status('PLANNING_FAILED', 'no path found')
            self.get_logger().warn('A*: no path found')
            return

        self._last_path_cells = path_cells

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
