"""D* Lite path planner over an inflated occupancy costmap.

Subscribes to /map for the occupancy grid, inflates obstacles by a configurable
radius, then plans 8-directional D* Lite paths from the current pose estimate to
any goal published on /goal_pose.  Replanning triggers immediately on new goals.

D* Lite (Koenig & Likhachev 2002) searches backwards from goal to start and
incrementally repairs the plan when the costmap changes, making it far more
efficient than re-running A* from scratch on every scan update.

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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                        ReliabilityPolicy)
from scipy.ndimage import distance_transform_edt
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker


_SQRT2 = math.sqrt(2.0)
_MAX_TERRAIN_COST = 5.0  # max step penalty at lethal boundary
_INF = float('inf')

_MAP_QOS = QoSProfile(
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
_DIRS = [(1, 0), (-1, 0), (0, 1), (0, -1),
         (1, 1), (1, -1), (-1, 1), (-1, -1)]
_COSTS = [1.0, 1.0, 1.0, 1.0, _SQRT2, _SQRT2, _SQRT2, _SQRT2]


def _heuristic(s: tuple[int, int], t: tuple[int, int]) -> float:
    """Octile distance heuristic — admissible for 8-connected grid."""
    dx = abs(s[0] - t[0])
    dy = abs(s[1] - t[1])
    return max(dx, dy) + (_SQRT2 - 1.0) * min(dx, dy)


class DStarLite:
    """D* Lite planner (Koenig & Likhachev 2002).

    Searches backwards (goal→start) so incremental updates when the start
    moves or the costmap changes are O(changed cells) rather than O(map).

    State space: 2-D (col, row) tuples.

    Edge cost c(s1, s2):
      - inf   if costmap[s2_row, s2_col] is not finite (lethal)
      - otherwise: step_cost (1 or √2) + terrain_cost (costmap value at s2)
    """

    def __init__(self,
                 costmap: np.ndarray,
                 start: tuple[int, int],
                 goal: tuple[int, int]) -> None:
        self._costmap = costmap.copy()
        self._h, self._w = costmap.shape
        self.start = start
        self.goal = goal
        self._km: float = 0.0

        # g[s]: best known cost from s to goal
        # rhs[s]: one-step lookahead value
        self._g: dict[tuple[int, int], float] = {}
        self._rhs: dict[tuple[int, int], float] = {}

        # Priority queue: list of [key_tuple, counter, node, valid]
        # Lazy deletion: 'valid' flag lets us discard stale entries.
        self._heap: list[list] = []
        self._counter: int = 0          # tie-breaking counter
        self._in_heap: dict[tuple[int, int], list] = {}  # node → current entry

        # Initialise: rhs[goal] = 0, everything else = inf (default via dict.get)
        self._rhs[self.goal] = 0.0
        self._push(self.goal, self._key(self.goal))

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _g_val(self, s: tuple[int, int]) -> float:
        return self._g.get(s, _INF)

    def _rhs_val(self, s: tuple[int, int]) -> float:
        return self._rhs.get(s, _INF)

    def _key(self, s: tuple[int, int]) -> tuple[float, float]:
        min_gr = min(self._g_val(s), self._rhs_val(s))
        return (min_gr + _heuristic(self.start, s) + self._km, min_gr)

    def _push(self, s: tuple[int, int], key: tuple[float, float]) -> None:
        """Insert or replace s in the priority queue with the given key."""
        # Invalidate any existing entry for s
        old = self._in_heap.get(s)
        if old is not None:
            old[3] = False  # mark stale

        entry: list = [key, self._counter, s, True]
        self._counter += 1
        self._in_heap[s] = entry
        heapq.heappush(self._heap, entry)

    def _remove(self, s: tuple[int, int]) -> None:
        """Logically remove s from the priority queue (lazy deletion)."""
        old = self._in_heap.pop(s, None)
        if old is not None:
            old[3] = False

    def _top_key(self) -> tuple[float, float]:
        """Return the smallest key currently in U, or (inf, inf) if empty."""
        while self._heap:
            entry = self._heap[0]
            if entry[3]:  # valid
                return entry[0]
            heapq.heappop(self._heap)
        return (_INF, _INF)

    def _pop(self) -> tuple[tuple[int, int], tuple[float, float]]:
        """Pop and return (node, key) for the minimum valid entry."""
        while self._heap:
            entry = heapq.heappop(self._heap)
            key, _, s, valid = entry
            if valid:
                self._in_heap.pop(s, None)
                return s, key
        raise IndexError('pop from empty priority queue')

    def _edge_cost(self, s1: tuple[int, int], s2: tuple[int, int],
                   step: float) -> float:
        """Cost of moving from s1 to s2 (s2 must be a valid neighbour of s1)."""
        c2, r2 = s2
        if not (0 <= c2 < self._w and 0 <= r2 < self._h):
            return _INF
        cell = self._costmap[r2, c2]
        if not math.isfinite(cell):
            return _INF
        return step + cell

    def _successors(self, s: tuple[int, int]) -> list[tuple[tuple[int, int], float]]:
        """All valid grid neighbours with associated move cost."""
        c, r = s
        result = []
        for (dc, dr), step in zip(_DIRS, _COSTS):
            nb = (c + dc, r + dr)
            cost = self._edge_cost(s, nb, step)
            if cost < _INF:
                result.append((nb, cost))
        return result

    def _predecessors(self, s: tuple[int, int]) -> list[tuple[tuple[int, int], float]]:
        """Predecessors == successors for symmetric 8-connected grid."""
        return self._successors(s)

    # ------------------------------------------------------------------
    # D* Lite core
    # ------------------------------------------------------------------

    def _update_vertex(self, s: tuple[int, int]) -> None:
        if s != self.goal:
            # rhs[s] = min over successors s' of c(s, s') + g[s']
            min_rhs = _INF
            c, r = s
            for (dc, dr), step in zip(_DIRS, _COSTS):
                nb = (c + dc, r + dr)
                cost = self._edge_cost(s, nb, step)
                if cost < _INF:
                    candidate = cost + self._g_val(nb)
                    if candidate < min_rhs:
                        min_rhs = candidate
            if min_rhs == _INF:
                self._rhs.pop(s, None)
            else:
                self._rhs[s] = min_rhs

        self._remove(s)
        if self._g_val(s) != self._rhs_val(s):
            self._push(s, self._key(s))

    def compute_shortest_path(self) -> None:
        """Run D* Lite until the shortest path to start is consistent."""
        while True:
            top = self._top_key()
            k_start = self._key(self.start)
            if not (top < k_start or self._rhs_val(self.start) != self._g_val(self.start)):
                break

            u, k_old = self._pop()
            k_new = self._key(u)

            if k_old < k_new:
                # Key increased — re-insert with correct key
                self._push(u, k_new)
            elif self._g_val(u) > self._rhs_val(u):
                # Overconsistent: lower g to rhs
                self._g[u] = self._rhs_val(u)
                for s, _ in self._predecessors(u):
                    self._update_vertex(s)
            else:
                # Underconsistent: raise g to inf, re-expand
                self._g.pop(u, None)  # g[u] = inf
                self._update_vertex(u)
                for s, _ in self._predecessors(u):
                    self._update_vertex(s)

    def extract_path(self) -> list[tuple[int, int]] | None:
        """Greedily follow lowest-cost successors from start to goal.

        Returns list of (col, row) tuples, or None if goal is unreachable.
        """
        if not math.isfinite(self._g_val(self.start)):
            return None

        path = [self.start]
        visited: set[tuple[int, int]] = {self.start}
        cur = self.start
        max_steps = 10_000

        for _ in range(max_steps):
            if cur == self.goal:
                return path

            c, r = cur
            best_nb: tuple[int, int] | None = None
            best_val = _INF

            for (dc, dr), step in zip(_DIRS, _COSTS):
                nb = (c + dc, r + dr)
                if nb in visited:
                    continue
                cost = self._edge_cost(cur, nb, step)
                if cost >= _INF:
                    continue
                val = cost + self._g_val(nb)
                if val < best_val:
                    best_val = val
                    best_nb = nb

            if best_nb is None or not math.isfinite(best_val):
                return None  # stuck / unreachable

            visited.add(best_nb)
            path.append(best_nb)
            cur = best_nb

        return None  # cycle protection triggered

    def update_start(self, new_start: tuple[int, int]) -> None:
        """Move the start position, accumulating heuristic correction km."""
        self._km += _heuristic(self.start, new_start)
        self.start = new_start

    def update_costmap(self, new_costmap: np.ndarray) -> None:
        """Apply an updated costmap and repair the plan incrementally.

        Detects cells that changed lethality or changed cost significantly,
        updates affected vertices and their predecessors, then re-runs
        compute_shortest_path().
        """
        old = self._costmap
        new = new_costmap

        # Find cells whose traversability flipped OR cost changed noticeably
        old_lethal = ~np.isfinite(old)
        new_lethal = ~np.isfinite(new)
        lethality_changed = old_lethal != new_lethal

        both_finite = ~old_lethal & ~new_lethal
        cost_changed = both_finite & (np.abs(np.where(both_finite, new - old, 0.0)) > 0.1)

        changed_mask = lethality_changed | cost_changed
        changed_rows, changed_cols = np.where(changed_mask)

        self._costmap = new_costmap.copy()

        # For each changed cell, update the cell and all its predecessors
        affected: set[tuple[int, int]] = set()
        for r, c in zip(changed_rows.tolist(), changed_cols.tolist()):
            s = (int(c), int(r))
            affected.add(s)
            for (dc, dr) in _DIRS:
                pred = (c + dc, r + dr)
                pc, pr = pred
                if 0 <= pc < self._w and 0 <= pr < self._h:
                    affected.add(pred)

        for s in affected:
            self._update_vertex(s)

        self.compute_shortest_path()


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

        # D* Lite planner instance (None until first goal received)
        self._dstar: DStarLite | None = None

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
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self._on_scan, 10)

        dyn_period = 1.0 / max(0.1, float(self.get_parameter('dyn_replan_hz').value))
        self.create_timer(dyn_period, self._dyn_replan_cb)

        self.get_logger().info('D* Lite planner ready')

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
        # If a goal is already set, rebuild D* Lite from scratch with the new map
        if self._goal is not None and self._costmap is not None:
            self._do_plan()

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
        old_costmap = self._costmap
        self._build_costmap()
        self._publish_costmap()
        self._dyn_changed = False

        new_costmap = self._costmap
        if new_costmap is None:
            return

        if self._dstar is not None and old_costmap is not None:
            # Incremental update: move start, patch changed cells, replan
            start_cell = self._world_to_cell(self._robot_x, self._robot_y)
            start_free = _snap_to_free(new_costmap, *start_cell)
            if start_free is None:
                self._publish_status('PLANNING_FAILED', 'robot position unreachable')
                return

            self._dstar.update_start(start_free)
            self._dstar.update_costmap(new_costmap)  # calls compute_shortest_path internally

            path_cells = self._dstar.extract_path()
            if path_cells is None:
                if not self._path_still_valid():
                    self._publish_status('PLANNING_FAILED', 'no path found after costmap update')
                    self.get_logger().warn('D* Lite: no path found after dynamic update')
                return

            self._last_path_cells = path_cells
            self._publish_path(path_cells)
        else:
            # No D* Lite instance yet or no previous costmap — full replan
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
        """Full D* Lite initialisation from scratch (new goal or new map)."""
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

        # Build D* Lite from scratch
        self._dstar = DStarLite(self._costmap, start_free, goal_free)
        self._dstar.compute_shortest_path()

        path_cells = self._dstar.extract_path()

        if path_cells is None:
            self._publish_status('PLANNING_FAILED', 'no path found')
            self.get_logger().warn('D* Lite: no path found')
            return

        self._last_path_cells = path_cells
        self._publish_path(path_cells)

    def _publish_path(self, path_cells: list[tuple[int, int]]) -> None:
        """Convert cell path to world-frame Path message and publish."""
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
        self.get_logger().info(f'D* Lite plan: {len(path_msg.poses)} waypoints')

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
