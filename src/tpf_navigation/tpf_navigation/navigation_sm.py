from __future__ import annotations

import json
import math
from enum import Enum, auto
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                        ReliabilityPolicy, qos_profile_sensor_data)

_MAP_QOS = QoSProfile(
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class State(Enum):
    IDLE = auto()
    LOCALIZING = auto()
    WAIT_GOAL = auto()
    PLANNING = auto()
    FOLLOWING_PATH = auto()
    AVOIDING_OBSTACLE = auto()
    ALIGNING_FINAL_YAW = auto()
    GOAL_REACHED = auto()
    ERROR_RECOVERY = auto()


_OBSTACLE_DIST_TRIGGER = 1.0
_OBSTACLE_DETECT_RANGE = 1.5
_OBSTACLE_FREE_THRESH = 50
_OBSTACLE_CELL_SIZE = 0.05
_OBSTACLE_HIT_REQUIRED = 3
_OBSTACLE_DECAY_S = 5.0
_COV_DIAG_THRESH = 1.0
_LOCALIZING_TIMEOUT_S = 60.0
_PLANNING_TIMEOUT_S = 5.0
_GOAL_REACHED_DWELL_S = 2.0
_AVOID_WAIT_S = 0.5
_MAX_RECOVERIES = 3
_RECOVERY_ROTATE_RAD = math.radians(180.0)


class NavigationSM(Node):
    def __init__(self) -> None:
        super().__init__('navigation_sm')

        self.declare_parameter('obstacle_stop_range_m', _OBSTACLE_DIST_TRIGGER)
        self.declare_parameter('obstacle_range_m', _OBSTACLE_DETECT_RANGE)
        self.declare_parameter('obstacle_confirm_hits', _OBSTACLE_HIT_REQUIRED)
        self.declare_parameter('obstacle_decay_s', _OBSTACLE_DECAY_S)
        self.declare_parameter('localized_cov_thresh', _COV_DIAG_THRESH)
        self.declare_parameter('localizing_timeout_s', _LOCALIZING_TIMEOUT_S)
        self.declare_parameter('planning_timeout_s', _PLANNING_TIMEOUT_S)

        self._obs_stop_range = float(self.get_parameter('obstacle_stop_range_m').value)
        self._obs_detect_range = float(self.get_parameter('obstacle_range_m').value)
        self._obs_confirm_hits = int(self.get_parameter('obstacle_confirm_hits').value)
        self._obs_decay_s = float(self.get_parameter('obstacle_decay_s').value)
        self._cov_thresh = float(self.get_parameter('localized_cov_thresh').value)
        self._localizing_timeout = float(self.get_parameter('localizing_timeout_s').value)
        self._planning_timeout = float(self.get_parameter('planning_timeout_s').value)

        self._state = State.IDLE
        self._state_entry_time: float = self.get_clock().now().nanoseconds * 1e-9

        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._robot_yaw: float = 0.0
        self._localized: bool = False

        self._current_goal: PoseStamped | None = None
        self._last_nav_status: str = ''

        self._map: OccupancyGrid | None = None

        # obstacle accumulator: (cell_x, cell_y) -> [hit_count, last_seen_time, consecutive_callbacks]
        self._obstacle_hits: dict[tuple[int, int], list[Any]] = {}
        self._confirmed_obstacles: set[tuple[int, int]] = set()

        self._recovery_count: int = 0
        self._recovery_rotating: bool = False
        self._recovery_rotate_start_yaw: float = 0.0
        self._recovery_rotate_started: bool = False

        self._avoid_entry_time: float = 0.0

        # Stuck detection: track last known position during path following
        self._stuck_check_x: float = 0.0
        self._stuck_check_y: float = 0.0
        self._stuck_check_time: float = 0.0
        self._STUCK_TIMEOUT_S: float = 8.0
        self._STUCK_DIST_M: float = 0.05

        qos = qos_profile_sensor_data

        self.create_subscription(PoseWithCovarianceStamped, '/pose_estimate', self._cb_pose, qos)
        self.create_subscription(String, '/nav_status', self._cb_nav_status, 10)
        self.create_subscription(LaserScan, '/scan', self._cb_scan, qos)
        self.create_subscription(OccupancyGrid, '/map', self._cb_map, _MAP_QOS)
        self.create_subscription(PoseStamped, '/goal_pose', self._cb_goal_pose, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self._cb_initialpose, 10)

        self._pub_nav_state = self.create_publisher(String, '/nav_state', 10)
        self._pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self._pub_obstacles = self.create_publisher(MarkerArray, '/detected_obstacles', 10)
        self._pub_goal_pose = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.create_timer(0.1, self._tick)
        self.create_timer(0.5, self._pub_state_name)

    # ------------------------------------------------------------------ #
    # Subscriptions                                                        #
    # ------------------------------------------------------------------ #

    def _cb_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._robot_x = p.x
        self._robot_y = p.y
        self._robot_yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)

        cov = msg.pose.covariance
        diag_sum = cov[0] + cov[7] + cov[35]
        was_localized = self._localized
        self._localized = diag_sum < self._cov_thresh
        if was_localized and not self._localized:
            # MCL just went LOST — stale obstacle detections would be at wrong positions
            self._obstacle_hits.clear()
            self._confirmed_obstacles.clear()

    def _cb_nav_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._last_nav_status = data.get('status', '')
        except (json.JSONDecodeError, AttributeError):
            self._last_nav_status = msg.data

    def _cb_map(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _cb_goal_pose(self, msg: PoseStamped) -> None:
        # Exclude PLANNING: SM publishes to /goal_pose for replanning and
        # would otherwise re-trigger itself from its own echo.
        if self._state in (State.FOLLOWING_PATH, State.ALIGNING_FINAL_YAW,
                           State.WAIT_GOAL, State.GOAL_REACHED,
                           State.ERROR_RECOVERY):
            self._current_goal = msg
            self._recovery_count = 0
            self._transition(State.PLANNING)

    def _cb_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        if self._state in (State.IDLE, State.ERROR_RECOVERY, State.WAIT_GOAL):
            self._recovery_count = 0
            self._transition(State.LOCALIZING)

    def _cb_scan(self, msg: LaserScan) -> None:
        if self._map is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        self._decay_obstacles(now)

        map_res = self._map.info.resolution
        map_ox = self._map.info.origin.position.x
        map_oy = self._map.info.origin.position.y
        map_w = self._map.info.width
        map_h = self._map.info.height
        map_data = self._map.data

        rx, ry, ryaw = self._robot_x, self._robot_y, self._robot_yaw
        # Laser mount — TurtleBot3 burger in Gazebo: x=-0.032m, no yaw offset
        _LASER_X = -0.032
        _LASER_YAW = 0.0
        sx = rx + _LASER_X * math.cos(ryaw)
        sy = ry + _LASER_X * math.sin(ryaw)

        cells_hit_this_cb: set[tuple[int, int]] = set()

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if not (msg.range_min <= r <= min(msg.range_max, self._obs_detect_range)):
                continue

            wx = sx + r * math.cos(ryaw + _LASER_YAW + angle)
            wy = sy + r * math.sin(ryaw + _LASER_YAW + angle)

            col = int((wx - map_ox) / map_res)
            row = int((wy - map_oy) / map_res)
            if not (0 <= col < map_w and 0 <= row < map_h):
                continue

            map_val = map_data[row * map_w + col]
            # hit in free cell → potential dynamic obstacle
            if 0 <= map_val < _OBSTACLE_FREE_THRESH:
                cx = int(wx / _OBSTACLE_CELL_SIZE)
                cy = int(wy / _OBSTACLE_CELL_SIZE)
                cells_hit_this_cb.add((cx, cy))

        for cell in cells_hit_this_cb:
            if cell not in self._obstacle_hits:
                self._obstacle_hits[cell] = [0, now, 0]
            entry = self._obstacle_hits[cell]
            entry[0] += 1
            entry[1] = now
            entry[2] += 1
            if entry[2] >= self._obs_confirm_hits:
                self._confirmed_obstacles.add(cell)

        self._publish_obstacle_markers()

        if self._state == State.FOLLOWING_PATH:
            for cell in self._confirmed_obstacles:
                wx = cell[0] * _OBSTACLE_CELL_SIZE
                wy = cell[1] * _OBSTACLE_CELL_SIZE
                dist = math.hypot(wx - rx, wy - ry)
                if dist < self._obs_stop_range:
                    self._stop_robot()
                    self._avoid_entry_time = now
                    self._transition(State.AVOIDING_OBSTACLE)
                    break

    # ------------------------------------------------------------------ #
    # Main loop                                                            #
    # ------------------------------------------------------------------ #

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self._state_entry_time
        s = self._state

        if s == State.LOCALIZING:
            if self._localized:
                self._stop_robot()
                self._transition(State.WAIT_GOAL)
            elif elapsed > self._localizing_timeout:
                self._stop_robot()
                self.get_logger().warn('Localization timed out after 60s')
                self._transition(State.ERROR_RECOVERY)
            else:
                twist = Twist()
                twist.angular.z = 0.3
                self._pub_cmd_vel.publish(twist)

        elif s == State.PLANNING:
            status = self._last_nav_status
            if status == 'PLANNING_OK':
                self._last_nav_status = ''
                self._transition(State.FOLLOWING_PATH)
            elif status == 'PLANNING_FAILED':
                self._last_nav_status = ''
                self._transition(State.ERROR_RECOVERY)
            elif elapsed > self._planning_timeout:
                self.get_logger().warn('Planning timed out after 5s')
                self._transition(State.ERROR_RECOVERY)

        elif s == State.FOLLOWING_PATH:
            # Abort if MCL lost track (e.g. robot fell or collided badly)
            if not self._localized:
                self.get_logger().warn('MCL LOST during navigation — aborting to LOCALIZING')
                self._stop_robot()
                self._transition(State.LOCALIZING)
                return

            # Stuck detection: abort if robot hasn't moved in _STUCK_TIMEOUT_S seconds
            dist_moved = math.hypot(self._robot_x - self._stuck_check_x,
                                    self._robot_y - self._stuck_check_y)
            if dist_moved > self._STUCK_DIST_M:
                self._stuck_check_x = self._robot_x
                self._stuck_check_y = self._robot_y
                self._stuck_check_time = now
            elif now - self._stuck_check_time > self._STUCK_TIMEOUT_S:
                self.get_logger().warn('Robot stuck — triggering recovery')
                self._stop_robot()
                self._transition(State.ERROR_RECOVERY)
                return

            status = self._last_nav_status
            if status == 'AT_POSITION':
                self._last_nav_status = ''
                self._transition(State.ALIGNING_FINAL_YAW)
            elif status == 'GOAL_REACHED':
                self._last_nav_status = ''
                self._transition(State.GOAL_REACHED)

        elif s == State.AVOIDING_OBSTACLE:
            if self._last_nav_status == 'GOAL_REACHED':
                self._last_nav_status = ''
                self._transition(State.GOAL_REACHED)
            elif elapsed >= _AVOID_WAIT_S:
                self._transition(State.PLANNING)

        elif s == State.ALIGNING_FINAL_YAW:
            status = self._last_nav_status
            if status == 'GOAL_REACHED':
                self._last_nav_status = ''
                self._transition(State.GOAL_REACHED)

        elif s == State.GOAL_REACHED:
            if elapsed >= _GOAL_REACHED_DWELL_S:
                self._transition(State.WAIT_GOAL)

        elif s == State.ERROR_RECOVERY:
            self._run_recovery(now)

    def _pub_state_name(self) -> None:
        msg = String()
        msg.data = self._state.name
        self._pub_nav_state.publish(msg)

    # ------------------------------------------------------------------ #
    # Transitions                                                          #
    # ------------------------------------------------------------------ #

    def _transition(self, new_state: State) -> None:
        old = self._state
        self._state = new_state
        self._state_entry_time = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(f'State transition: {old.name} → {new_state.name}')

        self._pub_state_name()

        if new_state == State.PLANNING and self._current_goal is not None:
            self._publish_goal(self._current_goal)

        if new_state == State.FOLLOWING_PATH:
            self._stuck_check_x = self._robot_x
            self._stuck_check_y = self._robot_y
            self._stuck_check_time = self._state_entry_time

        if new_state == State.ERROR_RECOVERY:
            self._recovery_rotating = False
            self._recovery_rotate_started = False

    # ------------------------------------------------------------------ #
    # Recovery                                                             #
    # ------------------------------------------------------------------ #

    def _run_recovery(self, now: float) -> None:
        if self._recovery_count >= _MAX_RECOVERIES:
            self.get_logger().warn(
                f'Maximum recoveries ({_MAX_RECOVERIES}) reached — staying in ERROR_RECOVERY'
            )
            self._stop_robot()
            return

        if not self._recovery_rotating:
            self._stop_robot()
            self._recovery_rotating = True
            self._recovery_rotate_started = False
            return

        if not self._recovery_rotate_started:
            self._recovery_rotate_start_yaw = self._robot_yaw
            self._recovery_rotate_started = True

        rotated = abs(
            math.atan2(
                math.sin(self._robot_yaw - self._recovery_rotate_start_yaw),
                math.cos(self._robot_yaw - self._recovery_rotate_start_yaw),
            )
        )

        if rotated < _RECOVERY_ROTATE_RAD:
            twist = Twist()
            twist.angular.z = 0.4
            self._pub_cmd_vel.publish(twist)
        else:
            self._stop_robot()
            self._recovery_count += 1
            self._recovery_rotating = False
            self._recovery_rotate_started = False
            self.get_logger().info(
                f'Recovery maneuver {self._recovery_count} complete — retrying planning'
            )
            if self._current_goal is not None:
                self._transition(State.PLANNING)
            else:
                self._transition(State.WAIT_GOAL)

    # ------------------------------------------------------------------ #
    # Helpers                                                              #
    # ------------------------------------------------------------------ #

    def _stop_robot(self) -> None:
        self._pub_cmd_vel.publish(Twist())

    def _publish_goal(self, goal: PoseStamped) -> None:
        self._pub_goal_pose.publish(goal)

    def _decay_obstacles(self, now: float) -> None:
        expired = [
            cell for cell, entry in self._obstacle_hits.items()
            if now - entry[1] > self._obs_decay_s
        ]
        for cell in expired:
            del self._obstacle_hits[cell]
            self._confirmed_obstacles.discard(cell)

    def _publish_obstacle_markers(self) -> None:
        arr = MarkerArray()
        for i, cell in enumerate(self._confirmed_obstacles):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'obstacles'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = cell[0] * _OBSTACLE_CELL_SIZE
            m.pose.position.y = cell[1] * _OBSTACLE_CELL_SIZE
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = _OBSTACLE_CELL_SIZE * 2
            m.scale.y = _OBSTACLE_CELL_SIZE * 2
            m.scale.z = _OBSTACLE_CELL_SIZE * 2
            m.color.r = 1.0
            m.color.a = 0.8
            arr.markers.append(m)
        self._pub_obstacles.publish(arr)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = NavigationSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
