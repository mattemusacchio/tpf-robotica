"""Pure Pursuit path follower with final-yaw alignment.

Subscribes to /plan (nav_msgs/Path) and /pose_estimate, publishes /cmd_vel.
Two-phase control:
  FOLLOWING  — Pure Pursuit to track the path
  ALIGNING   — proportional angular control to reach goal yaw
"""

from __future__ import annotations

import json
import math
from enum import Enum, auto

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


def _normalize(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class _State(Enum):
    IDLE = auto()
    FOLLOWING = auto()
    ALIGNING = auto()
    DONE = auto()


class PurePursuit(Node):
    def __init__(self) -> None:
        super().__init__('pure_pursuit')

        self.declare_parameter('lookahead_m', 0.4)
        self.declare_parameter('v_max', 0.2)
        self.declare_parameter('omega_max', 0.8)
        self.declare_parameter('omega_align_max', 0.5)
        self.declare_parameter('tolerance_m', 0.15)
        self.declare_parameter('align_tolerance_rad', 0.087)   # ~5 deg
        self.declare_parameter('align_kp', 1.2)
        self.declare_parameter('v_ramp_start_m', 0.6)         # start slowing
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('pose_topic', '/pose_estimate')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('nav_status_topic', '/nav_status')

        self._Ld = float(self.get_parameter('lookahead_m').value)
        self._v_max = float(self.get_parameter('v_max').value)
        self._om_max = float(self.get_parameter('omega_max').value)
        self._om_align = float(self.get_parameter('omega_align_max').value)
        self._tol = float(self.get_parameter('tolerance_m').value)
        self._align_tol = float(self.get_parameter('align_tolerance_rad').value)
        self._align_kp = float(self.get_parameter('align_kp').value)
        self._ramp_d = float(self.get_parameter('v_ramp_start_m').value)

        self._state: _State = _State.IDLE
        self._path: list[tuple[float, float]] = []
        self._goal_yaw: float = 0.0

        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._robot_yaw: float = 0.0
        self._path_idx: int = 0       # index into path for lookahead search

        self._cmd_pub = self.create_publisher(
            Twist,
            str(self.get_parameter('cmd_vel_topic').value), 10)
        self._status_pub = self.create_publisher(
            String,
            str(self.get_parameter('nav_status_topic').value), 10)

        self.create_subscription(
            Path,
            str(self.get_parameter('plan_topic').value),
            self._on_plan, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter('pose_topic').value),
            self._on_pose, 10)
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('goal_topic').value),
            self._on_goal, 10)

        rate = float(self.get_parameter('control_rate_hz').value)
        self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info('Pure Pursuit controller ready')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_plan(self, msg: Path) -> None:
        self._path = [(p.pose.position.x, p.pose.position.y)
                      for p in msg.poses]
        self._path_idx = 0
        if self._path:
            self._state = _State.FOLLOWING
            self.get_logger().info(f'New plan received: {len(self._path)} waypoints')

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._robot_yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)

    def _on_goal(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        self._goal_yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        if self._state is _State.IDLE or self._state is _State.DONE:
            return

        if self._state is _State.FOLLOWING:
            self._follow()
        elif self._state is _State.ALIGNING:
            self._align()

    def _follow(self) -> None:
        if not self._path:
            self._stop()
            self._state = _State.IDLE
            return

        goal_x, goal_y = self._path[-1]
        dist_to_goal = math.hypot(goal_x - self._robot_x,
                                  goal_y - self._robot_y)

        if dist_to_goal < self._tol:
            self._stop()
            self._state = _State.ALIGNING
            self._publish_status('AT_POSITION')
            return

        lp = self._find_lookahead()
        if lp is None:
            lp = self._path[-1]

        dx = lp[0] - self._robot_x
        dy = lp[1] - self._robot_y
        alpha = _normalize(math.atan2(dy, dx) - self._robot_yaw)

        curvature = 2.0 * math.sin(alpha) / self._Ld

        # Ramp linear velocity down near goal
        v = self._v_max
        if dist_to_goal < self._ramp_d:
            v *= dist_to_goal / self._ramp_d
        v = max(0.05, v)

        omega = curvature * v
        omega = max(-self._om_max, min(self._om_max, omega))

        self._publish_cmd(v, omega)

    def _find_lookahead(self) -> tuple[float, float] | None:
        """Find the first path point at distance ≥ L_d from robot."""
        best = None
        for i in range(self._path_idx, len(self._path)):
            px, py = self._path[i]
            d = math.hypot(px - self._robot_x, py - self._robot_y)
            if d >= self._Ld:
                self._path_idx = i
                return px, py
            best = (px, py)
        return best  # path too short — use last point

    def _align(self) -> None:
        err = _normalize(self._goal_yaw - self._robot_yaw)
        if abs(err) < self._align_tol:
            self._stop()
            self._state = _State.DONE
            self._publish_status('GOAL_REACHED')
            self.get_logger().info('GOAL_REACHED')
            return

        omega = self._align_kp * err
        omega = max(-self._om_align, min(self._om_align, omega))
        self._publish_cmd(0.0, omega)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _publish_cmd(self, v: float, omega: float) -> None:
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = omega
        self._cmd_pub.publish(msg)

    def _stop(self) -> None:
        self._publish_cmd(0.0, 0.0)

    def _publish_status(self, status: str, detail: str = '') -> None:
        msg = String()
        msg.data = json.dumps({'status': status, 'detail': detail})
        self._status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
