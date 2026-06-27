#!/usr/bin/env python3
"""
Waypoint-based coverage explorer for casa.world.

World geometry summary:
  Outer walls: x in [-2.95, 2.95], y in [-2.95, 2.95]
  Wall_40 (x=-0.95, y=-1.55..2.0): divides left/right — gap ONLY above y=2.0
  Wall_25 (y=-1.5, x=-3..2.0): top/bottom divider — gap at x>2.0
  Wall_42 (x=-3..-2.25, y=0.5) + Wall_44 (x=-1.65..-0.9, y=0.5):
    divide upper/lower-left room — gap at x=-2.25..-1.65 (center x=-1.95)
  Wall_35 (x=1.95, y=-2.0..-1.45), Wall_31 (x=0, y=-2.0..-1.45),
  Wall_38 (x=-1.95, y=-2.0..-1.45): vertical dividers in bottom corridor
  Wall_33 (x=0.95, y=-3.0..-2.5), Wall_29 (x=-0.95, y=-3.0..-2.5): bottom dividers
  Table center (~1.44, -0.044), legs r=2cm at (2.12,0.34),(2.12,-0.42),(0.76,0.34),(0.76,-0.42)
  HospitalBot at (-1.924, -0.489), collision box 0.7x0.7 → x:[-2.27,-1.57], y:[-0.84,-0.14]
  Sofa at (1.0, 1.33)
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

WAYPOINTS = [
    # 1. Main room — escape south of table, sweep right wall
    ( 0.0, -0.9),   # south of table (clear path)
    ( 2.3, -0.9),   # right wall below table
    ( 2.3,  1.5),   # right wall upper

    # 2. Upper right → through Wall_40 gap (y>2.0) into upper-left room
    ( 1.5,  2.5),   # upper right
    ( 0.0,  2.5),   # upper center
    (-0.5,  2.5),   # just inside Wall_40 gap
    (-1.5,  2.5),   # upper-left room
    (-2.3,  2.5),   # far upper-left corner
    (-2.3,  1.0),   # sweep down upper-left (above door at y=0.5)

    # 3. Enter lower-left room through door gap (x=-2.25..-1.65 at y=0.5, center x=-1.95)
    #    Robot 0.22m wide, gap 0.6m wide — sufficient clearance
    (-1.95, 0.65),  # approach door from above
    (-1.95, 0.10),  # through door into lower-left room
    (-2.5,  0.10),  # upper-left corner of lower room (avoid HospitalBot)
    (-2.5, -1.2),   # lower-left corner
    (-1.2, -1.2),   # lower-right (clear of HospitalBot right edge at x=-1.57)

    # 4. Exit lower-left through door, return north to Wall_40 gap
    (-1.95, 0.65),  # back through door into upper-left
    (-2.0,  2.5),   # north above Wall_40 end (y=2.0)
    ( 0.0,  2.5),   # through Wall_40 gap back to main room

    # 5. South sweep to bottom corridor entry
    ( 0.0, -0.9),   # south of table
    ( 2.3, -0.9),   # right wall south
    ( 2.3, -1.3),   # approaching Wall_25 gap (gap at x>2.0)

    # 6. Bottom corridor — navigate at y=-2.15, between upper walls (end at y=-2.0)
    #    and lower walls (start at y=-2.5). Full horizontal sweep.
    ( 2.3, -2.15),  # enter corridor (below all vertical walls)
    ( 0.0, -2.15),  # sweep left through center
    (-2.3, -2.15),  # far left of corridor
]


class CasaWaypointExplorer(Node):
    FORWARD_SPEED   = 0.22   # m/s
    TURN_SPEED      = 0.80   # rad/s (reactive)
    SAFE_DIST_M     = 0.30   # front obstacle threshold
    SIDE_DIST_M     = 0.20   # side wall follow threshold
    GOAL_DIST_M     = 0.20   # waypoint accepted radius
    KP_ANG          = 2.5    # proportional heading gain
    MAX_ANG_VEL     = 1.2    # rad/s cap
    WP_TIMEOUT_S    = 45.0   # skip waypoint if not reached in this time
    MAX_DURATION_S  = 1000   # hard stop

    def __init__(self) -> None:
        super().__init__('casa_explorer')
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry,  '/odom', self._on_odom, qos_profile_sensor_data)

        self._x   = 0.0
        self._y   = 0.0
        self._yaw = 0.0
        self._scan: LaserScan | None = None

        self._start       = time.time()
        self._wp_idx      = 0
        self._wp_start    = time.time()
        self._done        = False

        self.create_timer(0.10, self._control_loop)
        self.get_logger().info(
            f'Casa explorer v3 started — {len(WAYPOINTS)} waypoints, '
            f'{self.MAX_DURATION_S}s limit, timeout {self.WP_TIMEOUT_S}s/wp')

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._x = float(p.x)
        self._y = float(p.y)
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny, cosy)

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg

    def _min_in_cone(self, center_deg: float, half_deg: float) -> float:
        if self._scan is None:
            return float('inf')
        ai  = float(self._scan.angle_min)
        inc = float(self._scan.angle_increment)
        lo  = math.radians(center_deg - half_deg)
        hi  = math.radians(center_deg + half_deg)
        vals = [
            r for i, r in enumerate(self._scan.ranges)
            if lo <= (ai + i * inc) <= hi and math.isfinite(r) and r > 0.05
        ]
        return min(vals) if vals else float('inf')

    def _control_loop(self) -> None:
        if self._done:
            self._pub.publish(Twist())
            return

        now     = time.time()
        elapsed = now - self._start

        if elapsed > self.MAX_DURATION_S:
            self._stop('Time limit reached.')
            return

        if self._wp_idx >= len(WAYPOINTS):
            self._stop('All waypoints visited!')
            return

        if now - self._wp_start > self.WP_TIMEOUT_S:
            wx, wy = WAYPOINTS[self._wp_idx]
            self.get_logger().warn(
                f'WP {self._wp_idx} ({wx:.1f},{wy:.1f}) timeout — skipping')
            self._wp_idx  += 1
            self._wp_start = now
            return

        front = self._min_in_cone(0.0,   25.0)
        left  = self._min_in_cone(90.0,  40.0)
        right = self._min_in_cone(-90.0, 40.0)

        twist = Twist()

        if front < self.SAFE_DIST_M:
            twist.angular.z = self.TURN_SPEED if left >= right else -self.TURN_SPEED
            self._pub.publish(twist)
            return

        wx, wy = WAYPOINTS[self._wp_idx]
        dx     = wx - self._x
        dy     = wy - self._y
        dist   = math.hypot(dx, dy)

        if dist < self.GOAL_DIST_M:
            self.get_logger().info(
                f'WP {self._wp_idx}/{len(WAYPOINTS)-1} reached '
                f'({wx:.1f},{wy:.1f})  pos=({self._x:.2f},{self._y:.2f})'
                f'  t={elapsed:.0f}s')
            self._wp_idx  += 1
            self._wp_start = now
            return

        target_yaw = math.atan2(dy, dx)
        err = target_yaw - self._yaw
        while err >  math.pi: err -= 2 * math.pi
        while err < -math.pi: err += 2 * math.pi

        ang = max(-self.MAX_ANG_VEL, min(self.MAX_ANG_VEL, self.KP_ANG * err))
        fwd = self.FORWARD_SPEED * max(0.0, 1.0 - abs(err) / (math.pi / 2))

        if left < self.SIDE_DIST_M:
            ang = max(-self.MAX_ANG_VEL, ang - 0.4)
        elif right < self.SIDE_DIST_M:
            ang = min( self.MAX_ANG_VEL, ang + 0.4)

        twist.linear.x  = fwd
        twist.angular.z = ang
        self._pub.publish(twist)

    def _stop(self, reason: str) -> None:
        self._done = True
        self._pub.publish(Twist())
        self.get_logger().info(reason)


def main():
    rclpy.init()
    node = CasaWaypointExplorer()
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node._pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
