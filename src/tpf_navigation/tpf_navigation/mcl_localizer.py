"""Monte Carlo Localizer (MCL / AMCL) for Parte B.

Implements:
  - Odometry motion model (probabilistic robotics Ch. 5.4)
  - Likelihood-field LIDAR observation model (Ch. 6.4)
  - Landmark range/bearing observation model
  - Low-variance resampling
  - TF map→odom broadcast + PoseWithCovarianceStamped + PoseArray
"""

from __future__ import annotations

import json
import math
from typing import Any

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import (PoseArray, PoseStamped,
                                PoseWithCovarianceStamped,
                                TransformStamped)
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                        ReliabilityPolicy, qos_profile_sensor_data)

_MAP_QOS = QoSProfile(
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
from scipy.ndimage import distance_transform_edt
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def _normalize(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class MCLLocalizer(Node):
    def __init__(self) -> None:
        super().__init__('mcl_localizer')

        self.declare_parameter('num_particles', 500)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('detections_topic', '/aruco/detections')
        self.declare_parameter('landmarks_file', '')
        # Motion model noise (probabilistic robotics Table 5.6)
        self.declare_parameter('alpha1', 0.1)   # rot error from rot
        self.declare_parameter('alpha2', 0.1)   # rot error from trans
        self.declare_parameter('alpha3', 0.05)  # trans error from trans
        self.declare_parameter('alpha4', 0.05)  # trans error from rot
        # LIDAR model
        self.declare_parameter('lidar_sigma_m', 0.1)
        self.declare_parameter('lidar_ray_stride', 10)
        self.declare_parameter('lidar_max_range_m', 3.5)
        self.declare_parameter('lidar_weight', 1.0)
        # Landmark model
        self.declare_parameter('lm_sigma_range', 0.15)
        self.declare_parameter('lm_sigma_bearing', 0.10)
        self.declare_parameter('lm_weight', 2.0)
        # Publish
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('lost_var_thresh', 2.0)  # m² variance → LOST

        self._N = int(self.get_parameter('num_particles').value)
        self._alpha = [
            float(self.get_parameter(f'alpha{i}').value) for i in range(1, 5)
        ]
        self._lidar_sigma = float(self.get_parameter('lidar_sigma_m').value)
        self._lidar_stride = int(self.get_parameter('lidar_ray_stride').value)
        self._lidar_max = float(self.get_parameter('lidar_max_range_m').value)
        self._lidar_w = float(self.get_parameter('lidar_weight').value)
        self._lm_sr = float(self.get_parameter('lm_sigma_range').value)
        self._lm_sb = float(self.get_parameter('lm_sigma_bearing').value)
        self._lm_w = float(self.get_parameter('lm_weight').value)
        self._lost_thresh = float(self.get_parameter('lost_var_thresh').value)

        lm_file = str(self.get_parameter('landmarks_file').value)
        if lm_file:
            with open(lm_file) as f:
                data = yaml.safe_load(f)
            self._landmarks: dict[int, tuple[float, float]] = {
                int(lm['id']): (float(lm['x']), float(lm['y']))
                for lm in data['landmarks']
            }
        else:
            self._landmarks = {}

        # Particles: Nx3 [x, y, theta]; weights: N
        self._particles: np.ndarray | None = None
        self._weights: np.ndarray = np.ones(self._N) / self._N

        # Map / likelihood field
        self._map_info: Any = None
        self._lf: np.ndarray | None = None      # likelihood field (m)
        self._free_cells: np.ndarray | None = None  # Mx2 free cell world coords

        # Odometry bookkeeping
        self._prev_odom: tuple[float, float, float] | None = None
        self._pending_lidar: LaserScan | None = None
        self._pending_detections: list[dict] = []
        self._odom_x: float = 0.0
        self._odom_y: float = 0.0
        self._odom_yaw: float = 0.0

        self._tf_broadcaster = TransformBroadcaster(self)

        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/pose_estimate', 10)
        self._cloud_pub = self.create_publisher(PoseArray, '/particle_cloud', 10)

        self.create_subscription(
            OccupancyGrid, str(self.get_parameter('map_topic').value),
            self._on_map, _MAP_QOS)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value),
            self._on_odom, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, str(self.get_parameter('scan_topic').value),
            self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            String, str(self.get_parameter('detections_topic').value),
            self._on_detections, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose',
            self._on_initialpose, 10)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(f'MCL ready: {self._N} particles')

    # ------------------------------------------------------------------
    # Map
    # ------------------------------------------------------------------

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map_info = msg.info
        grid = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))
        obstacle_mask = (grid >= 50).astype(np.uint8)
        # distance_transform_edt returns distance to nearest zero (free) cell
        # We want distance to nearest occupied cell → invert
        dist_cells = distance_transform_edt(1 - obstacle_mask)
        self._lf = dist_cells * msg.info.resolution   # metres

        free_mask = (grid >= 0) & (grid < 50)
        rows, cols = np.where(free_mask)
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        res = msg.info.resolution
        self._free_cells = np.column_stack([
            ox + (cols + 0.5) * res,
            oy + (rows + 0.5) * res,
        ])

        if self._particles is None:
            self._init_uniform()
        self.get_logger().info(
            f'Map received {msg.info.width}x{msg.info.height}; '
            f'{len(self._free_cells)} free cells', once=True)

    def _init_uniform(self) -> None:
        if self._free_cells is None or len(self._free_cells) == 0:
            return
        idx = np.random.choice(len(self._free_cells), self._N, replace=True)
        xy = self._free_cells[idx]
        theta = np.random.uniform(-math.pi, math.pi, self._N)
        self._particles = np.column_stack([xy, theta])
        self._weights = np.ones(self._N) / self._N

    # ------------------------------------------------------------------
    # Initial pose
    # ------------------------------------------------------------------

    def _on_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        q = msg.pose.pose.orientation
        yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
        cov = np.array(msg.pose.covariance).reshape(6, 6)
        std_xy = math.sqrt(max(cov[0, 0], cov[1, 1], 0.01))
        std_th = math.sqrt(max(cov[5, 5], 0.01))
        cx = msg.pose.pose.position.x
        cy = msg.pose.pose.position.y
        xs = np.random.normal(cx, std_xy, self._N)
        ys = np.random.normal(cy, std_xy, self._N)
        ths = np.random.normal(yaw, std_th, self._N)
        self._particles = np.column_stack([xs, ys, ths])
        self._weights = np.ones(self._N) / self._N
        self.get_logger().info(f'Particles reset around ({cx:.2f}, {cy:.2f})')

    # ------------------------------------------------------------------
    # Odometry → motion model
    # ------------------------------------------------------------------

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        self._odom_yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)

        if self._particles is None:
            self._prev_odom = (self._odom_x, self._odom_y, self._odom_yaw)
            return

        if self._prev_odom is None:
            self._prev_odom = (self._odom_x, self._odom_y, self._odom_yaw)
            return

        px, py, pyaw = self._prev_odom
        dx = self._odom_x - px
        dy = self._odom_y - py
        dtheta = _normalize(self._odom_yaw - pyaw)
        trans = math.hypot(dx, dy)

        if trans < 0.001 and abs(dtheta) < 0.001:
            return  # robot not moving — skip predict to avoid particle spread

        self._predict(dx, dy, dtheta, trans)
        self._prev_odom = (self._odom_x, self._odom_y, self._odom_yaw)

        # Sensor update if we have pending data
        if self._pending_lidar is not None:
            self._update_lidar(self._pending_lidar)
            self._pending_lidar = None
        if self._pending_detections:
            self._update_landmarks(self._pending_detections)
            self._pending_detections = []
        self._normalize_and_resample()

    def _predict(self, dx: float, dy: float, dtheta: float, trans: float) -> None:
        a1, a2, a3, a4 = self._alpha
        N = self._N
        rot1 = math.atan2(dy, dx) if trans > 0.01 else 0.0
        rot2 = _normalize(dtheta - rot1)

        rot1_hat = rot1 - np.random.normal(0, a1 * abs(rot1) + a2 * trans, N)
        trans_hat = trans - np.random.normal(0, a3 * trans + a4 * (abs(rot1) + abs(rot2)), N)
        rot2_hat = rot2 - np.random.normal(0, a1 * abs(rot2) + a2 * trans, N)

        self._particles[:, 0] += trans_hat * np.cos(self._particles[:, 2] + rot1_hat)
        self._particles[:, 1] += trans_hat * np.sin(self._particles[:, 2] + rot1_hat)
        self._particles[:, 2] = _normalize(self._particles[:, 2] + rot1_hat + rot2_hat)

    # ------------------------------------------------------------------
    # LIDAR observation model (likelihood field)
    # ------------------------------------------------------------------

    def _on_scan(self, msg: LaserScan) -> None:
        self._pending_lidar = msg

    def _update_lidar(self, scan: LaserScan) -> None:
        if self._lf is None or self._map_info is None:
            return
        angles = (scan.angle_min
                  + np.arange(len(scan.ranges)) * scan.angle_increment)
        ranges = np.asarray(scan.ranges, dtype=np.float32)
        stride = self._lidar_stride
        valid = (ranges[::stride] > scan.range_min) & (ranges[::stride] < self._lidar_max)
        r_sel = ranges[::stride][valid]
        a_sel = angles[::stride][valid]
        if len(r_sel) == 0:
            return

        res = self._map_info.resolution
        ox = self._map_info.origin.position.x
        oy = self._map_info.origin.position.y
        h = self._map_info.height
        w = self._map_info.width
        sigma = self._lidar_sigma
        inv2s2 = 1.0 / (2.0 * sigma ** 2)

        log_w = np.zeros(self._N)
        px = self._particles[:, 0]
        py = self._particles[:, 1]
        pth = self._particles[:, 2]

        for r, a in zip(r_sel, a_sel):
            # endpoint in world frame for each particle
            wx = px + r * np.cos(pth + a)
            wy = py + r * np.sin(pth + a)
            cols = ((wx - ox) / res).astype(int)
            rows = ((wy - oy) / res).astype(int)
            in_bounds = (cols >= 0) & (cols < w) & (rows >= 0) & (rows < h)
            d = np.where(in_bounds, self._lf[
                np.clip(rows, 0, h - 1), np.clip(cols, 0, w - 1)], sigma * 3)
            log_w += -d * d * inv2s2

        self._weights *= np.exp(log_w * self._lidar_w)

    # ------------------------------------------------------------------
    # Landmark observation model
    # ------------------------------------------------------------------

    def _on_detections(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            dets = payload.get('detections', [])
        except (json.JSONDecodeError, AttributeError):
            return
        if dets:
            self._pending_detections = dets

    def _update_landmarks(self, detections: list[dict]) -> None:
        if not self._landmarks or self._particles is None:
            return
        sr, sb = self._lm_sr, self._lm_sb
        inv2sr2 = 1.0 / (2.0 * sr ** 2)
        inv2sb2 = 1.0 / (2.0 * sb ** 2)

        log_w = np.zeros(self._N)
        for det in detections:
            lm_id = int(det.get('id', -1))
            obs = det.get('base_link_approx', {})
            meas_r = obs.get('range')
            meas_b = obs.get('bearing')
            if lm_id not in self._landmarks or meas_r is None:
                continue
            lm_pos = self._landmarks[lm_id]
            dx = lm_pos[0] - self._particles[:, 0]
            dy = lm_pos[1] - self._particles[:, 1]
            exp_r = np.hypot(dx, dy)
            exp_b = np.arctan2(dy, dx) - self._particles[:, 2]
            exp_b = np.arctan2(np.sin(exp_b), np.cos(exp_b))

            dr = float(meas_r) - exp_r
            db = _normalize(float(meas_b) - exp_b)
            log_w += -dr * dr * inv2sr2 - db * db * inv2sb2

        self._weights *= np.exp(log_w * self._lm_w)

    # ------------------------------------------------------------------
    # Resampling
    # ------------------------------------------------------------------

    def _normalize_and_resample(self) -> None:
        total = self._weights.sum()
        if total < 1e-300:
            self._weights = np.ones(self._N) / self._N
            return
        self._weights /= total
        neff = 1.0 / (self._weights ** 2).sum()
        if neff < self._N / 2.0:
            self._low_variance_resample()

    def _low_variance_resample(self) -> None:
        cumsum = np.cumsum(self._weights)
        r = np.random.uniform(0, 1.0 / self._N)
        step = 1.0 / self._N
        idx = np.searchsorted(cumsum, r + np.arange(self._N) * step)
        idx = np.clip(idx, 0, self._N - 1)
        self._particles = self._particles[idx]
        self._weights = np.ones(self._N) / self._N

    # ------------------------------------------------------------------
    # Publish
    # ------------------------------------------------------------------

    def _weighted_mean_pose(self) -> tuple[float, float, float]:
        w = self._weights
        x = float(np.sum(w * self._particles[:, 0]))
        y = float(np.sum(w * self._particles[:, 1]))
        theta = float(math.atan2(
            float(np.sum(w * np.sin(self._particles[:, 2]))),
            float(np.sum(w * np.cos(self._particles[:, 2]))),
        ))
        return x, y, theta

    def _publish(self) -> None:
        if self._particles is None:
            return

        now = self.get_clock().now().to_msg()
        est_x, est_y, est_yaw = self._weighted_mean_pose()

        # Variance estimate
        var_x = float(np.sum(self._weights * (self._particles[:, 0] - est_x) ** 2))
        var_y = float(np.sum(self._weights * (self._particles[:, 1] - est_y) ** 2))
        lost = (var_x + var_y) > self._lost_thresh

        # PoseWithCovarianceStamped
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = est_x
        pose_msg.pose.pose.position.y = est_y
        qx, qy, qz, qw = _quat_from_yaw(est_yaw)
        pose_msg.pose.pose.orientation.x = qx
        pose_msg.pose.pose.orientation.y = qy
        pose_msg.pose.pose.orientation.z = qz
        pose_msg.pose.pose.orientation.w = qw
        cov = [0.0] * 36
        cov[0] = var_x
        cov[7] = var_y
        cov[35] = float(np.sum(self._weights * _normalize(self._particles[:, 2] - est_yaw) ** 2))
        pose_msg.pose.covariance = cov
        self._pose_pub.publish(pose_msg)

        # map→odom TF: T_map_odom = T_map_base * inv(T_odom_base)
        # T_odom_base from /odom:
        cos_e = math.cos(self._odom_yaw)
        sin_e = math.sin(self._odom_yaw)
        # inv(T_odom_base) applied to estimate:
        # t_map_odom = t_map_base - R_map_odom * t_odom_base
        # R_map_odom = R_map_base * R_base_odom = R_map_base * inv(R_odom_base)
        d_yaw = est_yaw - self._odom_yaw
        cos_d = math.cos(d_yaw)
        sin_d = math.sin(d_yaw)
        tx = est_x - (cos_d * self._odom_x - sin_d * self._odom_y)
        ty = est_y - (sin_d * self._odom_x + cos_d * self._odom_y)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = 'odom'
        tf_msg.transform.translation.x = tx
        tf_msg.transform.translation.y = ty
        tf_msg.transform.translation.z = 0.0
        qx2, qy2, qz2, qw2 = _quat_from_yaw(d_yaw)
        tf_msg.transform.rotation.x = qx2
        tf_msg.transform.rotation.y = qy2
        tf_msg.transform.rotation.z = qz2
        tf_msg.transform.rotation.w = qw2
        self._tf_broadcaster.sendTransform(tf_msg)

        # Particle cloud
        cloud = PoseArray()
        cloud.header.stamp = now
        cloud.header.frame_id = 'map'
        from geometry_msgs.msg import Pose
        for p in self._particles:
            pose = Pose()
            pose.position.x = float(p[0])
            pose.position.y = float(p[1])
            qx3, qy3, qz3, qw3 = _quat_from_yaw(float(p[2]))
            pose.orientation.x = qx3
            pose.orientation.y = qy3
            pose.orientation.z = qz3
            pose.orientation.w = qw3
            cloud.poses.append(pose)
        self._cloud_pub.publish(cloud)

        if lost:
            self.get_logger().warn(
                f'LOST: position variance {var_x + var_y:.2f} > {self._lost_thresh}',
                throttle_duration_sec=5.0,
            )


def main() -> None:
    rclpy.init()
    node = MCLLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
