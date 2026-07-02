"""Offline Graph SLAM back-end for pose-landmark graphs.

The front-end writes a JSON graph with odometry and ArUco range/bearing edges.
This module optimizes that graph offline with nonlinear least squares and exports
artifacts that can be plotted or fed into the occupancy-map phase.
"""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass
from math import atan2, cos, hypot, sin
from pathlib import Path
from typing import Any

import numpy as np

try:  # pragma: no cover - exercised in integration in this ROS image
    from scipy.optimize import least_squares
    from scipy.sparse import csr_matrix, lil_matrix
except Exception:  # pragma: no cover - fallback path for minimal environments
    least_squares = None
    lil_matrix = None
    csr_matrix = None


PI = 3.141592653589793


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""

    return atan2(sin(angle), cos(angle))


@dataclass(frozen=True)
class BackendConfig:
    # Wheel/IMU odometry on this platform is locally very accurate, so it is the
    # backbone of the graph. ArUco range/bearing is noisier and has gross outliers
    # (180-degree flips, far/oblique misreads), so it is down-weighted and tamed by
    # a robust loss instead of being allowed to warp the trajectory.
    odom_translation_sigma: float = 0.03
    odom_rotation_sigma: float = 0.03
    # Constant visual sigmas: only used when use_constant_visual_sigmas is True
    # (i.e. the user explicitly overrode via --visual-range-sigma/--visual-bearing-sigma).
    visual_range_sigma: float = 0.40
    visual_bearing_sigma: float = 0.30
    use_constant_visual_sigmas: bool = False
    # Distance-dependent visual sigmas (default model): ArUco range/bearing noise
    # grows with distance to the tag, so the residual weight is scaled per-edge from
    # the measured range r: sigma_range = base + quad * r^2, sigma_bearing = base + lin * r.
    visual_range_sigma_base: float = 0.05
    visual_range_sigma_quad: float = 0.04
    visual_bearing_sigma_base: float = 0.05
    visual_bearing_sigma_lin: float = 0.03
    # LiDAR scan-matching (ICP) pose-pose constraints: reliable geometry, trusted
    # almost as much as odometry and used for loop closures.
    icp_translation_sigma: float = 0.05
    icp_rotation_sigma: float = 0.04
    # scipy least_squares stops at max_nfev *function evaluations*, not iterations.
    # max_iterations is kept for backward-compat and folded into max_nfev.
    max_iterations: int = 150
    max_nfev: int = 20000
    loss: str = 'cauchy'
    f_scale: float = 1.0


@dataclass(frozen=True)
class VariableLayout:
    pose_ids: list[int]
    landmark_ids: list[int]
    pose_index: dict[int, int]
    landmark_index: dict[int, int]
    size: int


class GraphSlamBackend:
    """Optimize a front-end pose-landmark graph."""

    def __init__(self, graph: dict[str, Any], config: BackendConfig) -> None:
        self.graph = graph
        self.config = config
        self.keyframes = sorted(graph.get('keyframes', []), key=lambda item: int(item['id']))
        self.landmarks = sorted(graph.get('landmarks', []), key=lambda item: int(item['id']))
        self.odom_edges = graph.get('odom_edges', [])
        self.icp_edges = graph.get('icp_edges', [])
        self.visual_edges = graph.get('visual_edges', [])
        if not self.keyframes:
            raise ValueError('Graph has no keyframes')
        self.edge_filter_stats = self._filter_edges()
        self.layout = self._make_layout()
        self.initial_state = self._pack_initial_state()

    def _filter_edges(self) -> dict[str, int]:
        """Drop edges that reference unknown nodes or carry broken measurements.

        The sparsity matrix assumes every remaining edge emits residuals, so edges
        whose endpoints are missing from the graph must be removed before layout,
        not silently skipped inside residuals() (which would create a row-count
        mismatch). Visual edges with an implausible range (< 0.1 m or > 3.5 m) come
        from broken PnP solutions and are dropped here too.
        """

        keyframe_ids = {int(keyframe['id']) for keyframe in self.keyframes}
        landmark_ids = {int(landmark['id']) for landmark in self.landmarks}

        def pose_pose_ok(edge: dict[str, Any]) -> bool:
            return int(edge['from_id']) in keyframe_ids and int(edge['to_id']) in keyframe_ids

        kept_odom = [edge for edge in self.odom_edges if pose_pose_ok(edge)]
        kept_icp = [edge for edge in self.icp_edges if pose_pose_ok(edge)]

        kept_visual: list[dict[str, Any]] = []
        dropped_visual_missing = 0
        dropped_visual_range = 0
        for edge in self.visual_edges:
            if int(edge['keyframe_id']) not in keyframe_ids or int(edge['landmark_id']) not in landmark_ids:
                dropped_visual_missing += 1
                continue
            measured_range = float(edge['range'])
            if measured_range < 0.1 or measured_range > 3.5:
                dropped_visual_range += 1
                continue
            kept_visual.append(edge)

        stats = {
            'dropped_odom_edges': len(self.odom_edges) - len(kept_odom),
            'dropped_icp_edges': len(self.icp_edges) - len(kept_icp),
            'dropped_visual_edges_missing_node': dropped_visual_missing,
            'dropped_visual_edges_bad_range': dropped_visual_range,
        }
        self.odom_edges = kept_odom
        self.icp_edges = kept_icp
        self.visual_edges = kept_visual
        if any(stats.values()):
            print('graph_slam_backend edge filter: %s' % json.dumps(stats, sort_keys=True), flush=True)
        return stats

    def optimize(self) -> dict[str, Any]:
        """Run nonlinear least squares and return an optimized graph snapshot."""

        initial_residual = self.residuals(self.initial_state)
        initial_cost = 0.5 * float(np.dot(initial_residual, initial_residual))

        if least_squares is None or self.layout.size == 0:
            optimized_state = self.initial_state.copy()
            iterations = 0
            success = least_squares is not None
            message = 'No variables to optimize' if self.layout.size == 0 else 'scipy.optimize.least_squares unavailable'
        else:
            result = least_squares(
                self.residuals,
                self.initial_state,
                max_nfev=self.config.max_nfev,
                x_scale='jac',
                loss=self.config.loss,
                f_scale=self.config.f_scale,
                jac_sparsity=self.jacobian_sparsity(),
            )
            optimized_state = np.asarray(result.x, dtype=float)
            iterations = int(result.nfev)
            success = bool(result.success)
            message = str(result.message)

        final_residual = self.residuals(optimized_state)
        final_cost = 0.5 * float(np.dot(final_residual, final_residual))
        cost_reduction = initial_cost - final_cost
        cost_reduction_ratio = cost_reduction / initial_cost if initial_cost > 0.0 else 0.0
        optimized_keyframes, optimized_landmarks = self._unpack_solution(optimized_state)

        return {
            'frame_id': self.graph.get('frame_id', 'map'),
            'solver': {
                'success': success,
                'message': message,
                'iterations': iterations,
                'initial_cost': initial_cost,
                'final_cost': final_cost,
                'cost_reduction': cost_reduction,
                'cost_reduction_ratio': cost_reduction_ratio,
                'usable_solution': bool(np.isfinite(final_cost) and final_cost < initial_cost),
                'residual_count': int(final_residual.size),
                'variable_count': int(self.layout.size),
                'max_nfev': int(self.config.max_nfev),
                'max_iterations': int(self.config.max_iterations),
                'visual_sigma_model': 'constant' if self.config.use_constant_visual_sigmas else 'distance_dependent',
                'edge_filter': self.edge_filter_stats,
                'config': self.config.__dict__,
            },
            'counts': {
                'keyframes': len(optimized_keyframes),
                'landmarks': len(optimized_landmarks),
                'odom_edges': len(self.odom_edges),
                'icp_edges': len(self.icp_edges),
                'visual_edges': len(self.visual_edges),
            },
            'initial_keyframes': self.keyframes,
            'initial_landmarks': self.landmarks,
            'optimized_keyframes': optimized_keyframes,
            'optimized_landmarks': optimized_landmarks,
            'odom_edges': self.odom_edges,
            'icp_edges': self.icp_edges,
            'visual_edges': self.visual_edges,
            'residual_summary': self._residual_summary(optimized_state),
        }

    def residuals(self, state: np.ndarray) -> np.ndarray:
        """Return weighted residual vector for the current state."""

        residuals: list[float] = []
        pose_by_id = self._pose_dict(state)
        landmark_by_id = self._landmark_dict(state)

        for edge in self.odom_edges:
            residuals.extend(self._pose_pose_residual(
                edge, pose_by_id, self.config.odom_translation_sigma, self.config.odom_rotation_sigma))

        for edge in self.icp_edges:
            residuals.extend(self._pose_pose_residual(
                edge, pose_by_id, self.config.icp_translation_sigma, self.config.icp_rotation_sigma))

        for edge in self.visual_edges:
            keyframe_id = int(edge['keyframe_id'])
            landmark_id = int(edge['landmark_id'])
            if keyframe_id not in pose_by_id or landmark_id not in landmark_by_id:
                continue
            pose_x, pose_y, pose_theta = pose_by_id[keyframe_id]
            landmark_x, landmark_y = landmark_by_id[landmark_id]
            dx = landmark_x - pose_x
            dy = landmark_y - pose_y
            pred_range = hypot(dx, dy)
            pred_bearing = normalize_angle(atan2(dy, dx) - pose_theta)
            measured_range = float(edge['range'])
            sigma_range, sigma_bearing = self._visual_sigmas(measured_range)
            residuals.extend([
                (pred_range - measured_range) / sigma_range,
                normalize_angle(pred_bearing - float(edge['bearing'])) / sigma_bearing,
            ])

        return np.asarray(residuals, dtype=float)

    def _visual_sigmas(self, measured_range: float) -> tuple[float, float]:
        """Range/bearing sigmas for a visual edge, constant or distance-dependent."""

        if self.config.use_constant_visual_sigmas:
            return self.config.visual_range_sigma, self.config.visual_bearing_sigma
        sigma_range = self.config.visual_range_sigma_base + self.config.visual_range_sigma_quad * measured_range * measured_range
        sigma_bearing = self.config.visual_bearing_sigma_base + self.config.visual_bearing_sigma_lin * measured_range
        return sigma_range, sigma_bearing

    def _pose_pose_residual(
        self,
        edge: dict[str, Any],
        pose_by_id: dict[int, tuple[float, float, float]],
        translation_sigma: float,
        rotation_sigma: float,
    ) -> list[float]:
        """Weighted residual for a relative pose-pose constraint (odom or ICP)."""

        from_id = int(edge['from_id'])
        to_id = int(edge['to_id'])
        if from_id not in pose_by_id or to_id not in pose_by_id:
            return []
        x_i, y_i, theta_i = pose_by_id[from_id]
        x_j, y_j, theta_j = pose_by_id[to_id]
        dx_global = x_j - x_i
        dy_global = y_j - y_i
        pred_dx = cos(theta_i) * dx_global + sin(theta_i) * dy_global
        pred_dy = -sin(theta_i) * dx_global + cos(theta_i) * dy_global
        pred_dtheta = normalize_angle(theta_j - theta_i)
        return [
            (pred_dx - float(edge.get('dx', 0.0))) / translation_sigma,
            (pred_dy - float(edge.get('dy', 0.0))) / translation_sigma,
            normalize_angle(pred_dtheta - float(edge.get('dtheta', 0.0))) / rotation_sigma,
        ]

    def jacobian_sparsity(self) -> Any:
        """Return residual-variable sparsity for scalable finite differences."""

        if lil_matrix is None or self.layout.size == 0:
            return None

        residual_rows = (len(self.odom_edges) + len(self.icp_edges)) * 3 + len(self.visual_edges) * 2
        sparsity = lil_matrix((residual_rows, self.layout.size), dtype=int)
        row = 0

        for edge in (*self.odom_edges, *self.icp_edges):
            from_id = int(edge['from_id'])
            to_id = int(edge['to_id'])
            columns: list[int] = []
            if from_id in self.layout.pose_index:
                start = self.layout.pose_index[from_id]
                columns.extend([start, start + 1, start + 2])
            if to_id in self.layout.pose_index:
                start = self.layout.pose_index[to_id]
                columns.extend([start, start + 1, start + 2])
            for residual_row in range(row, row + 3):
                for column in columns:
                    sparsity[residual_row, column] = 1
            row += 3

        for edge in self.visual_edges:
            keyframe_id = int(edge['keyframe_id'])
            landmark_id = int(edge['landmark_id'])
            columns = []
            if keyframe_id in self.layout.pose_index:
                start = self.layout.pose_index[keyframe_id]
                columns.extend([start, start + 1, start + 2])
            if landmark_id in self.layout.landmark_index:
                start = self.layout.landmark_index[landmark_id]
                columns.extend([start, start + 1])
            for residual_row in range(row, row + 2):
                for column in columns:
                    sparsity[residual_row, column] = 1
            row += 2

        return sparsity

    def _make_layout(self) -> VariableLayout:
        pose_ids = [int(keyframe['id']) for keyframe in self.keyframes]
        optimized_pose_ids = pose_ids[1:]  # keep first pose fixed as gauge anchor
        landmark_ids = [int(landmark['id']) for landmark in self.landmarks]
        pose_index: dict[int, int] = {}
        cursor = 0
        for pose_id in optimized_pose_ids:
            pose_index[pose_id] = cursor
            cursor += 3
        landmark_index: dict[int, int] = {}
        for landmark_id in landmark_ids:
            landmark_index[landmark_id] = cursor
            cursor += 2
        return VariableLayout(
            pose_ids=pose_ids,
            landmark_ids=landmark_ids,
            pose_index=pose_index,
            landmark_index=landmark_index,
            size=cursor,
        )

    def _pack_initial_state(self) -> np.ndarray:
        state = np.zeros(self.layout.size, dtype=float)
        for keyframe in self.keyframes[1:]:
            idx = self.layout.pose_index[int(keyframe['id'])]
            state[idx:idx + 3] = [float(keyframe['x']), float(keyframe['y']), float(keyframe['theta'])]
        for landmark in self.landmarks:
            idx = self.layout.landmark_index[int(landmark['id'])]
            state[idx:idx + 2] = [float(landmark['x']), float(landmark['y'])]
        return state

    def _pose_dict(self, state: np.ndarray) -> dict[int, tuple[float, float, float]]:
        poses: dict[int, tuple[float, float, float]] = {}
        first = self.keyframes[0]
        poses[int(first['id'])] = (float(first['x']), float(first['y']), float(first['theta']))
        for keyframe in self.keyframes[1:]:
            pose_id = int(keyframe['id'])
            idx = self.layout.pose_index[pose_id]
            poses[pose_id] = (float(state[idx]), float(state[idx + 1]), normalize_angle(float(state[idx + 2])))
        return poses

    def _landmark_dict(self, state: np.ndarray) -> dict[int, tuple[float, float]]:
        landmarks: dict[int, tuple[float, float]] = {}
        for landmark in self.landmarks:
            landmark_id = int(landmark['id'])
            idx = self.layout.landmark_index[landmark_id]
            landmarks[landmark_id] = (float(state[idx]), float(state[idx + 1]))
        return landmarks

    def _unpack_solution(self, state: np.ndarray) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
        pose_by_id = self._pose_dict(state)
        landmark_by_id = self._landmark_dict(state)

        optimized_keyframes: list[dict[str, Any]] = []
        initial_keyframes_by_id = {int(keyframe['id']): keyframe for keyframe in self.keyframes}
        for pose_id in self.layout.pose_ids:
            initial = initial_keyframes_by_id[pose_id]
            x, y, theta = pose_by_id[pose_id]
            optimized_keyframes.append({
                'id': pose_id,
                'x': x,
                'y': y,
                'theta': theta,
                'stamp': float(initial.get('stamp', 0.0)),
                'frame_id': initial.get('frame_id', self.graph.get('frame_id', 'map')),
            })

        initial_landmarks_by_id = {int(landmark['id']): landmark for landmark in self.landmarks}
        optimized_landmarks: list[dict[str, Any]] = []
        for landmark_id in self.layout.landmark_ids:
            initial = initial_landmarks_by_id[landmark_id]
            x, y = landmark_by_id[landmark_id]
            optimized_landmarks.append({
                'id': landmark_id,
                'x': x,
                'y': y,
                'observations': int(initial.get('observations', 0)),
            })
        return optimized_keyframes, optimized_landmarks

    def _residual_summary(self, state: np.ndarray) -> dict[str, Any]:
        pose_by_id = self._pose_dict(state)
        landmark_by_id = self._landmark_dict(state)
        odom_norms: list[float] = []
        icp_norms: list[float] = []
        visual_range_errors: list[float] = []
        visual_bearing_errors: list[float] = []

        def edge_abs_error(edge: dict[str, Any]) -> float | None:
            from_id = int(edge['from_id'])
            to_id = int(edge['to_id'])
            if from_id not in pose_by_id or to_id not in pose_by_id:
                return None
            x_i, y_i, theta_i = pose_by_id[from_id]
            x_j, y_j, theta_j = pose_by_id[to_id]
            dx_global = x_j - x_i
            dy_global = y_j - y_i
            pred_dx = cos(theta_i) * dx_global + sin(theta_i) * dy_global
            pred_dy = -sin(theta_i) * dx_global + cos(theta_i) * dy_global
            pred_dtheta = normalize_angle(theta_j - theta_i)
            return hypot(pred_dx - float(edge.get('dx', 0.0)), pred_dy - float(edge.get('dy', 0.0))) + abs(
                normalize_angle(pred_dtheta - float(edge.get('dtheta', 0.0))))

        for edge in self.odom_edges:
            value = edge_abs_error(edge)
            if value is not None:
                odom_norms.append(value)

        for edge in self.icp_edges:
            value = edge_abs_error(edge)
            if value is not None:
                icp_norms.append(value)

        for edge in self.visual_edges:
            keyframe_id = int(edge['keyframe_id'])
            landmark_id = int(edge['landmark_id'])
            if keyframe_id not in pose_by_id or landmark_id not in landmark_by_id:
                continue
            pose_x, pose_y, pose_theta = pose_by_id[keyframe_id]
            landmark_x, landmark_y = landmark_by_id[landmark_id]
            dx = landmark_x - pose_x
            dy = landmark_y - pose_y
            visual_range_errors.append(hypot(dx, dy) - float(edge['range']))
            visual_bearing_errors.append(normalize_angle(atan2(dy, dx) - pose_theta - float(edge['bearing'])))

        return {
            'odom_abs_error_mean': _mean_abs(odom_norms),
            'icp_abs_error_mean': _mean_abs(icp_norms),
            'visual_range_error_mean_m': _mean_abs(visual_range_errors),
            'visual_range_error_max_m': _max_abs(visual_range_errors),
            'visual_bearing_error_mean_rad': _mean_abs(visual_bearing_errors),
            'visual_bearing_error_max_rad': _max_abs(visual_bearing_errors),
        }


def _mean_abs(values: list[float]) -> float | None:
    if not values:
        return None
    return float(np.mean(np.abs(np.asarray(values, dtype=float))))


def _max_abs(values: list[float]) -> float | None:
    if not values:
        return None
    return float(np.max(np.abs(np.asarray(values, dtype=float))))


def write_csvs(optimized_graph: dict[str, Any], output_dir: Path) -> None:
    """Write trajectory and landmark CSV exports."""

    output_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / 'optimized_trajectory.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=['id', 'stamp', 'x', 'y', 'theta', 'frame_id'])
        writer.writeheader()
        for row in optimized_graph['optimized_keyframes']:
            writer.writerow(row)

    with (output_dir / 'optimized_landmarks.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=['id', 'x', 'y', 'observations'])
        writer.writeheader()
        for row in optimized_graph['optimized_landmarks']:
            writer.writerow(row)


def load_config(args: argparse.Namespace) -> BackendConfig:
    # If the user explicitly passes --visual-range-sigma/--visual-bearing-sigma we
    # honour the old constant-sigma behaviour; otherwise use the distance-dependent
    # model. max_nfev counts function evaluations, so fold --max-iterations into it.
    use_constant = args.visual_range_sigma is not None or args.visual_bearing_sigma is not None
    visual_range_sigma = args.visual_range_sigma if args.visual_range_sigma is not None else 0.40
    visual_bearing_sigma = args.visual_bearing_sigma if args.visual_bearing_sigma is not None else 0.30
    return BackendConfig(
        odom_translation_sigma=args.odom_translation_sigma,
        odom_rotation_sigma=args.odom_rotation_sigma,
        visual_range_sigma=visual_range_sigma,
        visual_bearing_sigma=visual_bearing_sigma,
        use_constant_visual_sigmas=use_constant,
        visual_range_sigma_base=args.visual_range_sigma_base,
        visual_range_sigma_quad=args.visual_range_sigma_quad,
        visual_bearing_sigma_base=args.visual_bearing_sigma_base,
        visual_bearing_sigma_lin=args.visual_bearing_sigma_lin,
        icp_translation_sigma=args.icp_translation_sigma,
        icp_rotation_sigma=args.icp_rotation_sigma,
        max_iterations=args.max_iterations,
        max_nfev=max(int(args.max_nfev), int(args.max_iterations)),
        loss=args.loss,
        f_scale=args.f_scale,
    )


def optimize_graph_file(input_path: Path, output_path: Path, config: BackendConfig) -> dict[str, Any]:
    graph = json.loads(input_path.read_text(encoding='utf-8'))
    backend = GraphSlamBackend(graph, config)
    optimized = backend.optimize()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(optimized, indent=2, sort_keys=True), encoding='utf-8')
    write_csvs(optimized, output_path.parent)
    return optimized


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Optimize a tpf_slam front-end graph snapshot offline.')
    parser.add_argument('--input', default='log/slam_frontend_graph.json', help='Input front-end graph JSON path.')
    parser.add_argument('--output', default='log/slam_optimized_graph.json', help='Output optimized graph JSON path.')
    parser.add_argument('--odom-translation-sigma', type=float, default=0.03)
    parser.add_argument('--odom-rotation-sigma', type=float, default=0.03)
    parser.add_argument('--visual-range-sigma', type=float, default=None,
                        help='Override with a constant range sigma (disables the distance-dependent model).')
    parser.add_argument('--visual-bearing-sigma', type=float, default=None,
                        help='Override with a constant bearing sigma (disables the distance-dependent model).')
    parser.add_argument('--visual-range-sigma-base', type=float, default=0.05)
    parser.add_argument('--visual-range-sigma-quad', type=float, default=0.04)
    parser.add_argument('--visual-bearing-sigma-base', type=float, default=0.05)
    parser.add_argument('--visual-bearing-sigma-lin', type=float, default=0.03)
    parser.add_argument('--icp-translation-sigma', type=float, default=0.05)
    parser.add_argument('--icp-rotation-sigma', type=float, default=0.04)
    parser.add_argument('--max-iterations', type=int, default=150,
                        help='Backward-compat; folded into --max-nfev via max(max_nfev, max_iterations).')
    parser.add_argument('--max-nfev', type=int, default=20000,
                        help='Max scipy least_squares function evaluations.')
    parser.add_argument('--loss', default='cauchy')
    parser.add_argument('--f-scale', type=float, default=1.0)
    return parser


def main(argv: list[str] | None = None) -> None:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    optimized = optimize_graph_file(Path(args.input), Path(args.output), load_config(args))
    print(json.dumps({
        'output': args.output,
        'counts': optimized['counts'],
        'solver': optimized['solver'],
        'residual_summary': optimized['residual_summary'],
    }, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
