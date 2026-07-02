"""Second-pass ICP refinement using optimized poses as initialization.

After graph optimization, poses are much better than raw odometry.  Re-running
ICP with those poses as the starting guess lets us use a tighter correspondence
distance and find more loop closures, producing sharper constraints for a second
optimization round.
"""

from __future__ import annotations

import argparse
import json
import sqlite3
from math import atan2, cos, hypot, sin
from pathlib import Path
from typing import Any

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from .icp import icp_match, scan_to_points


def _normalize(a: float) -> float:
    return atan2(sin(a), cos(a))


def _relative_pose(opt_poses: dict, from_id: int, to_id: int) -> tuple[float, float, float]:
    a = opt_poses[from_id]
    b = opt_poses[to_id]
    gdx = b['x'] - a['x']
    gdy = b['y'] - a['y']
    dx = cos(a['theta']) * gdx + sin(a['theta']) * gdy
    dy = -sin(a['theta']) * gdx + cos(a['theta']) * gdy
    return dx, dy, _normalize(b['theta'] - a['theta'])


def _read_scans_for_keyframes(
    bag_path: Path,
    keyframe_stamps: dict[int, float],
    *,
    scan_topic: str,
    laser_x: float,
    laser_y: float,
    laser_yaw: float,
    icp_max_range_m: float,
) -> dict[int, np.ndarray]:
    """Deserialize only the scan closest in time to each keyframe — 737 reads, not 10797."""
    db3_files = sorted(bag_path.glob('*.db3'))
    scan_type = get_message('sensor_msgs/msg/LaserScan')

    # Pass 1: collect all scan (timestamp_ns, rowid, file_index) without deserializing.
    all_rows: list[tuple[int, int, int]] = []
    connections: list[sqlite3.Connection] = []
    for fi, db3 in enumerate(db3_files):
        conn = sqlite3.connect(str(db3))
        connections.append(conn)
        topics = {name: tid for tid, name, *_ in conn.execute(
            'SELECT id, name, type, serialization_format, offered_qos_profiles FROM topics'
        )}
        if scan_topic not in topics:
            continue
        tid = topics[scan_topic]
        for ts, rowid in conn.execute(
            'SELECT timestamp, id FROM messages WHERE topic_id=? ORDER BY timestamp', (tid,)
        ):
            all_rows.append((int(ts), int(rowid), fi))

    if not all_rows:
        for c in connections:
            c.close()
        return {}

    all_ts = np.array([r[0] for r in all_rows], dtype=np.int64)

    # Pass 2: for each keyframe, binary-search the closest scan row.
    needed: dict[tuple[int, int], list[int]] = {}  # (fi, rowid) -> [kf_ids]
    for kf_id, stamp_sec in keyframe_stamps.items():
        stamp_ns = int(stamp_sec * 1e9)
        idx = int(np.searchsorted(all_ts, stamp_ns))
        best_i, best_diff = -1, int(5e8)  # max 0.5 s
        for i in (idx - 1, idx, idx + 1):
            if 0 <= i < len(all_rows):
                diff = abs(int(all_ts[i]) - stamp_ns)
                if diff < best_diff:
                    best_diff, best_i = diff, i
        if best_i >= 0:
            _, rowid, fi = all_rows[best_i]
            needed.setdefault((fi, rowid), []).append(kf_id)

    # Pass 3: deserialize only the needed rows.
    result: dict[int, np.ndarray] = {}
    for fi, conn in enumerate(connections):
        rows_for_file = {rowid: kf_ids for (f, rowid), kf_ids in needed.items() if f == fi}
        if not rows_for_file:
            continue
        placeholders = ','.join('?' * len(rows_for_file))
        for rowid, data in conn.execute(
            f'SELECT id, data FROM messages WHERE id IN ({placeholders})',
            list(rows_for_file.keys()),
        ):
            msg = deserialize_message(bytes(data), scan_type)
            pts = scan_to_points(
                np.asarray(msg.ranges, dtype=np.float64),
                float(msg.angle_min),
                float(msg.angle_increment),
                float(msg.range_min),
                float(msg.range_max),
                icp_max_range_m,
                laser_x=laser_x,
                laser_y=laser_y,
                laser_yaw=laser_yaw,
                beam_stride=1,
            )
            if pts.shape[0] >= 25:
                for kf_id in rows_for_file[int(rowid)]:
                    result[kf_id] = pts

    for c in connections:
        c.close()
    return result


def _run_icp_edges(
    frontend_graph: dict[str, Any],
    opt_poses: dict[int, dict],
    keyframe_scans: dict[int, np.ndarray],
    *,
    max_correspondence_m: float,
    loop_radius_m: float,
    min_index_gap: int,
    max_per_keyframe: int,
    min_fitness: float,
    max_mean_error_m: float,
    max_disagreement: float,
) -> tuple[list[dict], dict]:
    kfs = sorted(frontend_graph['keyframes'], key=lambda k: int(k['id']))
    kf_ids = [kf['id'] for kf in kfs]
    stats: dict[str, int] = {
        'sequential': 0, 'loop_candidates': 0,
        'loop_closures': 0, 'loop_rejected': 0, 'no_scan': 0,
    }
    edges: list[dict] = []

    def try_icp(from_id: int, to_id: int, etype: str) -> dict | None:
        src = keyframe_scans.get(to_id)
        tgt = keyframe_scans.get(from_id)
        if src is None or tgt is None:
            stats['no_scan'] += 1
            return None
        guess = _relative_pose(opt_poses, from_id, to_id)
        r = icp_match(src, tgt, init=guess, max_iterations=60,
                      trim_ratio=0.85, max_correspondence_m=max_correspondence_m)
        if not r.converged or r.fitness < min_fitness:
            return None
        return {
            'from_id': from_id, 'to_id': to_id,
            'dx': r.dx, 'dy': r.dy, 'dtheta': r.dtheta,
            'distance': hypot(r.dx, r.dy),
            'fitness': r.fitness, 'mean_error': r.mean_error,
            'type': etype,
        }

    # Sequential refinement between consecutive keyframes.
    print(f'  Sequential ICP: {len(kf_ids) - 1} pairs ...', flush=True)
    for i in range(1, len(kf_ids)):
        e = try_icp(kf_ids[i - 1], kf_ids[i], 'sequential')
        if e is not None and e['mean_error'] <= max_mean_error_m * 2.0:
            edges.append(e)
            stats['sequential'] += 1

    # Loop closures via spatial proximity in optimized pose space.
    positions = np.array([[opt_poses[k]['x'], opt_poses[k]['y']] for k in kf_ids])
    print(f'  Loop closure ICP (radius={loop_radius_m}m, gap≥{min_index_gap}) ...', flush=True)
    for j, kf_to in enumerate(kf_ids):
        if j < min_index_gap:
            continue
        deltas = positions[:j] - positions[j]
        distances = np.hypot(deltas[:, 0], deltas[:, 1])
        candidates = sorted(
            [i for i in range(j) if j - i >= min_index_gap and distances[i] <= loop_radius_m],
            key=lambda i: distances[i],
        )
        accepted = 0
        for i in candidates:
            if accepted >= max_per_keyframe:
                break
            stats['loop_candidates'] += 1
            e = try_icp(kf_ids[i], kf_to, 'loop_closure')
            if e is None or e['mean_error'] > max_mean_error_m:
                continue
            gdx, gdy, gdth = _relative_pose(opt_poses, kf_ids[i], kf_to)
            disagreement = hypot(e['dx'] - gdx, e['dy'] - gdy) + abs(_normalize(e['dtheta'] - gdth))
            if disagreement > max_disagreement:
                stats['loop_rejected'] += 1
                continue
            edges.append(e)
            stats['loop_closures'] += 1
            accepted += 1

    return edges, stats


def main() -> None:
    parser = argparse.ArgumentParser(description='Second-pass ICP using optimized poses.')
    parser.add_argument('--frontend-graph', required=True,
                        help='Frontend graph JSON (original edges + keyframe stamps)')
    parser.add_argument('--optimized-graph', required=True,
                        help='Optimized graph JSON (better pose estimates)')
    parser.add_argument('--bag', required=True, help='rosbag2 directory')
    parser.add_argument('--output', required=True, help='Output graph JSON path')
    parser.add_argument('--max-correspondence-m', type=float, default=0.25,
                        help='ICP correspondence distance threshold (default 0.25)')
    parser.add_argument('--loop-radius-m', type=float, default=2.0,
                        help='Loop closure search radius using optimized positions (default 2.0)')
    parser.add_argument('--min-index-gap', type=int, default=25,
                        help='Min keyframe index gap for loop closures (default 25)')
    parser.add_argument('--max-per-keyframe', type=int, default=3,
                        help='Max loop closure edges per keyframe (default 3)')
    parser.add_argument('--max-mean-error-m', type=float, default=0.08,
                        help='ICP mean error acceptance threshold (default 0.08)')
    parser.add_argument('--max-disagreement', type=float, default=0.8,
                        help='Max ICP vs optimized-pose disagreement in m+rad (default 0.8)')
    parser.add_argument('--scan-topic', default='/tb4_0/scan', help='LiDAR topic name in the bag.')
    parser.add_argument('--laser-x-m', type=float, default=-0.04, help='Laser x offset in base frame.')
    parser.add_argument('--laser-y-m', type=float, default=0.0, help='Laser y offset in base frame.')
    parser.add_argument('--laser-yaw-rad', type=float, default=1.5707963267948966, help='Laser yaw offset in base frame.')
    parser.add_argument('--icp-max-range-m', type=float, default=3.5, help='Max LiDAR range used for ICP point clouds.')
    args = parser.parse_args()

    bag_path = Path(args.bag)
    output_path = Path(args.output)

    print('Loading graphs ...')
    with open(args.frontend_graph) as f:
        frontend_graph = json.load(f)
    with open(args.optimized_graph) as f:
        optimized_graph = json.load(f)

    opt_poses: dict[int, dict] = {kf['id']: kf for kf in optimized_graph['optimized_keyframes']}
    keyframe_stamps = {kf['id']: kf['stamp'] for kf in frontend_graph['keyframes']}

    print(f'Reading scans for {len(keyframe_stamps)} keyframes from bag ...')
    keyframe_scans = _read_scans_for_keyframes(
        bag_path, keyframe_stamps,
        scan_topic=args.scan_topic,
        laser_x=args.laser_x_m,
        laser_y=args.laser_y_m,
        laser_yaw=args.laser_yaw_rad,
        icp_max_range_m=args.icp_max_range_m,
    )
    print(f'  Matched scans for {len(keyframe_scans)}/{len(keyframe_stamps)} keyframes')

    print('Running second-pass ICP ...')
    icp_edges, stats = _run_icp_edges(
        frontend_graph, opt_poses, keyframe_scans,
        max_correspondence_m=args.max_correspondence_m,
        loop_radius_m=args.loop_radius_m,
        min_index_gap=args.min_index_gap,
        max_per_keyframe=args.max_per_keyframe,
        min_fitness=0.55,
        max_mean_error_m=args.max_mean_error_m,
        max_disagreement=args.max_disagreement,
    )

    print(json.dumps({'second_pass_stats': stats, 'icp_edges': len(icp_edges),
                      'was': len(frontend_graph.get('icp_edges', []))}, indent=2))

    new_graph = dict(frontend_graph)
    new_graph['icp_edges'] = icp_edges
    new_graph['counts'] = {
        'keyframes': len(frontend_graph['keyframes']),
        'landmarks': len(frontend_graph['landmarks']),
        'odom_edges': len(frontend_graph['odom_edges']),
        'visual_edges': len(frontend_graph['visual_edges']),
        'icp_edges': len(icp_edges),
    }
    new_graph['second_pass_stats'] = stats

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, 'w') as f:
        json.dump(new_graph, f, indent=2)
    print(f'Saved → {output_path}')
