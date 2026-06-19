"""Generate a Parte A validation report for ArUco Graph SLAM artifacts."""

from __future__ import annotations

import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any


def _load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(path)
    return json.loads(path.read_text(encoding='utf-8'))


def _solver_metrics(optimized_graph: dict[str, Any]) -> dict[str, Any]:
    solver = optimized_graph.get('solver', {})
    initial_cost = solver.get('initial_cost')
    final_cost = solver.get('final_cost')
    reduction = None
    reduction_ratio = None
    if isinstance(initial_cost, (int, float)) and isinstance(final_cost, (int, float)):
        reduction = float(initial_cost) - float(final_cost)
        if float(initial_cost) > 0.0:
            reduction_ratio = reduction / float(initial_cost)
    return {
        'success': solver.get('success'),
        'usable_solution': solver.get('usable_solution'),
        'message': solver.get('message'),
        'iterations': solver.get('iterations', solver.get('nfev')),
        'initial_cost': initial_cost,
        'final_cost': final_cost,
        'cost_reduction': reduction,
        'cost_reduction_ratio': reduction_ratio,
    }


def _loop_closure_metrics(graph: dict[str, Any]) -> dict[str, Any]:
    keyframe_by_id = {int(row['id']): row for row in graph.get('keyframes', [])}
    observations: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for edge in graph.get('visual_edges', []):
        landmark_id = int(edge['landmark_id'])
        keyframe_id = int(edge.get('from_id', edge.get('keyframe_id')))
        keyframe = keyframe_by_id.get(keyframe_id, {})
        observations[landmark_id].append({
            'keyframe_id': keyframe_id,
            'stamp': keyframe.get('stamp'),
            'range': edge.get('range'),
            'bearing': edge.get('bearing'),
        })

    reobserved = {}
    loop_candidates = {}
    for landmark_id, rows in observations.items():
        keyframes = sorted({int(row['keyframe_id']) for row in rows})
        if len(keyframes) < 2:
            continue
        stamps = [float(row['stamp']) for row in rows if row.get('stamp') is not None]
        detail = {
            'observations': len(rows),
            'distinct_keyframes': len(keyframes),
            'first_keyframe': keyframes[0],
            'last_keyframe': keyframes[-1],
            'keyframe_span': keyframes[-1] - keyframes[0],
            'time_span_sec': (max(stamps) - min(stamps)) if stamps else None,
        }
        reobserved[str(landmark_id)] = detail
        if detail['keyframe_span'] >= 10:
            loop_candidates[str(landmark_id)] = detail

    top = Counter({str(k): len(v) for k, v in observations.items()}).most_common(10)
    return {
        'observed_landmarks': len(observations),
        'reobserved_landmarks': len(reobserved),
        'loop_closure_candidate_landmarks': len(loop_candidates),
        'top_observed_landmarks': [
            {'id': landmark_id, 'observations': count} for landmark_id, count in top
        ],
        'reobserved_detail': reobserved,
        'loop_closure_candidate_detail': loop_candidates,
    }


def build_report(
    frontend_graph_path: Path,
    optimized_graph_path: Path,
    map_summary_path: Path | None,
) -> dict[str, Any]:
    frontend = _load_json(frontend_graph_path)
    optimized = _load_json(optimized_graph_path)
    map_summary = _load_json(map_summary_path) if map_summary_path and map_summary_path.exists() else {}

    keyframes = optimized.get('optimized_keyframes') or optimized.get('keyframes') or []
    landmarks = optimized.get('optimized_landmarks') or optimized.get('landmarks') or []
    visual_edges = optimized.get('visual_edges', frontend.get('visual_edges', []))
    odom_edges = optimized.get('odom_edges', frontend.get('odom_edges', []))
    solver = _solver_metrics(optimized)
    loop_closure = _loop_closure_metrics(frontend)
    map_stats = map_summary.get('stats', {})

    return {
        'inputs': {
            'frontend_graph': str(frontend_graph_path),
            'optimized_graph': str(optimized_graph_path),
            'map_summary': str(map_summary_path) if map_summary_path else None,
        },
        'outputs': {
            'occupancy_yaml': 'log/maps/laberinto_map.yaml',
            'occupancy_pgm': 'log/maps/laberinto_map.pgm',
            'occupancy_png': 'log/maps/laberinto_map.png',
            'optimized_landmarks': 'log/maps/optimized_landmarks.json',
            'optimized_trajectory': 'log/maps/optimized_trajectory.csv',
        },
        'graph': {
            'keyframes': len(keyframes),
            'landmarks': len(landmarks),
            'odom_edges': len(odom_edges),
            'visual_edges': len(visual_edges),
            'frontend_counts': frontend.get('counts', {}),
        },
        'solver': solver,
        'loop_closure': loop_closure,
        'map': {
            'width': map_summary.get('width'),
            'height': map_summary.get('height'),
            'resolution': map_summary.get('resolution'),
            'origin': map_summary.get('origin'),
            'processed_scans': map_stats.get('processed_scans'),
            'scan_messages': map_stats.get('scan_messages'),
            'valid_rays': map_stats.get('valid_rays'),
            'laser_transform': map_summary.get('laser_transform'),
        },
        'acceptance': {
            'has_keyframes': len(keyframes) > 0,
            'has_landmarks': len(landmarks) > 0,
            'has_visual_edges': len(visual_edges) > 0,
            'has_loop_closure_evidence': loop_closure['reobserved_landmarks'] > 0,
            'reduced_cost': (solver['cost_reduction'] or 0.0) > 0.0,
            'has_map': bool(map_summary.get('width') and map_summary.get('height')),
        },
    }


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Generate a Parte A Graph SLAM quality report.')
    parser.add_argument('--frontend-graph', default='log/laberinto_frontend_graph.json')
    parser.add_argument('--optimized-graph', default='log/laberinto_optimized_graph.json')
    parser.add_argument('--map-summary', default='log/maps/laberinto_map_summary.json')
    parser.add_argument('--output', default='log/part_a/part_a_summary.json')
    return parser


def main(argv: list[str] | None = None) -> None:
    args = build_arg_parser().parse_args(argv)
    report = build_report(Path(args.frontend_graph), Path(args.optimized_graph), Path(args.map_summary))
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True), encoding='utf-8')
    print(json.dumps({'output': str(output), 'acceptance': report['acceptance']}, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
