"""
Map improvement and visualization for TPF Robotica.

laberinto_map.pgm was previously over-eroded (walls already 1-2px thin;
cross-erosion deleted them).  This script restores connectivity via dilation
then removes isolated single-pixel noise.

casa_map.pgm is a blank placeholder - must be generated via SLAM in Gazebo.
To regenerate the laberinto map properly, run in a ROS2 terminal:

  source install/setup.bash
  ros2 run tpf_slam occupancy_grid_builder \\
    --bag data/rosbags/laberinto \\
    --optimized-graph log/laberinto_second_pass_graph.json \\
    --output-dir log/maps --map-name laberinto_map \\
    --resolution 0.05 --beam-stride 1 --scan-stride 1 \\
    --min-hits-for-occupied 2 --min-occupied-component-cells 1 \\
    --inflate-radius-m 0.0 --min-scan-travel-m 0.08
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from scipy.ndimage import binary_dilation, binary_erosion
from scipy.ndimage import label as ndlabel

BASE = Path(__file__).parent

OCC_VAL     = 0
FREE_VAL    = 254
UNKNOWN_VAL = 205

CROSS = np.array([[0, 1, 0],
                  [1, 1, 1],
                  [0, 1, 0]], dtype=bool)
FULL3 = np.ones((3, 3), dtype=bool)


# ─── restore over-eroded laberinto map ────────────────────────────────────────

def restore_laberinto_map(pgm_path: Path) -> None:
    img = np.array(Image.open(pgm_path), dtype=np.uint8)
    occupied  = img < 50
    free_orig = img > 220

    # Dilate thinned walls back to ~2-cell width
    restored = binary_dilation(occupied, structure=CROSS, iterations=1)

    # Remove isolated single-pixel noise
    labeled, n = ndlabel(restored, structure=FULL3)
    clean = np.zeros_like(restored)
    for comp_id in range(1, n + 1):
        if int((labeled == comp_id).sum()) >= 2:
            clean[labeled == comp_id] = True

    result = np.full_like(img, UNKNOWN_VAL)
    result[free_orig] = FREE_VAL
    result[clean]     = OCC_VAL
    # pixels freed by previous over-erosion stay free
    result[occupied & ~clean] = FREE_VAL

    orig  = int(occupied.sum())
    after = int(clean.sum())
    print(f'[laberinto] restore: {orig} -> {after} occupied cells')
    Image.fromarray(result).save(pgm_path)
    print(f'[laberinto] saved -> {pgm_path}')


# ─── blank casa placeholder ───────────────────────────────────────────────────

def create_blank_casa_map(pgm_path: Path, width: int = 140, height: int = 140) -> None:
    arr = np.full((height, width), UNKNOWN_VAL, dtype=np.uint8)
    Image.fromarray(arr).save(pgm_path)
    print(f'[casa] blank placeholder saved -> {pgm_path}')


# ─── visualization ────────────────────────────────────────────────────────────

MAPS = {
    'Parte A — Laberinto\n(Graph SLAM + ICP 2da pasada, TurtleBot4 real)': {
        'pgm'       : BASE / 'log/maps/laberinto_map.pgm',
        'resolution': 0.05,
        'origin'    : (-13.64630651473999, -9.160494327545166),
        'graph'     : BASE / 'log/laberinto_second_pass_graph.json',
        'pending'   : False,
    },
    'Parte B — Casa\n(Graph SLAM + ICP, TurtleBot3 Gazebo)': {
        'pgm'       : BASE / 'log/maps/casa_map.pgm',
        'resolution': 0.05,
        'origin'    : (-9.483926483976642, -9.182551582306258),
        'graph'     : BASE / 'log/casa_optimized_graph.json',
        'pending'   : False,
    },
}


def pgm_to_rgb(path: Path):
    img = np.array(Image.open(path), dtype=np.float32)
    occ = 1.0 - img / 255.0
    rgb = np.full((*img.shape, 3), [0.55, 0.57, 0.62], dtype=np.float32)
    rgb[occ < 0.196]  = [0.96, 0.96, 0.96]
    rgb[occ >= 0.65]  = [0.10, 0.12, 0.20]
    return rgb, img.shape


def load_trajectory(graph_path: Path | None):
    if graph_path is None or not graph_path.exists():
        return [], []
    g = json.loads(graph_path.read_text())
    kf = g.get('optimized_keyframes') or g.get('keyframes') or []
    return [float(k['x']) for k in kf], [float(k['y']) for k in kf]


def render_visualization() -> None:
    fig, axes = plt.subplots(1, 2, figsize=(17, 8), facecolor='#10102a')
    fig.suptitle('Mapas de Ocupacion — TPF Robotica',
                 fontsize=19, color='white', fontweight='bold', y=0.98)

    for ax, (title, meta) in zip(axes, MAPS.items()):
        res = meta['resolution']
        ox, oy = meta['origin']
        rgb, (h, w) = pgm_to_rgb(meta['pgm'])
        x_min, x_max = ox, ox + w * res
        y_min, y_max = oy, oy + h * res

        # PGM is stored with flipud (row 0 = y_max). origin='upper' renders correctly.
        ax.imshow(rgb, extent=[x_min, x_max, y_min, y_max],
                  origin='upper', interpolation='nearest')

        traj_x, traj_y = load_trajectory(meta['graph'])
        if traj_x:
            ax.plot(traj_x, traj_y, color='#ff6644', linewidth=0.8,
                    alpha=0.8, label='Trayectoria SLAM', zorder=3)
            ax.plot(traj_x[0],  traj_y[0],  'o', color='#44ff88',
                    markersize=6, zorder=4, label=f'Inicio  ({traj_x[0]:.1f}, {traj_y[0]:.1f})')
            ax.plot(traj_x[-1], traj_y[-1], 's', color='#ffcc00',
                    markersize=6, zorder=4, label=f'Fin  ({traj_x[-1]:.1f}, {traj_y[-1]:.1f})')
            ax.legend(fontsize=7.5, loc='upper right',
                      facecolor='#1a1a3a', edgecolor='#444466',
                      labelcolor='white', framealpha=0.90)
            # Zoom in on trajectory extent (world coords) + padding
            pad = 1.5
            ax.set_xlim(min(traj_x) - pad, max(traj_x) + pad)
            ax.set_ylim(min(traj_y) - pad, max(traj_y) + pad)

        if meta['pending']:
            ax.text(0.5, 0.5, 'Mapa pendiente\nEjecutar SLAM en Gazebo\n(ver workflow en README)',
                    transform=ax.transAxes, fontsize=12, color='#ffaa44',
                    ha='center', va='center', fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.7', facecolor='#0a0a1e',
                              edgecolor='#ffaa44', alpha=0.90))

        ax.set_facecolor('#10102a')
        ax.grid(True, color='#252545', linewidth=0.35, linestyle='--', alpha=0.6)
        ax.tick_params(colors='#9999bb', labelsize=8)
        for spine in ax.spines.values():
            spine.set_edgecolor('#252545')
        ax.set_xlabel('x  [m]', color='#9999bb', fontsize=9)
        ax.set_ylabel('y  [m]', color='#9999bb', fontsize=9)
        ax.set_title(title, color='white', fontsize=11, fontweight='bold', pad=10)

        kf_count = len(traj_x) if traj_x else 0
        info_lines = [
            f'Res: {res*100:.0f} cm/celda',
            f'Mapa: {w}x{h} celdas  ({w*res:.1f} x {h*res:.1f} m)',
        ]
        if kf_count:
            info_lines.append(f'Keyframes SLAM: {kf_count}')
        ax.text(0.02, 0.02, '\n'.join(info_lines),
                transform=ax.transAxes, fontsize=7.5, color='#ccccee',
                verticalalignment='bottom',
                bbox=dict(boxstyle='round,pad=0.4', facecolor='#0a0a1e',
                          edgecolor='#444466', alpha=0.88))

        # 1m scale bar using current axis limits
        xlim = ax.get_xlim()
        ylim = ax.get_ylim()
        if (xlim[1] - xlim[0]) > 2.0:
            bar_x0 = xlim[1] - 1.8
            bar_y0 = ylim[0] + 0.35
            ax.annotate('', xy=(bar_x0 + 1.0, bar_y0), xytext=(bar_x0, bar_y0),
                        arrowprops=dict(arrowstyle='<->', color='#ffcc44', lw=2.0))
            ax.text(bar_x0 + 0.5, bar_y0 + (ylim[1] - ylim[0]) * 0.022, '1 m',
                    color='#ffcc44', fontsize=8, ha='center', va='bottom')

    patches = [
        mpatches.Patch(color=[0.10, 0.12, 0.20], label='Ocupado (pared / obstaculo)'),
        mpatches.Patch(color=[0.96, 0.96, 0.96], label='Libre'),
        mpatches.Patch(color=[0.55, 0.57, 0.62], label='Desconocido'),
    ]
    fig.legend(handles=patches, loc='lower center', ncol=3,
               fontsize=9, framealpha=0.90,
               facecolor='#0a0a1e', edgecolor='#444466',
               labelcolor='white', bbox_to_anchor=(0.5, 0.01))

    plt.tight_layout(rect=[0, 0.06, 1, 0.97])
    out = BASE / 'maps_visualization.png'
    plt.savefig(out, dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
    print(f'Visualization saved -> {out}')
    plt.show()


# ─── main ─────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print('=== Rendering visualization ===')
    render_visualization()
