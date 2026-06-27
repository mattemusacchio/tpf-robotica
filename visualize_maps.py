"""Visualize Part A (laberinto) and Part B (casa) occupancy maps side by side."""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from PIL import Image
from pathlib import Path

BASE = Path(__file__).parent

MAPS = {
    'Parte A — Laberinto\n(Graph SLAM, TurtleBot4 real)': {
        'pgm': BASE / 'log/maps/laberinto_map.pgm',
        'resolution': 0.05,
        'origin': (-13.748633305978476, -9.09304539721079),
    },
    'Parte B — Casa\n(Gazebo, TurtleBot3)': {
        'pgm': BASE / 'src/tpf_navigation/maps/casa_map.pgm',
        'resolution': 0.05,
        'origin': (-3.5, -3.5),
    },
}

OCCUPIED_THRESH = 0.65
FREE_THRESH     = 0.196
NEGATE          = False


def pgm_to_display(path: Path):
    img = Image.open(path)
    arr = np.array(img, dtype=np.float32)
    # ROS convention: pixel value → occupancy probability
    if NEGATE:
        occ = arr / 255.0
    else:
        occ = 1.0 - arr / 255.0  # bright = free, dark = occupied

    rgb = np.ones((*arr.shape, 3), dtype=np.float32)  # white default (unknown)
    free_mask     = occ < FREE_THRESH
    occupied_mask = occ >= OCCUPIED_THRESH
    unknown_mask  = ~free_mask & ~occupied_mask

    rgb[free_mask]     = [0.96, 0.96, 0.96]  # near-white for free
    rgb[occupied_mask] = [0.15, 0.15, 0.20]  # dark blue-grey for walls
    rgb[unknown_mask]  = [0.60, 0.62, 0.65]  # medium grey for unknown

    return rgb, arr.shape


fig, axes = plt.subplots(1, 2, figsize=(16, 8),
                         facecolor='#1a1a2e')
fig.suptitle('Mapas de Ocupación — TPF Robótica',
             fontsize=18, color='white', fontweight='bold', y=0.98)

for ax, (title, meta) in zip(axes, MAPS.items()):
    path = meta['pgm']
    res  = meta['resolution']
    ox, oy = meta['origin']

    rgb, shape = pgm_to_display(path)
    h, w = shape

    # World-space extents
    x_min = ox
    x_max = ox + w * res
    y_min = oy
    y_max = oy + h * res

    ax.imshow(rgb, extent=[x_min, x_max, y_min, y_max],
              origin='lower', interpolation='nearest')

    # Grid
    ax.set_facecolor('#1a1a2e')
    ax.grid(True, color='#444466', linewidth=0.4, linestyle='--', alpha=0.6)
    ax.tick_params(colors='#aaaacc', labelsize=8)
    for spine in ax.spines.values():
        spine.set_edgecolor('#444466')

    # Labels
    ax.set_xlabel('x  [m]', color='#aaaacc', fontsize=9)
    ax.set_ylabel('y  [m]', color='#aaaacc', fontsize=9)
    ax.set_title(title, color='white', fontsize=11, fontweight='bold', pad=10)

    # Metadata box
    info = (
        f'Resolución: {res*100:.0f} cm/celda\n'
        f'Dimensiones: {w}×{h} celdas\n'
        f'Área: {w*res:.1f} m × {h*res:.1f} m\n'
        f'Origen: ({ox:.2f}, {oy:.2f}) m'
    )
    ax.text(0.02, 0.02, info, transform=ax.transAxes,
            fontsize=7.5, color='#ccccee',
            verticalalignment='bottom',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='#0d0d1a',
                      edgecolor='#444466', alpha=0.85))

    # Scale bar — 1 m
    bar_x0 = x_max - 1.5
    bar_y0 = y_min + 0.3
    ax.plot([bar_x0, bar_x0 + 1.0], [bar_y0, bar_y0],
            color='#ffcc44', linewidth=3, solid_capstyle='butt')
    ax.text(bar_x0 + 0.5, bar_y0 + (y_max - y_min) * 0.02, '1 m',
            color='#ffcc44', fontsize=8, ha='center', va='bottom')

# Legend
legend_patches = [
    mpatches.Patch(color=[0.15, 0.15, 0.20], label='Ocupado (pared/obstáculo)'),
    mpatches.Patch(color=[0.96, 0.96, 0.96], label='Libre'),
    mpatches.Patch(color=[0.60, 0.62, 0.65], label='Desconocido'),
]
fig.legend(handles=legend_patches, loc='lower center', ncol=3,
           fontsize=9, framealpha=0.85,
           facecolor='#0d0d1a', edgecolor='#444466',
           labelcolor='white', bbox_to_anchor=(0.5, 0.01))

plt.tight_layout(rect=[0, 0.06, 1, 0.97])

out = BASE / 'maps_visualization.png'
plt.savefig(out, dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
print(f'Saved: {out}')
plt.show()
