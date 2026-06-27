"""Generate a 2-D occupancy map for custom_casa from its SDF wall geometry.

Rasterizes every box-shaped wall in the 'testing' model into a PGM grid at
0.05 m/cell resolution.  Outputs map.pgm + map.yaml in the standard nav2 map
format so nav2_map_server and our occupancy-based nodes can load it directly.

Usage:
    ros2 run tpf_navigation generate_casa_map --output-dir path/to/maps
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
from PIL import Image


# All box walls from the 'testing' model in casa.world.
# Each entry: (center_x, center_y, yaw_rad, box_x, box_y)
_WALLS = [
    (-2.95,   0.00,  -math.pi / 2, 6.00, 0.10),   # Wall_20  west
    ( 0.00,  -2.95,   0.00,        6.00, 0.10),   # Wall_21  south
    ( 2.95,   0.00,   math.pi / 2, 6.00, 0.10),   # Wall_22  east
    ( 0.00,   2.95,   math.pi,     6.00, 0.10),   # Wall_23  north
    (-0.50,  -1.50,   0.00,        5.00, 0.10),   # Wall_25  main divider
    (-0.95,  -2.75,   math.pi / 2, 0.50, 0.10),   # Wall_29  south col-L
    ( 0.00,  -1.725, -math.pi / 2, 0.55, 0.10),   # Wall_31  south mid
    ( 0.95,  -2.75,   math.pi / 2, 0.50, 0.10),   # Wall_33  south col-R
    ( 1.95,  -1.725, -math.pi / 2, 0.55, 0.10),   # Wall_35  south-R stub
    (-1.95,  -1.725, -math.pi / 2, 0.55, 0.10),   # Wall_38  south-L stub
    (-0.95,   0.225,  math.pi / 2, 3.55, 0.10),   # Wall_40  inner vertical
    (-2.625,  0.50,   0.00,        0.75, 0.10),   # Wall_42  inner horiz-L
    (-1.275,  0.50,   math.pi,     0.75, 0.10),   # Wall_44  inner horiz-R
]

RESOLUTION = 0.05          # m/cell
PADDING    = 0.30          # extra space around world bounds (m)
WORLD_MIN  = -3.20         # world x/y lower bound (m)
WORLD_MAX  =  3.20         # world x/y upper bound (m)
FREE_GREY  = 254           # PGM value for free space (nav2 convention)
OCC_GREY   = 0             # PGM value for occupied
UNKNOWN    = 205           # not used here but standard for unknown


def _cell_in_obb(cx: float, cy: float, wx: float, wy: float,
                 half_x: float, half_y: float, yaw: float) -> bool:
    """Return True if world point (wx, wy) falls inside the oriented box."""
    dx = wx - cx
    dy = wy - cy
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx = c * dx - s * dy
    ly = s * dx + c * dy
    return abs(lx) <= half_x and abs(ly) <= half_y


def build_grid(resolution: float = RESOLUTION) -> tuple[np.ndarray, float, float]:
    lo = WORLD_MIN - PADDING
    hi = WORLD_MAX + PADDING
    ncols = int(math.ceil((hi - lo) / resolution))
    nrows = int(math.ceil((hi - lo) / resolution))
    grid = np.full((nrows, ncols), FREE_GREY, dtype=np.uint8)

    for (cx, cy, yaw, bx, by) in _WALLS:
        half_x = bx / 2.0 + resolution       # inflate by 1 cell so thin walls fill
        half_y = by / 2.0 + resolution
        # Bounding box of the OBB for fast rejection
        corners_x = [
            cx + math.cos(yaw) * dx + math.cos(yaw + math.pi / 2) * dy
            for dx in (-half_x, half_x)
            for dy in (-half_y, half_y)
        ]
        corners_y = [
            cy + math.sin(yaw) * dx + math.sin(yaw + math.pi / 2) * dy
            for dx in (-half_x, half_x)
            for dy in (-half_y, half_y)
        ]
        min_col = max(0, int((min(corners_x) - lo) / resolution) - 1)
        max_col = min(ncols - 1, int((max(corners_x) - lo) / resolution) + 1)
        min_row = max(0, int((min(corners_y) - lo) / resolution) - 1)
        max_row = min(nrows - 1, int((max(corners_y) - lo) / resolution) + 1)

        for row in range(min_row, max_row + 1):
            wy = lo + (row + 0.5) * resolution
            for col in range(min_col, max_col + 1):
                wx = lo + (col + 0.5) * resolution
                if _cell_in_obb(cx, cy, wx, wy, half_x, half_y, yaw):
                    grid[row, col] = OCC_GREY

    return grid, lo, lo


def main() -> None:
    parser = argparse.ArgumentParser(description='Generate custom_casa occupancy map.')
    parser.add_argument('--output-dir', default='maps', help='Directory to write map files')
    parser.add_argument('--resolution', type=float, default=RESOLUTION)
    args = parser.parse_args()

    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)

    grid, origin_x, origin_y = build_grid(args.resolution)

    # PGM: row 0 = south (y_min), need to flip for image convention (row 0 = top = y_max)
    img_array = np.flipud(grid)
    img = Image.fromarray(img_array, mode='L')
    pgm_path = out / 'casa_map.pgm'
    img.save(str(pgm_path))

    yaml_text = (
        f'image: casa_map.pgm\n'
        f'resolution: {args.resolution}\n'
        f'origin: [{origin_x:.4f}, {origin_y:.4f}, 0.0]\n'
        f'negate: 0\n'
        f'occupied_thresh: 0.65\n'
        f'free_thresh: 0.196\n'
    )
    (out / 'casa_map.yaml').write_text(yaml_text)

    nrows, ncols = grid.shape
    n_occ = int(np.sum(grid == OCC_GREY))
    print(f'Map: {ncols}x{nrows} cells @ {args.resolution}m/cell')
    print(f'Origin: ({origin_x:.3f}, {origin_y:.3f})')
    print(f'Occupied cells: {n_occ}')
    print(f'Saved: {pgm_path}')
    print(f'Saved: {out / "casa_map.yaml"}')


if __name__ == '__main__':
    main()
