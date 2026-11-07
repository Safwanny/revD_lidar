#!/usr/bin/env python3
"""Score a saved SLAM map against the reference grid from world_grid.

The SLAM map frame starts at the robot's spawn pose, so the map is placed in
the world with that pose only (no fitting: drift counts against the map).

- accuracy: distance from each SLAM occupied cell to the nearest reference
  occupied cell (mean, p95, share within TOLERANCE);
- coverage: share of the reference surfaces facing reachable free space that
  have a SLAM occupied cell within TOLERANCE.

    map_score <slam.yaml> <reference.yaml> --spawn x y yaw [--plot out.png]
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree
import yaml

TOLERANCE = 0.10


def load_map(yaml_path):
    """(occupied, free) boolean grids with row 0 = lowest y, plus metadata."""
    meta = yaml.safe_load(Path(yaml_path).read_text(encoding='utf-8'))
    raw = (Path(yaml_path).parent / meta['image']).read_bytes()
    tokens, position = [], 0
    while len(tokens) < 4:
        line_end = raw.index(b'\n', position)
        line = raw[position:line_end].split(b'#')[0]
        tokens += line.split()
        position = line_end + 1
    assert tokens[0] == b'P5', 'binary PGM only'
    width, height = int(tokens[1]), int(tokens[2])
    image = np.frombuffer(raw[position:position + width * height],
                          dtype=np.uint8).reshape(height, width)[::-1]
    p = image / 255.0 if meta.get('negate', 0) else (255 - image) / 255.0
    return p > meta['occupied_thresh'], p < meta['free_thresh'], meta


def cell_centres(mask, meta, pose=(0.0, 0.0, 0.0)):
    """World (x, y) of the True cells, the map placed at pose."""
    rows, cols = np.nonzero(mask)
    r = meta['resolution']
    x = meta['origin'][0] + (cols + 0.5) * r
    y = meta['origin'][1] + (rows + 0.5) * r
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.column_stack((pose[0] + c * x - s * y, pose[1] + s * x + c * y))


def visible_surfaces(occupied, free, meta, start):
    """Occupied reference cells next to free space reachable from start."""
    labels, _ = ndimage.label(free)
    r = meta['resolution']
    col = int((start[0] - meta['origin'][0]) / r)
    row = int((start[1] - meta['origin'][1]) / r)
    assert labels[row, col], 'spawn pose is not in free space'
    reachable = labels == labels[row, col]
    return occupied & ndimage.binary_dilation(reachable)


def score(slam_yaml, reference_yaml, spawn):
    slam_occupied, _, slam_meta = load_map(slam_yaml)
    ref_occupied, ref_free, ref_meta = load_map(reference_yaml)
    slam_points = cell_centres(slam_occupied, slam_meta, spawn)
    ref_points = cell_centres(ref_occupied, ref_meta)
    surface_points = cell_centres(
        visible_surfaces(ref_occupied, ref_free, ref_meta, spawn), ref_meta)
    error, _ = cKDTree(ref_points).query(slam_points)
    gap, _ = cKDTree(slam_points).query(surface_points)
    return {
        'slam_occupied_cells': len(slam_points),
        'accuracy_mean_m': float(error.mean()),
        'accuracy_p95_m': float(np.percentile(error, 95)),
        'accuracy_within_tol': float((error <= TOLERANCE).mean()),
        'coverage_within_tol': float((gap <= TOLERANCE).mean()),
        'tolerance_m': TOLERANCE,
    }, slam_points, ref_points


def plot(slam_points, ref_points, title, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(figsize=(10, 8))
    axes.scatter(*ref_points.T, s=1, c='0.75', label='reference')
    axes.scatter(*slam_points.T, s=1, c='tab:red', label='SLAM')
    axes.set_aspect('equal')
    axes.set_xlabel('x (m)')
    axes.set_ylabel('y (m)')
    axes.set_title(title)
    axes.legend(loc='upper right', markerscale=8)
    figure.savefig(out, dpi=120, bbox_inches='tight')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('slam_yaml')
    parser.add_argument('reference_yaml')
    parser.add_argument('--spawn', type=float, nargs=3, required=True,
                        metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--plot')
    args = parser.parse_args()
    result, slam_points, ref_points = score(
        args.slam_yaml, args.reference_yaml, args.spawn)
    print(json.dumps(result, indent=2))
    if args.plot:
        plot(slam_points, ref_points, Path(args.slam_yaml).resolve().parent.name,
             args.plot)


if __name__ == '__main__':
    main()
