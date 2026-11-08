#!/usr/bin/env python3
"""Render a Gazebo world's reference occupancy grid at the scan plane.

Slices every static box and cylinder collision at the height of laser_link
(read from the robot description, so it follows properties.xacro) and writes
a Nav2 map (<out>.pgm + <out>.yaml) in the world frame: what a perfect 2D
LiDAR map of the world would hold. map_score compares SLAM maps against it.

    world_grid <world.sdf> <out> [--resolution 0.05]
"""

import argparse
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np

RESOLUTION = 0.05
MARGIN = 0.5
OCCUPIED, FREE = 0, 254


def scan_height(urdf_xml):
    """laser_link height above base_footprint, from a URDF string."""
    joints = {j.find('child').get('link'): j
              for j in ET.fromstring(urdf_xml).findall('joint')}
    link, z = 'laser_link', 0.0
    while link in joints:
        origin = joints[link].find('origin')
        roll, pitch, _ = (float(v) for v in origin.get('rpy', '0 0 0').split())
        assert abs(roll) < 1e-9 and abs(pitch) < 1e-9, 'tilted chain'
        z += float(origin.get('xyz').split()[2])
        link = joints[link].find('parent').get('link')
    assert link == 'base_footprint', link
    return z


def robot_scan_height():
    xacro_file = (Path(get_package_share_directory('revd_lidar_description'))
                  / 'urdf' / 'robot.urdf.xacro')
    urdf = subprocess.run(['xacro', str(xacro_file)], check=True,
                          capture_output=True, text=True).stdout
    return scan_height(urdf)


def pose_of(element):
    """(x, y, z, yaw) of an element's <pose>; roll and pitch must be zero."""
    text = element.findtext('pose')
    x, y, z, roll, pitch, yaw = (
        [float(v) for v in text.split()] if text else [0.0] * 6)
    assert abs(roll) < 1e-9 and abs(pitch) < 1e-9, 'only yaw is supported'
    return x, y, z, yaw


def compose(a, b):
    c, s = math.cos(a[3]), math.sin(a[3])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1],
            a[2] + b[2], a[3] + b[3])


def shapes(sdf_path, height):
    """('box', x, y, yaw, sx, sy) / ('cylinder', x, y, r) cut by the plane."""
    world = ET.parse(sdf_path).getroot().find('world')
    out = []
    for model in world.findall('model'):
        model_pose = pose_of(model)
        for link in model.findall('link'):
            link_pose = compose(model_pose, pose_of(link))
            for collision in link.findall('collision'):
                x, y, z, yaw = compose(link_pose, pose_of(collision))
                geometry = collision.find('geometry')
                if geometry.find('plane') is not None:
                    continue
                if (size := geometry.findtext('box/size')) is not None:
                    sx, sy, sz = (float(v) for v in size.split())
                    if abs(height - z) <= sz / 2:
                        out.append(('box', x, y, yaw, sx, sy))
                elif geometry.find('cylinder') is not None:
                    radius = float(geometry.findtext('cylinder/radius'))
                    length = float(geometry.findtext('cylinder/length'))
                    if abs(height - z) <= length / 2:
                        out.append(('cylinder', x, y, radius))
                else:
                    raise ValueError(
                        f'{model.get("name")}: unsupported geometry')
    return out


def render(cut, resolution=RESOLUTION):
    """Occupancy (True = occupied, row 0 = lowest y) and the grid origin."""
    xs, ys = [], []
    for shape in cut:
        if shape[0] == 'box':
            c, s = abs(math.cos(shape[3])), abs(math.sin(shape[3]))
            rx = (c * shape[4] + s * shape[5]) / 2
            ry = (s * shape[4] + c * shape[5]) / 2
        else:
            rx = ry = shape[3]
        xs += [shape[1] - rx, shape[1] + rx]
        ys += [shape[2] - ry, shape[2] + ry]
    x0, y0 = min(xs) - MARGIN, min(ys) - MARGIN
    width = math.ceil((max(xs) + MARGIN - x0) / resolution)
    height = math.ceil((max(ys) + MARGIN - y0) / resolution)
    cx = x0 + (np.arange(width) + 0.5) * resolution
    cy = y0 + (np.arange(height) + 0.5) * resolution
    px, py = np.meshgrid(cx, cy)
    grid = np.zeros((height, width), dtype=bool)
    # A cell is occupied if the shape covers its centre, grown by half a cell
    # so 0.1 m walls and 24 mm legs never fall between centres.
    half = resolution / 2
    for shape in cut:
        if shape[0] == 'box':
            _, x, y, yaw, sx, sy = shape
            c, s = math.cos(yaw), math.sin(yaw)
            u = c * (px - x) + s * (py - y)
            v = -s * (px - x) + c * (py - y)
            grid |= (np.abs(u) <= sx / 2 + half) & (np.abs(v) <= sy / 2 + half)
        else:
            _, x, y, radius = shape
            grid |= np.hypot(px - x, py - y) <= radius + half
    return grid, (x0, y0)


def save_map(grid, origin, out, resolution=RESOLUTION):
    """Nav2 map_server files: <out>.pgm (row 0 = top) and <out>.yaml."""
    out = Path(out)
    image = np.where(grid, OCCUPIED, FREE).astype(np.uint8)[::-1]
    header = f'P5\n{image.shape[1]} {image.shape[0]}\n255\n'.encode()
    out.with_suffix('.pgm').write_bytes(header + image.tobytes())
    out.with_suffix('.yaml').write_text(
        f'image: {out.with_suffix(".pgm").name}\n'
        f'mode: trinary\nresolution: {resolution}\n'
        f'origin: [{origin[0]:.4f}, {origin[1]:.4f}, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n',
        encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('world', help='world SDF')
    parser.add_argument('out', help='output path without extension')
    parser.add_argument('--resolution', type=float, default=RESOLUTION,
                        help='cell size in m (nav_eval scores gaps at 0.01)')
    args = parser.parse_args()
    height = robot_scan_height()
    grid, origin = render(shapes(args.world, height), args.resolution)
    save_map(grid, origin, args.out, args.resolution)
    print(f'{args.out}: {grid.shape[1]} x {grid.shape[0]} cells at '
          f'{args.resolution} m, sliced at z = {height:.4f} m')


if __name__ == '__main__':
    main()
