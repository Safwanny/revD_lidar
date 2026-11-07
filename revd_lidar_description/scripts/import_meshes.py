#!/usr/bin/env python3

# Copyright 2026 Safwan
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Import the Rev_D_W11.25 meshes at URDF size.

W11.25 has no per-link export. 02_cad/parts holds one copy of each part,
centred, in build orientation; placing them would re-implement the CAD
transforms. ros2/mecanum_01_full.stl is the placed assembly instead: metres,
REP-103, origin on the ground under the centre. It is split by body:

  wheel_xx.stl  the wheel (hub outward, with its M3 x 35 and nuts) and the
                drive line on the axle (axle, coupler, motor shaft, grub
                screws, plus the bearings, whose outer races really sit still
                in the wishbone); in the wheel frame, centred on the roller
                crowns, the contact line the kinematics use
  body.stl      everything else, in the ground frame

Each body is simplified on its own scale. The CAD folder is only read.

  import_meshes.py <Rev_D_W11.25 dir> <output dir>

Fails loudly if the wheels are not where the CAD says.
"""

from pathlib import Path
import sys

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

# build_mecanum_01_fusion.py WX, AZ; clearance_check.txt roller envelope
# 59.92; track 186.5. The hub's inner face (69.5) and the coupler's 7 mm
# radius are read off the assembly mesh.
WHEEL_X = 0.105
WHEEL_Z = 0.02996
WHEEL_R = 0.02996
WHEEL_Y = 0.09325
HUB_Y_MIN = 0.0695
DRIVE_LINE_R = 0.008
TOL = 0.3e-3
WHEELS = {'wheel_fl': (1, 1), 'wheel_fr': (1, -1),
          'wheel_rl': (-1, 1), 'wheel_rr': (-1, -1)}


def load_stl(path):
    data = Path(path).read_bytes()
    n = int(np.frombuffer(data[80:84], '<u4')[0])
    rec = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    return np.frombuffer(data[84:84 + 50 * n], rec)['v'].astype(np.float64)


def save_stl(path, tris):
    tris = tris.astype('<f4')
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(tris), np.dtype(
        [('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    rec['n'], rec['v'] = n, tris
    with open(path, 'wb') as f:
        f.write(b'Mecanum_01 Rev D W11.25 import'.ljust(80, b' '))
        f.write(np.uint32(len(tris)).tobytes())
        f.write(rec.tobytes())


def components(tris):
    q = np.round(tris.reshape(-1, 3) * 1e6).astype(np.int64)
    _, vid = np.unique(q, axis=0, return_inverse=True)
    t = len(tris)
    rows = np.repeat(np.arange(t), 3)
    cols = t + vid.ravel()
    size = t + vid.max() + 1
    graph = coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(size, size))
    _, label = connected_components(graph, directed=False)
    label = label[:t]
    return [tris[label == k] for k in np.unique(label)]


def simplify(tris, cell):
    """Vertex clustering: merge vertices sharing a grid cell, drop slivers."""
    pts = tris.reshape(-1, 3)
    key = np.floor(pts / cell).astype(np.int64)
    _, cid = np.unique(key, axis=0, return_inverse=True)
    cid = cid.ravel()
    mean = np.zeros((cid.max() + 1, 3))
    np.add.at(mean, cid, pts)
    mean /= np.bincount(cid)[:, None]
    f = cid.reshape(-1, 3)
    keep = (f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])
    f = f[keep]
    _, first = np.unique(np.sort(f, axis=1), axis=0, return_index=True)
    return mean[f[np.sort(first)]]


def save_simplified(path, parts, offset):
    """Simplify each part on its own scale, so small screws keep their shape."""
    out = []
    for part in parts:
        pts = part.reshape(-1, 3)
        size = (pts.max(0) - pts.min(0)).max()
        out.append(simplify(part, float(np.clip(size / 15, 0.3e-3, 1.5e-3))))
    mesh = np.concatenate(out) - offset
    save_stl(path, mesh)
    return len(mesh)


def wheel_of(part):
    """Name of the wheel this body turns with, or None."""
    pts = part.reshape(-1, 3)
    sx, sy = np.sign(pts[:, :2].mean(0))
    r = np.hypot(pts[:, 0] - sx * WHEEL_X, pts[:, 2] - WHEEL_Z).max()
    outboard = (sy * pts[:, 1]).min() > HUB_Y_MIN - TOL
    if r > WHEEL_R + TOL or not (outboard or r < DRIVE_LINE_R):
        return None
    return next(k for k, v in WHEELS.items() if v == (sx, sy))


def main(cad_dir, out_dir):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    tris = load_stl(Path(cad_dir) / 'ros2' / 'mecanum_01_full.stl')
    groups = {name: [] for name in [*WHEELS, 'body']}
    for part in components(tris):
        groups[wheel_of(part) or 'body'].append(part)

    for name, parts in groups.items():
        offset = np.zeros(3)
        if name != 'body':
            sx, sy = WHEELS[name]
            pts = np.concatenate(parts).reshape(-1, 3)
            # Roller crowns: the points that touch the floor.
            rim = pts[np.hypot(pts[:, 0] - sx * WHEEL_X, pts[:, 2] - WHEEL_Z)
                      > WHEEL_R - 0.5e-3]
            offset = np.array([sx * WHEEL_X,
                               (rim[:, 1].min() + rim[:, 1].max()) / 2,
                               WHEEL_Z])
            assert abs(offset[1] - sy * WHEEL_Y) < TOL, f'{name} y {offset}'
            r = np.hypot(pts[:, 0] - offset[0], pts[:, 2] - offset[2]).max()
            assert abs(r - WHEEL_R) < TOL, f'{name} radius {r}'
        n = save_simplified(out / f'{name}.stl', parts, offset)
        print(f'{name}: {len(parts)} bodies, {sum(map(len, parts))} -> {n} '
              f'tris, frame {np.round(offset * 1e3, 2)} mm')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
