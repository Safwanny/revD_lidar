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

"""Localization error of a map -> base_footprint TUM file against truth.

No alignment: the map frame is the spawn pose (the map was built from it),
so truth is moved into that frame and compared as is. Converged = the first
sample within 0.1 m and 5 deg; error statistics are taken from there on.
Lost = share of converged time with more than 0.3 m error.
"""

import argparse
import json
import math

import numpy as np

CONVERGED_M = 0.10
CONVERGED_RAD = math.radians(5.0)
LOST_M = 0.30
MAX_DT = 0.05


def load_tum(path):
    """(t, x, y, yaw) rows."""
    rows = np.loadtxt(path, ndmin=2)
    qx, qy, qz, qw = rows[:, 4], rows[:, 5], rows[:, 6], rows[:, 7]
    yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return np.column_stack((rows[:, 0], rows[:, 1], rows[:, 2], yaw))


def score(truth, estimate, spawn=(0.0, 0.0, 0.0)):
    sx, sy, syaw = spawn
    c, s = math.cos(syaw), math.sin(syaw)
    dx, dy = truth[:, 1] - sx, truth[:, 2] - sy
    truth = np.column_stack((truth[:, 0], c * dx + s * dy, -s * dx + c * dy,
                             truth[:, 3] - syaw))
    index = np.clip(np.searchsorted(truth[:, 0], estimate[:, 0]), 1,
                    len(truth) - 1)
    index -= estimate[:, 0] - truth[index - 1, 0] < truth[index, 0] - estimate[:, 0]
    keep = np.abs(truth[index, 0] - estimate[:, 0]) <= MAX_DT
    t, e, g = estimate[keep, 0], estimate[keep], truth[index[keep]]
    xy = np.hypot(e[:, 1] - g[:, 1], e[:, 2] - g[:, 2])
    yaw = np.abs((e[:, 3] - g[:, 3] + math.pi) % (2 * math.pi) - math.pi)
    hits = np.nonzero((xy < CONVERGED_M) & (yaw < CONVERGED_RAD))[0]
    result = {'samples': int(len(t)), 'converged': bool(len(hits))}
    if not len(hits):
        return result
    first = hits[0]
    xy, yaw, t = xy[first:], yaw[first:], t[first:]
    result.update({
        'converge_s': round(float(t[0] - estimate[0, 0]), 2),
        'rmse_m': round(float(np.sqrt(np.mean(xy ** 2))), 4),
        'p95_m': round(float(np.percentile(xy, 95)), 4),
        'max_m': round(float(xy.max()), 4),
        'yaw_rmse_deg': round(math.degrees(np.sqrt(np.mean(yaw ** 2))), 3),
        'lost_share': round(float(np.mean(xy > LOST_M)), 4),
    })
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('truth')
    parser.add_argument('estimate')
    parser.add_argument('--spawn', nargs=3, type=float, default=(0, 0, 0))
    args = parser.parse_args()
    print(json.dumps(score(load_tum(args.truth), load_tum(args.estimate),
                           args.spawn), indent=2))


if __name__ == '__main__':
    main()
