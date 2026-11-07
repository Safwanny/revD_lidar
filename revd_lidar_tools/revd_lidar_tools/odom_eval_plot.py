#!/usr/bin/env python3
"""Plot an odom_eval output directory: paths and heading error per run.

Usage: odom_eval_plot <output_dir> [<output_dir> ...]
Writes <output_dir>/odom_eval.png. Estimates are aligned to the truth by
their starting pose only, as odom_eval scores them.
"""

import math
from pathlib import Path
import sys

import matplotlib
import numpy as np

from revd_lidar_tools.odom_eval import RUNS, relative, wrap

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402  after the backend is chosen

STYLE = {'truth': ('k', 'truth'), 'ekf': ('tab:blue', 'EKF'),
         'wheel': ('tab:orange', 'wheel only')}


def load(path):
    """Time (s from start) and (x, y, yaw) rows of a TUM file."""
    data = np.loadtxt(path, ndmin=2)
    qx, qy, qz, qw = data[:, 4:8].T
    yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return data[:, 0], np.column_stack([data[:, 1], data[:, 2], yaw])


def to_truth_frame(truth_origin, poses):
    """Move a trajectory so its first pose sits on the truth's first pose."""
    x0, y0, t0 = truth_origin
    c, s = math.cos(t0), math.sin(t0)
    out = []
    for pose in poses:
        x, y, yaw = relative(poses[0], pose)
        out.append((x0 + c * x - s * y, y0 + s * x + c * y, wrap(t0 + yaw)))
    return np.array(out)


def plot(directory):
    """Write odom_eval.png for one output directory."""
    directory = Path(directory)
    names = [name for name, _ in RUNS]
    figure, axes = plt.subplots(2, len(names), figsize=(5 * len(names), 9))
    for column, name in enumerate(names):
        times, truth = load(directory / f'{name}_truth.tum')
        path_axis, error_axis = axes[0, column], axes[1, column]
        path_axis.plot(truth[:, 0], truth[:, 1], STYLE['truth'][0],
                       label='truth')
        for source in ('ekf', 'wheel'):
            stamps, poses = load(directory / f'{name}_{source}.tum')
            poses = to_truth_frame(truth[0], poses)
            colour, label = STYLE[source]
            path_axis.plot(poses[:, 0], poses[:, 1], color=colour,
                           label=label)
            # Heading error against the truth at the estimate's stamps.
            truth_yaw = np.interp(stamps, times, np.unwrap(truth[:, 2]))
            error = np.degrees([wrap(e - t) for e, t in
                                zip(np.unwrap(poses[:, 2]), truth_yaw)])
            error_axis.plot(stamps - times[0], error, color=colour,
                            label=label)
        path_axis.set_title(name)
        path_axis.set_aspect('equal', adjustable='datalim')
        path_axis.set_xlabel('x (m)')
        path_axis.set_ylabel('y (m)')
        path_axis.legend()
        error_axis.set_xlabel('time (s)')
        error_axis.set_ylabel('heading error (deg)')
        error_axis.axhline(0, color='0.7', linewidth=0.8)
        error_axis.grid(alpha=0.3)
    figure.suptitle(directory.name)
    figure.tight_layout()
    figure.savefig(directory / 'odom_eval.png', dpi=110)
    plt.close(figure)


def main():
    """Plot every directory named on the command line."""
    for directory in sys.argv[1:]:
        plot(directory)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
