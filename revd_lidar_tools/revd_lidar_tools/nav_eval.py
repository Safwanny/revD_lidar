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

"""Send Nav2 a seeded chain of goals and score every run against truth.

Goals are drawn from the world's reference grid (world_grid): free cells
reachable from the spawn, at least CLEARANCE from any wall, each at least
MIN_HOP from the last, with a random heading. Each run starts where the
last one ended. Per goal: Nav2's status, time, distance from the goal on
/ground_truth/odom (evaluation only), and the smallest gap between the hull
and an occupied reference cell (<= 0 counts as a collision; good to half
a cell, so score on a 0.01 m reference from world_grid --resolution 0.01,
as the governor stops about 0.03 m short). /cmd_vel_nav messages per
second while a goal is active give the controller loop rate.

Writes nav_eval.json and the ground-truth track (truth.tum, world frame)
to output_dir; exits 0 when at least 90 % succeed.
"""

import json
import math
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from revd_lidar_tools.map_score import load_map
from scipy import ndimage

CLEARANCE = 0.4
MIN_HOP = 2.0
REACHED = 0.25
TIMEOUT = 240.0
PASS_RATE = 0.9
HALF = (0.15, 0.11)


def sample_goals(occupied, meta, spawn, count, seed):
    """Draw count (x, y, yaw) world goals, seeded, chained from the spawn."""
    r, origin = meta['resolution'], meta['origin']
    distance = ndimage.distance_transform_edt(~occupied) * r
    labels, _ = ndimage.label(~occupied)
    row = int((spawn[1] - origin[1]) / r)
    col = int((spawn[0] - origin[0]) / r)
    rows, cols = np.nonzero((labels == labels[row, col])
                            & (distance >= CLEARANCE))
    points = np.column_stack((origin[0] + (cols + 0.5) * r,
                              origin[1] + (rows + 0.5) * r))
    rng = np.random.default_rng(seed)
    goals, last = [], np.array(spawn[:2])
    while len(goals) < count:
        far = points[np.hypot(*(points - last).T) >= MIN_HOP]
        last = far[rng.integers(len(far))]
        goals.append((float(last[0]), float(last[1]),
                      float(rng.uniform(-math.pi, math.pi))))
    return goals


def hull_gap(pose, distance, meta):
    """Smallest wall distance over the hull outline, less half a cell."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    hx, hy = HALF
    # 2 cm apart, so no 24 mm chair leg fits between two points.
    outline = [(u, side * hy) for u in np.linspace(-hx, hx, 16)
               for side in (-1, 1)] + \
              [(side * hx, v) for v in np.linspace(-hy, hy, 12)
               for side in (-1, 1)]
    r, origin = meta['resolution'], meta['origin']
    gap = math.inf
    for u, v in outline:
        col = int((x + c * u - s * v - origin[0]) / r)
        row = int((y + s * u + c * v - origin[1]) / r)
        gap = min(gap, float(distance[row, col]) - r / 2)
    return gap


class NavEval(Node):

    def __init__(self):
        super().__init__('nav_eval')
        self.declare_parameter('reference', '')
        self.declare_parameter('spawn', [0.0, 0.0, 0.0])
        self.declare_parameter('goals', 20)
        self.declare_parameter('seed', 1)
        self.declare_parameter('output_dir', 'nav_eval')
        occupied, _, self.meta = load_map(
            self.get_parameter('reference').value)
        self.spawn = list(self.get_parameter('spawn').value)
        self.distance = ndimage.distance_transform_edt(~occupied) \
            * self.meta['resolution']
        self.goals = sample_goals(
            occupied, self.meta, self.spawn,
            self.get_parameter('goals').value,
            self.get_parameter('seed').value)
        self.output = Path(self.get_parameter('output_dir').value).resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        self.truth = None
        self.track = []
        self.commands = 0
        self.create_subscription(Odometry, '/ground_truth/odom',
                                 self.on_truth, 20)
        self.create_subscription(TwistStamped, '/cmd_vel_nav',
                                 self.on_command, 20)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def on_truth(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        self.truth = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y),
                                           1 - 2 * (q.y * q.y + q.z * q.z)))
        self.track.append((self.now(), *self.truth))

    def on_command(self, _):
        self.commands += 1

    def to_map(self, x, y, yaw):
        """World pose -> map frame (the map's origin is the spawn pose)."""
        sx, sy, syaw = self.spawn
        c, s = math.cos(syaw), math.sin(syaw)
        return (c * (x - sx) + s * (y - sy), -s * (x - sx) + c * (y - sy),
                yaw - syaw)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_until(self, done, timeout):
        """Spin until done() or timeout sim seconds; track the hull gap."""
        end, gap = self.now() + timeout, math.inf
        while not done() and self.now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.truth is not None:
                gap = min(gap, hull_gap(self.truth, self.distance, self.meta))
        return gap

    def run_goal(self, goal):
        x, y, yaw = self.to_map(*goal)
        request = NavigateToPose.Goal()
        request.pose.header.frame_id = 'map'
        request.pose.pose.position.x, request.pose.pose.position.y = x, y
        request.pose.pose.orientation.z = math.sin(yaw / 2)
        request.pose.pose.orientation.w = math.cos(yaw / 2)
        sent = self.nav.send_goal_async(request)
        self.spin_until(sent.done, 10.0)
        if not sent.done() or not sent.result().accepted:
            return {'goal': goal, 'status': 'rejected', 'success': False}
        start, commands = self.now(), self.commands
        result = sent.result().get_result_async()
        gap = self.spin_until(result.done, TIMEOUT)
        took = self.now() - start
        if not result.done():
            sent.result().cancel_goal_async()
            self.spin_until(lambda: False, 2.0)
        status = result.result().status if result.done() else None
        miss = math.dist(self.truth[:2], goal[:2])
        return {
            'goal': [round(v, 3) for v in goal],
            'status': {GoalStatus.STATUS_SUCCEEDED: 'succeeded',
                       GoalStatus.STATUS_ABORTED: 'aborted',
                       GoalStatus.STATUS_CANCELED: 'canceled'}.get(
                           status, 'timeout'),
            'time_s': round(took, 1),
            'miss_m': round(miss, 3),
            'min_gap_m': round(gap, 3),
            'collision': gap <= 0.0,
            'cmd_rate_hz': round((self.commands - commands)
                                 / max(took, 1e-9), 1),
            'success': status == GoalStatus.STATUS_SUCCEEDED
            and miss < REACHED,
        }

    def run(self):
        while self.truth is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.nav.wait_for_server(timeout_sec=120.0):
            self.get_logger().error('Nav2 not up')
            return 1
        # Costmaps fill and AMCL settles.
        wall = time.monotonic()
        while time.monotonic() - wall < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        runs = []
        for index, goal in enumerate(self.goals):
            runs.append(self.run_goal(goal))
            runs[-1]['end'] = [round(v, 3) for v in self.truth]
            self.get_logger().info(f'goal {index + 1}: {runs[-1]}')
        successes = sum(r['success'] for r in runs)
        summary = {
            'goals': len(runs),
            'succeeded': successes,
            'success_rate': successes / len(runs),
            'collisions': sum(r.get('collision', False) for r in runs),
            'min_gap_m': min(r.get('min_gap_m', math.inf) for r in runs),
            'cmd_rate_hz_median': float(np.median(
                [r['cmd_rate_hz'] for r in runs if 'cmd_rate_hz' in r])),
        }
        # Planar TUM: x y 0 and the yaw as a z-axis quaternion.
        (self.output / 'truth.tum').write_text(''.join(
            f'{t:.3f} {x:.4f} {y:.4f} 0 0 0 {math.sin(a / 2):.6f} '
            f'{math.cos(a / 2):.6f}\n' for t, x, y, a in self.track[::5]),
            encoding='utf-8')
        (self.output / 'nav_eval.json').write_text(json.dumps(
            {'summary': summary, 'runs': runs}, indent=2) + '\n',
            encoding='utf-8')
        self.get_logger().info(json.dumps(summary))
        return 0 if summary['success_rate'] >= PASS_RATE else 1


def main():
    rclpy.init()
    node = NavEval()
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 1
    node.destroy_node()
    rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
