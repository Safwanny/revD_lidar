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
Drive Rev D at the walls through the safety chain and check it stops short.

navigation_basic, spawned at (SPAWN) facing +X, in the open corner between
the interior wall and the north-east walls. Commands go in on the twist_mux
inputs above the speed cap; Gazebo's true pose (tests may read it) measures
the gap between the hull and the walls. In order: the e-stop lock holds the
robot, teleop outranks Nav2 and Nav2 the policy, /lidar/ok switches the
governor to tof_only mode and back at runtime, then twice (facing +X, then
turned to face -X, so every face and two opposite corners lead) it drives to
the east wall, slides along it (slowed, never stopped, by the wall beside
it) into the north-east corner, escapes the corner both ways, and drives
diagonally back into the corner and away again.
"""

import math
import os
import signal
import time
import unittest

from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool

SPAWN = (3.0, 1.0)
EAST_WALL_X = 5.0 - 0.075   # inner faces, navigation_basic.sdf
NORTH_WALL_Y = 4.0 - 0.075
HULL = ((0.15, 0.11), (0.15, -0.11), (-0.15, -0.11), (-0.15, 0.11))
COMMAND = 0.6               # above the 0.45 m/s cap (config/safety.yaml)
SPEED_CAP = 0.45
SIDE_CAP = 0.6 * SPEED_CAP  # a wall within 5 cm of the side
MIN_GAP = 0.03


def generate_test_description():
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('revd_lidar_bringup'), 'launch',
             'simulation.launch.py'])),
        launch_arguments={
            'world': 'navigation_basic',
            'x': str(SPAWN[0]),
            'y': str(SPAWN[1]),
            'gui': 'false',
            'rviz': 'false',
            'tof_markers': 'false',
            'render_engine': 'ogre2',
        }.items(),
    )
    return LaunchDescription([simulation, launch_testing.actions.ReadyToTest()])


class TestSafetyChain(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('safety_chain_test')
        cls.pose = None
        cls.speed = 0.0
        cls.offset = None
        cls.mux = None
        cls.governed = None
        cls.node.create_subscription(
            Odometry, '/ground_truth/odom', cls.on_truth, 10)
        cls.node.create_subscription(
            TwistStamped, '/cmd_vel_mux',
            lambda message: setattr(cls, 'mux', message.twist), 10)
        cls.inputs = {
            name: cls.node.create_publisher(
                TwistStamped, f'/cmd_vel_{name}', 10)
            for name in ('teleop', 'nav', 'policy')}
        cls.node.create_subscription(
            TwistStamped, '/mobile_base_controller/reference',
            lambda message: setattr(cls, 'governed', message.twist), 10)
        cls.e_stop = cls.node.create_publisher(Bool, '/e_stop', 10)
        cls.lidar_ok = cls.node.create_publisher(
            Bool, '/lidar/ok',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def on_truth(cls, message):
        p = message.pose.pose.position
        if cls.offset is None:
            # World frame if it starts at the spawn point, else spawn-relative.
            near = math.dist((p.x, p.y), SPAWN) < 0.2
            cls.offset = (0.0, 0.0) if near else SPAWN
        q = message.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        cls.pose = (p.x + cls.offset[0], p.y + cls.offset[1], yaw)
        v = message.twist.twist.linear
        cls.speed = math.hypot(v.x, v.y)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def hull(self):
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        return [(x + c * hx - s * hy, y + s * hx + c * hy) for hx, hy in HULL]

    def gaps(self):
        """Distances from the hull to the east and the north wall."""
        corners = self.hull()
        return (EAST_WALL_X - max(cx for cx, _ in corners),
                NORTH_WALL_Y - max(cy for _, cy in corners))

    def publish(self, source, vx, vy, wz=0.0):
        command = TwistStamped()
        command.header.stamp = self.node.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'
        command.twist.linear.x, command.twist.linear.y = vx, vy
        command.twist.angular.z = wz
        self.inputs[source].publish(command)

    def settle(self):
        # The 0.5 s reference timeout stops the wheels; a slow sim (CI) takes
        # longer in wall time, so wait for it instead of a fixed spin.
        deadline = time.monotonic() + 10.0
        self.spin(1.0)
        while self.speed >= 0.01 and time.monotonic() < deadline:
            self.spin(0.2)

    def drive(self, vx, vy, seconds, wz=0.0, until_stopped=False):
        """
        Publish teleop at 20 Hz; return the top speed and smallest gap.

        (vx, vy) is in the world frame, turned into the body frame by the
        true heading, so a run goes the same way whatever the robot faces.
        """
        top, closest, still_since = 0.0, math.inf, None
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
            self.publish('teleop', c * vx + s * vy, -s * vx + c * vy, wz)
            self.spin(0.05)
            top = max(top, self.speed)
            closest = min(closest, *self.gaps())
            if until_stopped and top > 0.1:
                still_since = (still_since or time.monotonic()) \
                    if self.speed < 0.005 else None
                if still_since and time.monotonic() - still_since > 2.0:
                    break
        self.settle()
        return top, closest

    def turn_to(self, yaw):
        """Turn on the spot to ``yaw`` (world frame) with teleop."""
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            error = (yaw - self.pose[2] + math.pi) % (2 * math.pi) - math.pi
            if abs(error) < 0.05:
                break
            wz = math.copysign(min(1.0, max(0.2, 2.0 * abs(error))), error)
            self.publish('teleop', 0.0, 0.0, wz)
            self.spin(0.05)
        self.settle()

    def stopped_at_wall(self, name, gap, closest):
        print(f'{name}: stopped {gap:.3f} m from the wall, closest '
              f'{closest:.3f} m')
        self.assertGreater(closest, 0.0, f'{name}: touched the wall')
        self.assertLess(gap, MIN_GAP + 0.03, f'{name}: stopped far short')

    def wall_runs(self, label):
        """Run the wall sequence, whatever way the robot faces."""
        top, closest = self.drive(COMMAND, 0.0, 60.0,
                                  until_stopped=True)
        print(f'{label} forward: top speed {top:.3f} m/s')
        self.assertGreater(top, SPEED_CAP - 0.05, f'{label} forward: never '
                           'reached the cap in the open')
        self.assertLessEqual(top, SPEED_CAP + 0.02,
                             f'{label} forward: over the cap')
        self.stopped_at_wall(f'{label} forward', self.gaps()[0], closest)

        # Along the east wall, 3 cm away: slowed, not stopped.
        top, closest = self.drive(0.0, COMMAND, 60.0,
                                  until_stopped=True)
        print(f'{label} along the wall: top speed {top:.3f} m/s')
        self.assertGreater(top, SIDE_CAP - 0.05, f'{label} along: too slow')
        self.assertLessEqual(top, SIDE_CAP + 0.02,
                             f'{label} along: side cap unused')
        self.stopped_at_wall(f'{label} along the wall', self.gaps()[1],
                             closest)

        # In the corner, each way out moves.
        for name, vx, vy, wall in (('out of the corner -y', 0.0, -COMMAND, 1),
                                   ('out of the corner -x', -COMMAND, 0.0, 0)):
            before = self.gaps()[wall]
            self.drive(vx, vy, 1.5)
            print(f'{label} {name}: gained '
                  f'{self.gaps()[wall] - before:.3f} m')
            self.assertGreater(self.gaps()[wall] - before, 0.1,
                               f'{label} {name}')
            # The test's stamps are wall time against sim time: the
            # reference timeout must still stop the wheels.
            self.assertLess(self.speed, 0.01, f'{label} {name}: still moving')

        diagonal = COMMAND / math.sqrt(2.0)
        _, closest = self.drive(diagonal, diagonal, 60.0, until_stopped=True)
        self.stopped_at_wall(f'{label} diagonal', min(self.gaps()), closest)
        before = min(self.gaps())
        self.drive(-diagonal, -diagonal, 2.0)
        self.assertGreater(min(self.gaps()) - before, 0.1,
                           f'{label} diagonal: stuck')

    def test_safety_chain(self):
        deadline = time.monotonic() + 120.0
        while self.pose is None and time.monotonic() < deadline:
            self.spin(0.2)
        self.assertIsNotNone(self.pose, 'no ground truth')
        self.spin(10.0)  # controllers, EKF and the ToF clouds come up

        self.e_stop.publish(Bool(data=True))
        self.spin(0.5)
        start = self.pose
        self.drive(COMMAND, 0.0, 3.0)
        self.assertLess(math.dist(start[:2], self.pose[:2]), 0.02,
                        'moved under e-stop')
        self.e_stop.publish(Bool(data=False))
        self.spin(0.5)

        # Every input live at once: the highest priority wins.
        for winner, losers in (('teleop', ('nav', 'policy')),
                               ('nav', ('policy',))):
            type(self).mux = None
            end = time.monotonic() + 1.0
            while time.monotonic() < end:
                self.publish(winner, -0.1, 0.0)
                for loser in losers:
                    self.publish(loser, 0.2, 0.0)
                self.spin(0.05)
            self.settle()
            self.assertIsNotNone(self.mux, 'no /cmd_vel_mux output')
            self.assertAlmostEqual(self.mux.linear.x, -0.1,
                                   msg=f'{winner} did not win')
        self.assertLess(self.pose[0], start[0] - 0.05, 'did not back up')

        # LiDAR lost: only axis moves. Back to normal once it is ok again.
        for ok, vy in ((False, 0.0), (True, -0.1)):
            self.lidar_ok.publish(Bool(data=ok))
            self.spin(0.5)
            end = time.monotonic() + 0.5
            while time.monotonic() < end:
                self.publish('teleop', -0.2, -0.1)
                self.spin(0.05)
            self.settle()
            self.assertAlmostEqual(self.governed.linear.y, vy, places=3,
                                   msg=f'/lidar/ok {ok}: wrong mode')
            self.assertAlmostEqual(self.governed.linear.x, -0.2, places=3)

        self.wall_runs('facing +x')
        # Clear of the walls now: turning is free.
        self.turn_to(math.pi)
        print(f'turned to {self.pose[2]:.3f} rad')
        self.assertGreater(abs(self.pose[2]), math.pi - 0.1, 'did not turn')
        self.wall_runs('facing -x')


@launch_testing.post_shutdown_test()
class TestGazeboStopped(unittest.TestCase):

    def test_kill_headless_gazebo(self):
        # The headless server ignores SIGINT/SIGTERM and outlives the launch.
        # Its world file carries the launch's pid, which is this process.
        world = f'revd_lidar_world_{os.getpid()}_'
        for entry in os.listdir('/proc'):
            try:
                with open(f'/proc/{entry}/cmdline', 'rb') as cmdline:
                    if world.encode() in cmdline.read():
                        os.kill(int(entry), signal.SIGKILL)
            except (OSError, ValueError):
                pass
