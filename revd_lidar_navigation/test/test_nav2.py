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
AMCL on a map, Nav2 to a goal, and teleop overriding Nav2 on the way.

Starts the simulation (revd_lidar_bringup) with localization.launch.py and
nav2.launch.py beside it, as on the robot.

navigation_basic, spawned at the origin facing +X. The map is the world's
reference grid (world_grid), rendered into a temporary folder, so the test
needs no saved SLAM map; map = world here. AMCL starts from a guess
GUESS_ERROR off the true pose. One goal past the centre column: once Nav2
has the robot moving, teleop holds zero for HOLD seconds and the robot must
stand still; then teleop goes quiet, Nav2 resumes and reaches the goal, and
AMCL's map -> base_footprint must agree with Gazebo's true pose. Before
the goal, the local costmap must hold the scan's obstacles.
"""

import math
from pathlib import Path
import subprocess
import tempfile
import time
import unittest

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

GOAL = (-3.5, -2.5)
GUESS_ERROR = (0.2, -0.15, 0.1)
GOAL_TIMEOUT = 180.0
HOLD = 3.0
# Nav2's goal checker is 0.15 m.
TOLERANCE = 0.25
LOCALIZED = 0.1
MAP = Path(tempfile.mkdtemp()) / 'reference'


def generate_test_description():
    world = Path(get_package_share_directory('revd_lidar_gazebo'),
                 'worlds', 'navigation_basic.sdf')
    subprocess.run(['ros2', 'run', 'revd_lidar_tools', 'world_grid',
                    str(world), str(MAP)], check=True)

    def launch(package, name, **arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare(package), 'launch', name])),
            launch_arguments=arguments.items())

    return LaunchDescription([
        launch('revd_lidar_bringup', 'simulation.launch.py',
               world='navigation_basic', gui='false', rviz='false',
               tof_markers='false', render_engine='ogre2'),
        launch('revd_lidar_navigation', 'localization.launch.py',
               localization='amcl', map=str(MAP),
               initial_x=str(GUESS_ERROR[0]), initial_y=str(GUESS_ERROR[1]),
               initial_yaw=str(GUESS_ERROR[2])),
        launch('revd_lidar_navigation', 'nav2.launch.py'),
        launch_testing.actions.ReadyToTest(),
    ])


class TestNav2(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('nav2_test')
        cls.pose = None
        cls.speed = 0.0
        cls.lethal = 0
        cls.node.create_subscription(
            Odometry, '/ground_truth/odom', cls.on_truth, 10)
        cls.node.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', cls.on_costmap,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        cls.teleop = cls.node.create_publisher(
            TwistStamped, '/cmd_vel_teleop', 10)
        cls.nav = ActionClient(cls.node, NavigateToPose, 'navigate_to_pose')
        cls.tf = Buffer()
        cls.listener = TransformListener(cls.tf, cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def on_truth(cls, message):
        p = message.pose.pose.position
        cls.pose = (p.x, p.y)
        v = message.twist.twist.linear
        cls.speed = math.hypot(v.x, v.y)

    @classmethod
    def on_costmap(cls, message):
        cls.lethal = sum(cost >= 99 for cost in message.data)

    def spin(self, seconds, teleop=False):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if teleop:
                message = TwistStamped()
                message.header.stamp = self.node.get_clock().now().to_msg()
                message.header.frame_id = 'base_link'
                self.teleop.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_nav2(self):
        deadline = time.monotonic() + 120.0
        while self.pose is None and time.monotonic() < deadline:
            self.spin(0.2)
        self.assertIsNotNone(self.pose, 'no ground truth')
        self.assertTrue(self.nav.wait_for_server(timeout_sec=120.0),
                        'Nav2 not up')
        self.spin(10.0)  # controllers, EKF, AMCL and the costmaps settle
        # The local costmap marks the scan (an empty one leaves MPPI blind).
        self.assertGreater(self.lethal, 0, 'local costmap has no obstacles')

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = GOAL
        goal.pose.pose.orientation.w = 1.0
        sent = self.nav.send_goal_async(goal)
        while not sent.done():
            self.spin(0.1)
        self.assertTrue(sent.result().accepted)
        result = sent.result().get_result_async()

        start = time.monotonic()
        while self.speed < 0.1 and time.monotonic() - start < 30.0:
            self.spin(0.1)
        self.assertGreater(self.speed, 0.1, 'Nav2 never moved the robot')
        self.spin(1.0, teleop=True)  # the robot brakes
        held = self.pose
        self.spin(HOLD - 1.0, teleop=True)
        moved = math.dist(held, self.pose)
        print(f'moved {moved:.3f} m while teleop held zero')
        self.assertLess(moved, 0.02, 'Nav2 moved the robot under teleop')
        self.assertFalse(result.done(), 'the goal ended under teleop')

        while not result.done() \
                and time.monotonic() - start < GOAL_TIMEOUT:
            self.spin(0.1)
        status = result.result().status if result.done() else None
        miss = math.dist(self.pose, GOAL)
        estimate = self.tf.lookup_transform(
            'map', 'base_footprint', Time()).transform.translation
        error = math.dist(self.pose, (estimate.x, estimate.y))
        print(f'status {status}, {miss:.2f} m from the goal, '
              f'AMCL {error:.3f} m from truth')
        self.assertEqual(status, GoalStatus.STATUS_SUCCEEDED)
        self.assertLess(miss, TOLERANCE)
        self.assertLess(error, LOCALIZED)
