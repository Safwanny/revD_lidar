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
Nav2 on the map (config/nav2.yaml, config/navigate.xml).

Needs map -> odom from localization.launch.py. Publishes on /cmd_vel_nav,
the twist_mux input below teleop, so the safety governor still limits it.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('revd_lidar_navigation')
    params = [PathJoinSubstitution([share, 'config', 'nav2.yaml']),
              {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    to_mux = [('cmd_vel', '/cmd_vel_nav')]
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the simulation clock; false on the robot.'),
        Node(
            package='nav2_controller',
            executable='controller_server',
            parameters=params,
            remappings=to_mux,
            output='screen',
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            parameters=params,
            output='screen',
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            parameters=params,
            remappings=to_mux,
            output='screen',
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            parameters=[*params, {
                'default_nav_to_pose_bt_xml': PathJoinSubstitution(
                    [share, 'config', 'navigate.xml'])}],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=params,
            output='screen',
        ),
    ])
