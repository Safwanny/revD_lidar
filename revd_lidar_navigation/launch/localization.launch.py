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
Localize on a saved map (config/localization.yaml).

localization:=amcl (default): map_server serves <map>.yaml and AMCL (Omni
model) owns map -> odom. localization:=slam_toolbox: slam_toolbox in
localization mode on <map>.posegraph. map is the path without extension, so
both read the same mapping run. initial_x/y/yaw: the starting guess in the
map frame (the mapping run's spawn pose is 0, 0, 0). Runs beside the
simulation or the robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _nodes(context):
    share = FindPackageShare('revd_lidar_navigation').perform(context)
    config = f'{share}/config/localization.yaml'
    slam_config = f'{share}/config/slam.yaml'
    value = (lambda name: LaunchConfiguration(name).perform(context))
    sim_time = {'use_sim_time': value('use_sim_time') == 'true'}
    map_path = value('map')
    pose = [float(value(name))
            for name in ('initial_x', 'initial_y', 'initial_yaw')]
    if value('localization') == 'amcl':
        manager = {'node_names': ['map_server', 'amcl']}
        nodes = [
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                parameters=[config, sim_time,
                            {'yaml_filename': map_path + '.yaml'}],
                output='screen',
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                parameters=[config, sim_time, {'initial_pose': dict(
                    zip(('x', 'y', 'yaw'), pose))}],
                output='screen',
            ),
        ]
    else:
        manager = {'node_names': ['slam_toolbox'], 'bond_timeout': 0.0}
        nodes = [
            Node(
                package='slam_toolbox',
                executable='localization_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[slam_config, config, sim_time, {
                    'map_file_name': map_path,
                    'map_start_pose': pose,
                }],
                output='screen',
            ),
        ]
    return nodes + [Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[config, sim_time, manager],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'localization', default_value='amcl',
            choices=['amcl', 'slam_toolbox']),
        DeclareLaunchArgument(
            'map',
            description=('Saved map path without extension: <map>.yaml for '
                         'AMCL, <map>.posegraph for slam_toolbox.')),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the simulation clock; false on the robot.'),
        OpaqueFunction(function=_nodes),
    ])
