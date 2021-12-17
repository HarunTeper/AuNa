#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Joep Tool

import os
from ament_index_python.packages import get_package_share_directory
from car_simulator import yaml_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,TextSubstitution
from car_simulator import yaml_launch
from launch_ros.actions import Node


def include_spawn_description(context: LaunchContext):
    
    # Package Directories
    pkg_dir = get_package_share_directory('car_simulator')

    # Paths to folders and files
    launch_file_dir = os.path.join(pkg_dir, 'launch', 'spawn')

    # Launch Argument Configurations
    robot_number = LaunchConfiguration('robot_number', default='2')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='racetrack_decorated')

    # Names and poses of the robots
    robots = []
    for num in range(int(robot_number.perform(context))):
        robots.append({
            'name': 'robot'+str(num), 
            'namespace': 'robot'+str(num), 
            'x_pose': yaml_launch.get_yaml_value("map_params",world_name.perform(context), ["spawn","offset","x"])+num*yaml_launch.get_yaml_value("map_params",world_name.perform(context), ["spawn","linear","x"]),
            'y_pose': yaml_launch.get_yaml_value("map_params",world_name.perform(context), ["spawn","offset","y"])+num*yaml_launch.get_yaml_value("map_params",world_name.perform(context), ["spawn","linear","y"]),
            'z_pose': yaml_launch.get_yaml_value("map_params",world_name.perform(context), ["spawn","offset","z"])+num*yaml_launch.get_yaml_value("map_params",world_name.perform(context), ["spawn","linear","z"]),
            }
        )
    
    # Nodes and other launch files
    car_spawn_cmds = []
    for robot in robots:
        car_spawn_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'spawn_single_robot.launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'x_pose':TextSubstitution(text=str(robot['x_pose'])),
                    'y_pose':TextSubstitution(text=str(robot['y_pose'])),
                    'z_pose':TextSubstitution(text=str(robot['z_pose'])),
                    'namespace':robot['namespace'],
                    'urdf_namespace':robot['namespace']+'/',
                    'name':robot['name']
                }.items()
            )
        )
    
    car_spawn_cmds.append(Node(
            package='car_simulator',
            executable='globalTF',
            name='globalTF',
            output='screen'
        )
    )
    
    return car_spawn_cmds

def generate_launch_description():

    # Launch Arguments
    robot_number_arg = DeclareLaunchArgument(
        'robot_number',
        default_value='2',
        description='Number of spawned robots'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='racetrack_decorated',
        description='Gazebo world file name in /worlds folder'
    )

    # Launch Description
    ld = LaunchDescription()

    ld.add_action(robot_number_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(world_name_arg)

    ld.add_action(OpaqueFunction(function=include_spawn_description))

    return ld
