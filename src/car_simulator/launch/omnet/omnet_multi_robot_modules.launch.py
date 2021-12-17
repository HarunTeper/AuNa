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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def include_omnet_description(context: LaunchContext):

    # Package Directories
    pkg_dir = get_package_share_directory('car_simulator')

    # Paths to folders and files
    omnet_launch_file_dir = os.path.join(pkg_dir, 'launch', 'omnet')

    # Launch Argument Configurations
    robot_number = LaunchConfiguration('robot_number', default='2')

    # Nodes and other launch files
    cmds = []

    for num in range(int(robot_number.perform(context))):
        cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(omnet_launch_file_dir, 'omnet_single_robot_modules.launch.py')),
                launch_arguments={
                    'namespace': "robot"+str(num),
                }.items(),
            )
        )
    
    cmds.append(
        Node(
            package='car_simulator',
            executable='arteryInfo',
            name='arteryInfo',
            output='screen'
        )
    )

    return cmds

def generate_launch_description():

    # Launch Arguments
    robot_number_arg = DeclareLaunchArgument(
        'robot_number',
        default_value='2',
        description='Number of spawned robots'
    )
    
    # Launch Description
    ld = LaunchDescription()

    ld.add_action(robot_number_arg)

    ld.add_action(OpaqueFunction(function=include_omnet_description))

    return ld
