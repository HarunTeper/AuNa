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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace',default='')

    # Launch Arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace for ROS nodes and topics'
    )

    # Nodes and other launch files
    omnetTX_cmd = Node(
            package='car_simulator',
            executable='omnetTX',
            name='omnetTX',
            namespace = namespace,
            arguments= {namespace},
            output='screen'
        )

    omnetRX_cmd = Node(
            package='car_simulator',
            executable='omnetRX',
            name='omnetRX',
            namespace = namespace,
            arguments= {namespace},
            output='screen'
        )

    # Launch Description
    ld = LaunchDescription()

    ld.add_action(namespace_arg)

    ld.add_action(omnetTX_cmd)
    ld.add_action(omnetRX_cmd)
    return ld
