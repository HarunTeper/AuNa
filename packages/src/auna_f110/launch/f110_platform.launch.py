# Copyright 2025 Harun Teper
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


"""Sensor launch file"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Return launch description"""

    # Paths to folders and files
    auna_physical_pkg_dir = get_package_share_directory('auna_f110')
    auna_physical_launch_file_dir = os.path.join(
        auna_physical_pkg_dir, 'launch')

    # Launch arguments
    namespace_arg = DeclareLaunchArgument('namespace', default_value='robot')

    # Launch configuration
    namespace = LaunchConfiguration('namespace')

    # Nodes and other launch files
    lidar_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            auna_physical_launch_file_dir, 'lidar_sensor.launch.py')),
        launch_arguments={
            'namespace': namespace
        }.items()
    )

    # Nodes and other launch files
    vesc_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            auna_physical_launch_file_dir, 'vesc.launch.py')),
        launch_arguments={
            'namespace': namespace
        }.items()
    )

    vesc_start_node = Node(
        package='auna_f110',
        executable='vesc_start',
        name='vesc_start',
        namespace=namespace,
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(namespace_arg)
    launch_description.add_action(lidar_launch_file)
    launch_description.add_action(vesc_launch_file)
    launch_description.add_action(vesc_start_node)

    return launch_description
