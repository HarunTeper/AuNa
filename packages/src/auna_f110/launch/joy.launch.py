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


"""Sensor launch file."""
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from auna_common import yaml_launch


def generate_launch_description():
    """Return launch description."""

    # Paths to folders and files
    physical_pkg_dir = get_package_share_directory('auna_f110')
    config_file_dir = os.path.join(physical_pkg_dir, 'config')
    joy_config_file_path = os.path.join(config_file_dir, 'teleop_joy.yaml')

    # Launch arguments
    namespace_arg = DeclareLaunchArgument('namespace', default_value='robot')
    use_ps4_arg = DeclareLaunchArgument('use_ps4', default_value='false')
    use_g29_arg = DeclareLaunchArgument('use_g29', default_value='false')

    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    use_ps4 = LaunchConfiguration('use_ps4')
    use_g29 = LaunchConfiguration('use_g29')

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_linux_node',
        namespace=namespace,
        parameters=[yaml_launch.get_yaml_value(
            joy_config_file_path, ['joy_node', 'ros__parameters'])],
    )

    teleop_joy_node_ps4 = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        namespace=namespace,
        parameters=[yaml_launch.get_yaml_value(
            joy_config_file_path, ['teleop_joy_ps4', 'ros__parameters'])],
        condition=IfCondition(use_ps4),
    )

    teleop_joy_node_g29 = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        namespace=namespace,
        parameters=[yaml_launch.get_yaml_value(
            joy_config_file_path, ['joy_teleop_g29', 'ros__parameters'])],
        condition=IfCondition(use_g29),
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(namespace_arg)
    launch_description.add_action(use_ps4_arg)
    launch_description.add_action(use_g29_arg)

    launch_description.add_action(joy_node)
    launch_description.add_action(teleop_joy_node_ps4)
    launch_description.add_action(teleop_joy_node_g29)

    return launch_description
