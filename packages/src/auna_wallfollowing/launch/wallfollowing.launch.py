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


from launch_ros.actions import Node, PushRosNamespace
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_context import LaunchContext
from launch_ros.substitutions import FindPackageShare
import os


def include_launch_description(context: LaunchContext):
    """Return launch description"""

    launch_description_content = []

    # Get robot_index and construct namespace
    robot_index = int(os.environ.get('ROBOT_INDEX', '0'))
    namespace = f"robot{robot_index}"

    # Get path to config file
    config_file = PathJoinSubstitution([
        FindPackageShare('auna_wallfollowing'),
        'config',
        'wallfollowing.yaml'
    ])

    wallfollowing_node = Node(
        package='auna_wallfollowing',
        executable='wallfollowing',
        name='wallfollowing_node',
        output='screen',
        parameters=[config_file],
    )

    push_ns = PushRosNamespace(namespace)
    launch_description_content.append(push_ns)

    # Add the wallfollowing node to the launch description content
    launch_description_content.append(wallfollowing_node)

    return launch_description_content   


def generate_launch_description():
    """Return launch description"""
    # Launch Description
    launch_description = LaunchDescription()

    # Declare config_file argument
    launch_description.add_action(DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to the configuration file for wallfollowing parameters.'
    ))

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
