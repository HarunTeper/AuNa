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

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch import LaunchDescription


def include_launch_description(context: LaunchContext):
    """Return launch description."""
    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    vesc_config = LaunchConfiguration('vesc_config')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Nodes and other launch files
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        namespace=namespace,
        parameters=[vesc_config],
        output='screen'
    )

    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        namespace=namespace,
        parameters=[vesc_config],
        output='screen',
        remappings=remappings
    )

    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        namespace=namespace,
        parameters=[vesc_config],
        output='screen'
    )

    launch_description_content = []
    launch_description_content.append(vesc_driver_node)
    launch_description_content.append(vesc_to_odom_node)
    launch_description_content.append(ackermann_to_vesc_node)

    return launch_description_content


def generate_launch_description():
    """Return launch description."""
    # Launch arguments
    namespace_arg = DeclareLaunchArgument('namespace', default_value='robot')
    vesc_config_arg = DeclareLaunchArgument(
        'vesc_config',
        default_value='/home/ubuntu/workspace/auna_common/config/f110/vesc.yaml',
        description='Path to config file for vesc'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(namespace_arg)
    launch_description.add_action(vesc_config_arg)

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
