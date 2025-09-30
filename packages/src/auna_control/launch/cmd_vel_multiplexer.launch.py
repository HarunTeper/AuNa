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


import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    param_file = LaunchConfiguration('param_file')
    topic_file = LaunchConfiguration('topic_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get initial_source from environment variable or use default
    robot_index = os.environ.get('ROBOT_INDEX', '0')
    initial_source = os.environ.get('INITIAL_SOURCE', 'OFF')

    namespace = f'robot{robot_index}'

    default_param_file = PathJoinSubstitution([
        FindPackageShare('auna_control'),
        'config',
        'params.yaml'
    ])
    default_topic_file = PathJoinSubstitution([
        FindPackageShare('auna_control'),
        'config',
        'topics.yaml'
    ])

    declare_param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param_file,
        description='Path to the cmd_vel_multiplexer parameter YAML file'
    )

    declare_topic_file_arg = DeclareLaunchArgument(
        'topic_file',
        default_value=default_topic_file,
        description='Path to the cmd_vel_multiplexer topic YAML file'
    )

    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the node (empty for none)'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    cmd_vel_multiplexer_node = Node(
        package='auna_control',
        executable='cmd_vel_multiplexer_node',
        namespace=namespace,
        name='cmd_vel_multiplexer_node',
        output='screen',
        parameters=[param_file, {"topic_file": topic_file}, {
            'use_sim_time': use_sim_time,
            'initial_source': initial_source
        }]
    )

    return LaunchDescription([
        declare_param_file_arg,
        declare_topic_file_arg,
        declare_namespace_arg,
        declare_use_sim_time_arg,
        cmd_vel_multiplexer_node,
    ])
