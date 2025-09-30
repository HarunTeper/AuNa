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


"""Single robot communication launch file"""

import os
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext


def include_launch_description(context: LaunchContext):
    """Return launch description"""

    communication_type = os.environ.get('COMMUNICATION_TYPE', 'cam')
    if communication_type != 'cam':
        return [LogInfo(msg="COMMUNICATION_TYPE is not 'cam', skipping CAM nodes.")]
    robot_index = int(os.environ.get('ROBOT_INDEX', '0'))

    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace')
    log_level = LaunchConfiguration('log_level')
    enable_cam_logging = LaunchConfiguration('enable_cam_logging')
    cam_log_file_path = LaunchConfiguration('cam_log_file_path')

    # Get resolved values
    namespace_value = context.perform_substitution(namespace)

    # Create unique log file path for each robot
    robot_log_file = context.perform_substitution(
        cam_log_file_path).replace('.log', f'_{namespace_value}.log')

    cam_communication_cmd = Node(
        package='auna_comm',
        executable='cam_communication',
        name='cam_communication',
        namespace=namespace,
        parameters=[
            {'robot_index': robot_index},
            {'enable_cam_logging': enable_cam_logging},
            {'cam_log_file_path': robot_log_file}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen'
    )

    cam_receiver_cmd = Node(
        package='auna_comm',
        executable='cam_receiver',
        name='cam_receiver',
        namespace=namespace,
        parameters=[{
            'filter_index': robot_index,
            'robot_namespace': namespace_value
        }],
        output='screen'
    )

    return [cam_communication_cmd, cam_receiver_cmd]


def generate_launch_description():
    """Return launch description"""

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot0',
        description='Robot namespace'
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )
    enable_cam_logging_arg = DeclareLaunchArgument(
        'enable_cam_logging',
        default_value='false',
        description='Enable CAM message logging'
    )
    cam_log_file_path_arg = DeclareLaunchArgument(
        'cam_log_file_path',
        default_value='/home/vscode/workspace/cam_messages.log',
        description='Path for CAM message log file'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(namespace_arg)
    launch_description.add_action(log_level_arg)
    launch_description.add_action(enable_cam_logging_arg)
    launch_description.add_action(cam_log_file_path_arg)
    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
