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


"""Multiple cars omnet module launch file."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import PushRosNamespace, Node


def include_launch_description(context: LaunchContext):
    """Return launch description."""
    # Package Directories
    pkg_dir = get_package_share_directory('auna_comm')

    # Paths to folders and files
    omnet_launch_file_dir = os.path.join(pkg_dir, 'launch')

    # Launch Argument Configurations
    robot_number = LaunchConfiguration('robot_number', default='2')
    namespace = LaunchConfiguration('namespace')

    # Get resolved values
    ns_value = context.perform_substitution(namespace)
    robot_number_value = int(context.perform_substitution(robot_number))

    # Nodes and other launch files
    launch_description_content = []

    for num in range(robot_number_value):
        # Define the namespace for this robot using the provided namespace parameter
        robot_ns = f'{ns_value}{num}'

        # Define the filter index (robot_index - 1)
        filter_idx = str(num - 1)

        # Create group for this robot's nodes
        group_actions = [
            PushRosNamespace(robot_ns),

            # Include cam_communication node
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    omnet_launch_file_dir, 'cam_communication.launch.py')),
                launch_arguments={
                    'robot_index': str(num),
                    'filter_index': filter_idx,
                }.items(),
            )
        ]

        # Only add cam_receiver node for robots other than the first one (not robot0)
        if num > 0:
            group_actions.append(
                Node(
                    package='auna_comm',
                    executable='cam_receiver',
                    name='cam_receiver',
                    parameters=[{
                        'filter_index': int(filter_idx),
                        'robot_namespace': robot_ns
                    }],
                    output='screen'
                )
            )

        launch_description_content.append(GroupAction(group_actions))

    return launch_description_content


def generate_launch_description():
    """Return launch description."""
    # Launch Arguments
    robot_number_arg = DeclareLaunchArgument(
        'robot_number',
        default_value='2',
        description='Number of spawned robots'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace'
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

    launch_description.add_action(robot_number_arg)
    launch_description.add_action(namespace_arg)
    launch_description.add_action(enable_cam_logging_arg)
    launch_description.add_action(cam_log_file_path_arg)
    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
