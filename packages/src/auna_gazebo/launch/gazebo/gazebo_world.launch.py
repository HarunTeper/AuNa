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


"""Gazebo world launch file."""
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_launch_description(context: LaunchContext):
    """Return launch description."""
    # Package Directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get the environment variable WORLD_NAME, fallback to 'racetrack_decorated' if not set
    world_name = os.environ.get('WORLD_NAME', 'racetrack_decorated')

    # auna_common paths
    auna_common_path = "/home/ubuntu/workspace/auna_common"
    world_file = os.path.join(
        auna_common_path,
        'worlds',
        f'{world_name}.world'
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': world_file,
                'use_sim_time': use_sim_time,
            }.items()
        ),

        Node(
            package='auna_gazebo',
            executable='robot_name_publisher',
            name='robot_name_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ]


def generate_launch_description():
    """Return launch description."""
    # Declare use_sim_time argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    return LaunchDescription([
        use_sim_time_arg,
        OpaqueFunction(function=include_launch_description)
    ])
