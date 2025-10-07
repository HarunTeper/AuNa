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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('auna_waypoints')
    world_name = os.environ.get('WORLD_NAME', 'racetrack_decorated')
    # Prefer precomputed yaw files if present, otherwise fall back to plain waypoints
    waypoints_dir = os.path.join(pkg_dir, 'config', world_name)
    default_waypoint_file = os.path.join(
        waypoints_dir, 'cacc_waypoints.yaml')

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value=world_name,
        description='Name of the world'
    )

    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value=default_waypoint_file,
        description='Path to the waypoint file'
    )

    return LaunchDescription([
        world_name_arg,
        waypoint_file_arg,
        Node(
            package='auna_waypoints',
            executable='cacc_waypoint_publisher',
            name='cacc_waypoint_publisher',
            output='screen',
            parameters=[
                {'waypoint_file': LaunchConfiguration('waypoint_file')}
            ]
        )
    ])
