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


"""Spawn robot launch file."""
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    """Return launch description."""
    # Launch Argument Configurations
    name = LaunchConfiguration('name')
    urdf_namespace = LaunchConfiguration('urdf_namespace')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    r_orientation = LaunchConfiguration('r_orientation')
    p_orientation = LaunchConfiguration('p_orientation')
    y_orientation = LaunchConfiguration('y_orientation')

    # Launch Arguments
    name_arg = DeclareLaunchArgument(
        'name',
        default_value='robot',
        description='Gazebo robot object name'
    )
    urdf_namespace_arg = DeclareLaunchArgument(
        'urdf_namespace',
        default_value='',
        description='ROS2 robot urdf namespace (must not be empty)'
    )
    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Robot spawn x position'
    )
    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Robot spawn y position'
    )
    z_pose_arg = DeclareLaunchArgument(
        'z_pose',
        default_value='0.0',
        description='Robot spawn z position'
    )
    r_orientation_arg = DeclareLaunchArgument(
        'r_orientation',
        default_value='0.0',
        description='Robot spawn r orientation angle'
    )
    p_orientation_arg = DeclareLaunchArgument(
        'p_orientation',
        default_value='0.0',
        description='Robot spawn p orientation angle'
    )
    y_orientation_arg = DeclareLaunchArgument(
        'y_orientation',
        default_value='0.0',
        description='Robot spawn y orientation angle'
    )

    # Group spawner command with urdf_namespace handling
    spawner_group = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', name,
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-R', r_orientation,
            '-P', p_orientation,
            '-Y', y_orientation,
            '-robot_namespace', urdf_namespace,
        ],
    )

    return LaunchDescription([
        name_arg,
        urdf_namespace_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        r_orientation_arg,
        p_orientation_arg,
        y_orientation_arg,
        spawner_group
    ])
