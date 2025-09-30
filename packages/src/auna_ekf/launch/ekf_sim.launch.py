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


# packages/src/auna_ekf/launch/ekf.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch.launch_context import LaunchContext


def include_launch_description(context: LaunchContext):
    """Return launch description"""
    robot_index = int(os.environ.get('ROBOT_INDEX', '0'))

    # Individual components
    tf_remap = SetRemap(src='/tf', dst='tf')
    tf_static_remap = SetRemap(src='/tf_static', dst='tf_static')

    auna_ekf_pkg_share = get_package_share_directory('auna_ekf')
    ekf_local_config_path = os.path.join(
        auna_ekf_pkg_share, 'config', 'ekf', 'ekf_local.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_local_config_path,
            {'use_sim_time': use_sim_time}
        ],
    )

    # Namespace group for proper topic scoping
    ekf_group = GroupAction([
        PushRosNamespace(f'robot{robot_index}'),
        tf_remap,
        tf_static_remap,
        ekf_node
    ])

    launch_actions = []
    launch_actions.append(ekf_group)
    return launch_actions


def generate_launch_description():

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    return LaunchDescription([
        declare_use_sim_time_arg,
        OpaqueFunction(function=include_launch_description)
    ])
