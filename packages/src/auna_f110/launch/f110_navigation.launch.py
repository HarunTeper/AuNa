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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Return launch description."""

    # Package Directories
    navigation_pkg_dir = get_package_share_directory('auna_nav2')

    # Paths to folders and files
    nav_launch_file_dir = os.path.join(navigation_pkg_dir, 'launch')

    # Paths to folders and files
    default_rviz_config_file = os.path.join(
        navigation_pkg_dir, 'rviz', 'config_navigation_namespace.rviz')
    default_params_file = os.path.join(
        navigation_pkg_dir, 'config', 'nav2_params', 'nav2_params.yaml')
    map_path = os.path.join(navigation_pkg_dir, 'maps', 'arena', 'map.yaml')

    # Launch arguments
    namespace_arg = DeclareLaunchArgument('namespace', default_value='robot')

    # Launch Configuration
    namespace = LaunchConfiguration('namespace')
    rviz_config = LaunchConfiguration(
        'rviz_config', default=default_rviz_config_file)

    nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            nav_launch_file_dir, 'navigation_single_robot.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'rviz_config': rviz_config,
            'map': map_path,
            'params_file': default_params_file,
            'enable_slam': 'False',  # slam can only be used without a namespace
            'enable_localization': 'False',
            'enable_navigation': 'True',
            'enable_rviz': 'False',
            'enable_map_server': 'True',
        }.items(),
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(namespace_arg)
    launch_description.add_action(nav_cmd)

    return launch_description
