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
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_launch_description(context: LaunchContext):
    """Return launch description."""
    # Package Directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_auna_gazebo = get_package_share_directory('auna_gazebo')

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

    # Setup Gazebo model paths
    # Add both the auna_gazebo models directory and common model locations
    gz_model_path = os.path.join(pkg_auna_gazebo, 'models')
    
    # Get existing GZ_SIM_RESOURCE_PATH if it exists
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    
    # Build the complete resource path
    # Include common Gazebo model locations
    resource_paths = [
        gz_model_path,
        '/usr/share/gazebo-11/models',  # Gazebo Classic models (for backward compatibility)
        '/usr/share/gz/gz-sim8/models',  # New Gazebo models
        '/usr/share/gz/gz-sim7/models',  # Alternative version
    ]
    
    if existing_gz_path:
        resource_paths.append(existing_gz_path)
    
    gz_resource_path = ':'.join(resource_paths)

    return [
        # Set Gazebo resource path environment variable
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=gz_resource_path
        ),
        
        # Set verbose logging for Gazebo
        SetEnvironmentVariable(
            name='GZ_VERBOSE',
            value='4'  # 1=error, 2=warning, 3=info, 4=debug
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': f'-v 4 -r {world_file}',  # -v 4 for verbose debug output
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
