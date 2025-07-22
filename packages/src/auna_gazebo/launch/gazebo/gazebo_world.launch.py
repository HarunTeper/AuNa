"""Gazebo world launch file"""
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_launch_description(context: LaunchContext):
    """Return launch description"""
    # Package Directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_auna_gazebo = get_package_share_directory('auna_gazebo')

    # Get the environment variable WORLD_NAME, fallback to 'default' if not set
    world_name = os.environ.get('WORLD_NAME', 'racetrack_decorated')
    # Construct world path using resolved context
    world = os.path.join(pkg_auna_gazebo, 'worlds', world_name +'.world')
    
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': world,
                'gazebo_ros_state': 'True',
            }.items()
        ),

        Node(
            package='auna_gazebo',
            executable='robot_name_publisher',
            name='robot_name_publisher',
            output='screen'
        )
    ]


def generate_launch_description():
    """Return launch description"""
    return LaunchDescription([
        OpaqueFunction(function=include_launch_description)
    ])
