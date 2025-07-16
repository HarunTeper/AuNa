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

    # Construct world path using resolved context
    world = os.path.join(pkg_auna_gazebo, 'worlds', str(
        context.launch_configurations['world_name'])+'.world')

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': world,
                'gazebo_ros_state': context.launch_configurations['gazebo_ros_state'],
                'gui': context.launch_configurations.get('gui', 'true'),
                'server': 'true',
                'verbose': 'true',  # Reduce verbosity for faster startup
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

    world_arg = DeclareLaunchArgument(
        'world_name',
        default_value='racetrack_decorated',
        description='Gazebo world file name in /worlds folder'
    )

    gazebo_state_arg = DeclareLaunchArgument(
        'gazebo_ros_state',
        default_value='true',
        description='Enable/disable Gazebo ROS state'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Enable/disable Gazebo GUI'
    )

    return LaunchDescription([
        world_arg,
        gazebo_state_arg,
        gui_arg,
        OpaqueFunction(function=include_launch_description)
    ])
