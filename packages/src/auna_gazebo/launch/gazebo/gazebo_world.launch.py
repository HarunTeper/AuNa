"""Gazebo world launch file"""
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Return launch description"""

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_auna_gazebo = get_package_share_directory('auna_gazebo')

    world_arg = DeclareLaunchArgument(
        'world_name',
        default_value='racetrack_decorated',
        description='Gazebo world file name in /worlds folder'
    )

    world_name = LaunchConfiguration('world_name')
    world_path = os.path.join(pkg_auna_gazebo, 'worlds', world_name) + '.world'
    state = LaunchConfiguration('gazebo_ros_state', default='true')

    return LaunchDescription([
        world_arg,
        # Gazebo launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': world_path,
                'gazebo_ros_state': state,
            }.items()
        ),
        Node(
            package='auna_gazebo',
            executable='robot_name_publisher',
            name='robot_name_publisher',
            output='screen'
        )
    ])
