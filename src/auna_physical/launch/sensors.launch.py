"""Sensor launch file"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Return launch description"""

    # Paths to folders and files
    auna_physical_pkg_dir = get_package_share_directory('auna_physical')
    auna_physical_launch_file_dir = os.path.join(auna_physical_pkg_dir, 'launch')

    # Launch arguments
    namespace_arg = DeclareLaunchArgument('namespace', default_value='robot')

    # Launch configuration
    namespace = LaunchConfiguration('namespace')

    # Nodes and other launch files
    lidar_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(auna_physical_launch_file_dir, 'lidar_sensor.launch.py')),
        launch_arguments={
            'namespace': namespace
        }.items()
    )

    joy_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(auna_physical_launch_file_dir, 'joy.launch.py')),
        launch_arguments={
            'namespace': namespace
        }.items()
    )

    vesc_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(auna_physical_launch_file_dir, 'vesc.launch.py')),
        launch_arguments={
            'namespace': namespace
        }.items()
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(namespace_arg)
    launch_description.add_action(lidar_launch_file)
    # launch_description.add_action(joy_launch_file)
    launch_description.add_action(vesc_launch_file)

    return launch_description
