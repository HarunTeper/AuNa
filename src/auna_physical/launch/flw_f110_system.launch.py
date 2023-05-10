import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Return launch description"""

    # Package Directories
    pkg_dir = get_package_share_directory('auna_physical')

    # Paths to folders and files
    launch_file_dir = os.path.join(pkg_dir, 'launch')

    # Launch arguments
    namespace_arg = DeclareLaunchArgument('namespace', default_value='robot')

    # Launch Configuration
    namespace = LaunchConfiguration('namespace')
    
    sensor_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'sensors.launch.py')),
        launch_arguments={
            'namespace': namespace,
        }.items(),
    )
    nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'f110_navigation.launch.py')),
        launch_arguments={
            'namespace': namespace,
        }.items(),
    )
    vicon_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'vicon_tf_converter.launch.py')),
        launch_arguments={
            'namespace': namespace,
        }.items(),
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(namespace_arg)
    launch_description.add_action(nav_cmd)
    launch_description.add_action(sensor_cmd)
    launch_description.add_action(vicon_cmd)

    return launch_description
