import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Return launch description"""

    # Package Directories
    gazebo_pkg_dir = get_package_share_directory('auna_gazebo')
    navigation_pkg_dir = get_package_share_directory('auna_nav2')

    # Paths to folders and files
    gazebo_launch_file_dir = os.path.join(gazebo_pkg_dir, 'launch', 'gazebo')
    spawn_launch_file_dir = os.path.join(gazebo_pkg_dir, 'launch', 'spawn')
    nav_launch_file_dir = os.path.join(navigation_pkg_dir, 'launch')

    # Paths to folders and files
    default_rviz_config_file = os.path.join(navigation_pkg_dir, 'rviz', 'config_navigation_namespace.rviz')
    default_params_file = os.path.join(navigation_pkg_dir, 'config', 'nav2_params', 'nav2_params.yaml')
    map_path = os.path.join(navigation_pkg_dir, 'maps', 'arena', 'map.yaml')

    # Launch Argument Configurations
    world_name = LaunchConfiguration('world_name', default='racetrack_decorated')
    rviz_config = LaunchConfiguration('rviz_config', default=default_rviz_config_file)
    world_name = LaunchConfiguration('world_name')

    # Launch Arguments
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='arena',
        description='Gazebo world file name'
    )

    # Nodes and other launch files
    world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_launch_file_dir, 'gazebo_world.launch.py')),
        launch_arguments={
            'world_name': world_name
        }.items(),
    )
    spawn_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(spawn_launch_file_dir, 'spawn_single_robot.launch.py')),
        launch_arguments={
            'world_name': world_name,
            'namespace': 'robot',
            'ground_truth': 'False'
        }.items(),
    )
    nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_launch_file_dir, 'navigation_single_robot.launch.py')),
        launch_arguments={
            'namespace': 'robot',
            'rviz_config': rviz_config,
            'map': map_path,
            'params_file': default_params_file,
            'enable_slam': 'False',  # slam can only be used without a namespace
            'enable_localization': 'True',
            'enable_navigation': 'True',
            'enable_rviz': 'True',
            'enable_map_server': 'True',
        }.items(),
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(world_name_arg)

    launch_description.add_action(world_cmd)
    launch_description.add_action(spawn_cmd)
    launch_description.add_action(nav_cmd)

    return launch_description
