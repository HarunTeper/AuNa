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
    omnet_pkg_dir = get_package_share_directory('auna_omnet')

    # Paths to folders and files
    gazebo_launch_file_dir = os.path.join(gazebo_pkg_dir, 'launch', 'gazebo')
    spawn_launch_file_dir = os.path.join(gazebo_pkg_dir, 'launch', 'spawn')
    nav_launch_file_dir = os.path.join(navigation_pkg_dir, 'launch')
    omnet_launch_file_dir = os.path.join(omnet_pkg_dir, 'launch')

    # Launch Argument Configurations
    robot_number = LaunchConfiguration('robot_number', default='2')
    world_name = LaunchConfiguration('world_name', default='racetrack_decorated')

    # Launch Arguments
    robot_number_arg = DeclareLaunchArgument(
        'robot_number',
        default_value='2',
        description='Number of spawned robots'
    )
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='racetrack_decorated',
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
        PythonLaunchDescriptionSource(os.path.join(spawn_launch_file_dir, 'spawn_multi_robot.launch.py')),
        launch_arguments={
            'robot_number': robot_number,
            'world_name': world_name,
            'ground_truth': 'False'
        }.items(),
    )
    omnet_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(omnet_launch_file_dir, 'omnet_multi_robot_modules.launch.py')),
        launch_arguments={
            'robot_number': robot_number,
        }.items(),
    )
    nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_launch_file_dir, 'navigation_multi_robot.launch.py')),
        launch_arguments={
            'robot_number': robot_number,
            'world_name': world_name,
            'enable_slam': 'False',  # slam can only be used without a namespace
            'enable_localization': 'True',
            'enable_navigation': 'True',
            'enable_rviz': 'True',
            'enable_map_server': 'True',
        }.items(),
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(robot_number_arg)
    launch_description.add_action(world_name_arg)

    launch_description.add_action(world_cmd)
    launch_description.add_action(spawn_cmd)
    launch_description.add_action(omnet_cmd)
    launch_description.add_action(nav_cmd)

    return launch_description
