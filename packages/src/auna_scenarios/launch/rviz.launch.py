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
    default_rviz_config_file = os.path.join(
        navigation_pkg_dir, 'rviz', 'empty_config.rviz')
    default_params_file = os.path.join(
        navigation_pkg_dir, 'config', 'nav2_params', 'nav2_params.yaml')
    map_path = os.path.join(navigation_pkg_dir, 'maps', 'arena', 'map.yaml')

    # Launch Argument Configurations
    world_name = LaunchConfiguration('world_name', default='arena')
    robot_number = LaunchConfiguration('robot_number', default='2')
    namespace = LaunchConfiguration('namespace', default='robot')
    rviz_config = LaunchConfiguration(
        'rviz_config', default=default_rviz_config_file)

    # Launch Arguments
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='arena',
        description='Gazebo world file name'
    )

    robot_number_arg = DeclareLaunchArgument(
        'robot_number',
        default_value='2',
        description='Number of spawned robots'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot',
        description='Namespace of the robot'
    )

    # Nodes and other launch files
    nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            nav_launch_file_dir, 'navigation_multi_robot.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'robot_number': robot_number,
            'world_name': world_name,
            'enable_slam': 'False',
            'enable_localization': 'False',
            'enable_navigation': 'False',
            'enable_rviz': 'True',
            'enable_map_server': 'False',
        }.items(),
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(world_name_arg)
    launch_description.add_action(robot_number_arg)
    launch_description.add_action(namespace_arg)
    launch_description.add_action(nav_cmd)

    return launch_description
