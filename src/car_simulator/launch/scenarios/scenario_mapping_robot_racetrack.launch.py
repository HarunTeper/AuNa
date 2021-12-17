import os
from typing import Text
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,TextSubstitution

def generate_launch_description():

    # Package Directories
    pkg_dir = get_package_share_directory('car_simulator')

    # Paths to folders and files
    gazebo_launch_file_dir = os.path.join(pkg_dir, 'launch', 'gazebo')
    nav_launch_file_dir = os.path.join(pkg_dir, 'launch', 'navigation')
    spawn_launch_file_dir = os.path.join(pkg_dir, 'launch', 'spawn')
    default_params_file = os.path.join(pkg_dir, 'config', 'scenario_params', 'mapping_robot.yaml')
    default_rviz_config_file = os.path.join(pkg_dir, 'rviz','config_mapping.rviz')

    # Launch Argument Configurations
    params_file = LaunchConfiguration('params_file', default = default_params_file)
    rviz_config = LaunchConfiguration('rviz_config', default = default_rviz_config_file)
    slam = LaunchConfiguration('slam', default = 'True')
    use_namespace = LaunchConfiguration('use_namespace', default='false')
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
        PythonLaunchDescriptionSource(os.path.join(spawn_launch_file_dir, 'spawn_single_robot.launch.py')),
        launch_arguments={
            'name':'robot',
            'namespace':'',
            'urdf_namespace':'',
        }.items(),
    )
    nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_launch_file_dir, 'navigation_single_robot.launch.py')),
        launch_arguments={
            'namespace': '',
            'params_file': params_file,
            'rviz_config':rviz_config,
            'slam': slam,
            'use_namespace': use_namespace,
            'world_name': world_name
        }.items(),
    )
    
    # Launch Description
    ld = LaunchDescription()

    ld.add_action(robot_number_arg)
    ld.add_action(world_name_arg)

    ld.add_action(world_cmd)
    ld.add_action(spawn_cmd)
    ld.add_action(nav_cmd)

    return ld
