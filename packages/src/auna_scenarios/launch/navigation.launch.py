import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Return launch description"""

    # Package Directories
    scenarios_pkg_dir = get_package_share_directory('auna_scenarios')

    # Paths to folders and files
    scenarios_launch_file_dir = os.path.join(scenarios_pkg_dir, 'launch')

    # Launch Arguments
    map_type_arg = DeclareLaunchArgument(
        'map_type',
        default_value='racetrack_decorated',
        description='Map type to use'
    )
    robot_number_arg = DeclareLaunchArgument(
        'robot_number',
        default_value='2',
        description='Number of spawned robots'
    )
    nav2_arg = DeclareLaunchArgument(
        'nav2',
        default_value='true',
        description='Enable Nav2 for the leader robot'
    )
    communication_arg = DeclareLaunchArgument(
        'communication',
        default_value='auna_comm',
        description='Communication system to use (omnet or auna_comm)'
    )

    # Nodes and other launch files
    scenario_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            scenarios_launch_file_dir, 'scenario.launch.py')),
        launch_arguments={
            'map_type': LaunchConfiguration('map_type'),
            'robot_number': LaunchConfiguration('robot_number'),
            'nav2': LaunchConfiguration('nav2'),
            'communication': LaunchConfiguration('communication'),
            'cacc': 'false',
            'cacc_waypoints': 'false'
        }.items(),
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(map_type_arg)
    launch_description.add_action(robot_number_arg)
    launch_description.add_action(nav2_arg)
    launch_description.add_action(communication_arg)
    launch_description.add_action(scenario_cmd)

    return launch_description
