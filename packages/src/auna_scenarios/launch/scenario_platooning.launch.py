import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_launch_description(context: LaunchContext):
    """Return launch description"""

    # Package Directories
    scenario_pkg_dir = get_package_share_directory('auna_scenarios')
    cacc_pkg_dir = get_package_share_directory('auna_cacc')

    # Paths to folders and files
    scenario_launch_file_dir = os.path.join(scenario_pkg_dir, 'launch')
    cacc_launch_file_dir = os.path.join(cacc_pkg_dir, 'launch')

    # Launch Argument Configurations
    robot_number = LaunchConfiguration('robot_number', default='2')
    world_name = LaunchConfiguration('world_name', default='racetrack_decorated')

    # Nodes and other launch files
    launch_description_content = []

    # Nodes and other launch files
    launch_description_content.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(scenario_launch_file_dir, 'scenario_omnet_multi_robot_racetrack.launch.py')),
            launch_arguments={
                'robot_number': robot_number,
                'world_name': world_name
            }.items(),
        )
    )

    launch_description_content.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(cacc_launch_file_dir, 'cacc_controller.launch.py')),
            launch_arguments={
                'robot_number': robot_number
            }.items(),
        )
    )

    for num in range(int(robot_number.perform(context))-1):
        launch_description_content.append(
            Node(
                package='auna_omnet',
                executable='omnet_cam_filter',
                name='omnet_cam_filter',
                namespace="robot"+str(num+1),
                arguments={str(num)},
                output='screen'
            )
        )

    return launch_description_content


def generate_launch_description():
    """Return launch description"""

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

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(robot_number_arg)
    launch_description.add_action(world_name_arg)

    launch_description.add_action(OpaqueFunction(function=include_launch_description))

    return launch_description
