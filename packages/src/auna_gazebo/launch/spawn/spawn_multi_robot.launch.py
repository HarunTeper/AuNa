"""Multiple robot spawn launch file"""  # Renamed from 'car' to 'robot' per TODO

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, GroupAction
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from auna_common import yaml_launch


def include_launch_description(context: LaunchContext):
    """Return launch description"""
    # Package Directories
    pkg_dir = get_package_share_directory('auna_gazebo')
    launch_file_dir = os.path.join(pkg_dir, 'launch', 'spawn')
    robot_description_file_dir = os.path.join(
        pkg_dir, 'launch', 'robot_description')

    # Get configurations from context
    robot_number = int(context.launch_configurations['robot_number'])
    use_sim_time = context.launch_configurations['use_sim_time']
    world_name = context.launch_configurations['world_name']
    ground_truth = context.launch_configurations['ground_truth']

    # Get robot configurations from YAML
    map_path = os.path.join(
        pkg_dir, "config", "map_params", f"{world_name}.yaml")

    # Create robot configurations
    robots = []
    for num in range(robot_number):
        namespace = f'robot{num}'
        robots.append({
            'name': namespace,
            'namespace': namespace,
            'x_pose': yaml_launch.get_yaml_value(map_path, ["spawn", "offset", "x"]) +
            num *
            yaml_launch.get_yaml_value(map_path, ["spawn", "linear", "x"]),
            'y_pose': yaml_launch.get_yaml_value(map_path, ["spawn", "offset", "y"]) +
            num *
            yaml_launch.get_yaml_value(map_path, ["spawn", "linear", "y"]),
            'z_pose': yaml_launch.get_yaml_value(map_path, ["spawn", "offset", "z"]) +
            num *
            yaml_launch.get_yaml_value(map_path, ["spawn", "linear", "z"]),
        })

    # Create launch actions for each robot
    launch_actions = []

    # Spawn each robot with its own namespace group
    for robot in robots:
        robot_group = GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    # Renamed from spawn_single_robot
                    os.path.join(launch_file_dir,
                                 'spawn_single_robot.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'x_pose': str(robot['x_pose']),
                    'y_pose': str(robot['y_pose']),
                    'z_pose': str(robot['z_pose']),
                    'namespace': robot['namespace'],
                    'name': robot['name'],
                    'ground_truth': ground_truth,
                }.items()
            )
        ])
        launch_actions.append(robot_group)

    # Add global TF node
    launch_actions.append(
        Node(
            package='auna_gazebo',
            executable='global_tf',
            name='global_tf',
            output='screen'
        )
    )

    return launch_actions


def generate_launch_description():
    """Return launch description"""
    # Launch Arguments
    robot_number_arg = DeclareLaunchArgument(
        'robot_number',
        default_value='2',
        description='Number of spawned robots'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='racetrack_decorated',
        description='Gazebo world file name in /worlds folder'
    )

    ground_truth_arg = DeclareLaunchArgument(
        'ground_truth',
        default_value='False',
        description='Whether to use ground_truth_localization for localization'
    )

    return LaunchDescription([
        # Launch arguments
        robot_number_arg,
        use_sim_time_arg,
        world_name_arg,
        ground_truth_arg,
        # Launch description generation
        OpaqueFunction(function=include_launch_description)
    ])
