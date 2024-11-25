"""Multiple car spawn launch file"""

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from auna_common import yaml_launch


def include_launch_description(context: LaunchContext):
    """Return launch description"""

    # Package Directories
    pkg_dir = get_package_share_directory('auna_gazebo')

    # Paths to folders and files
    launch_file_dir = os.path.join(pkg_dir, 'launch', 'spawn')

    # Launch Argument Configurations
    robot_number = LaunchConfiguration('robot_number')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world_name')
    ground_truth = LaunchConfiguration('ground_truth')

    # Names and poses of the robots
    map_path = os.path.join(pkg_dir, "config", "map_params", world_name.perform(context)+".yaml")
    robots = []
    for num in range(int(robot_number.perform(context))):
        robots.append({
            'name': 'robot'+str(num),
            'namespace': 'robot'+str(num),
            'x_pose': yaml_launch.get_yaml_value(map_path, ["spawn", "offset", "x"])+num*yaml_launch.get_yaml_value(map_path, ["spawn", "linear", "x"]),
            'y_pose': yaml_launch.get_yaml_value(map_path, ["spawn", "offset", "y"])+num*yaml_launch.get_yaml_value(map_path, ["spawn", "linear", "y"]),
            'z_pose': yaml_launch.get_yaml_value(map_path, ["spawn", "offset", "z"])+num*yaml_launch.get_yaml_value(map_path, ["spawn", "linear", "z"]),
            }
        )

    # Nodes and other launch files
    launch_description_content = []

    for robot in robots:
        launch_description_content.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'spawn_single_robot.launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                    'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                    'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                    'namespace': robot['namespace'],
                    'name': robot['name'],
                    'ground_truth': ground_truth,
                }.items()
            )
        )

    launch_description_content.append(
        Node(
            package='auna_gazebo',
            executable='global_tf',
            name='global_tf',
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
        description='Whether to use gazebo_pose as ground truth localization'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(robot_number_arg)
    launch_description.add_action(use_sim_time_arg)
    launch_description.add_action(world_name_arg)
    launch_description.add_action(ground_truth_arg)

    launch_description.add_action(OpaqueFunction(function=include_launch_description))

    return launch_description
