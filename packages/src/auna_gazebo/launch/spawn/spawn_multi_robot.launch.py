"""Multiple robot spawn launch file"""  # Updated terminology from 'car' to 'robot'

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, GroupAction, PushRosNamespace
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from auna_common import yaml_launch


def generate_launch_description():
    """Return launch description"""
    # Package Directories
    pkg_dir = get_package_share_directory('auna_gazebo')
    launch_file_dir = os.path.join(pkg_dir, 'launch', 'spawn')

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
        description='Whether to use ground_truth_localization'
    )

    # Launch Configurations
    robot_number = LaunchConfiguration('robot_number')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world_name')
    ground_truth = LaunchConfiguration('ground_truth')

    def get_robot_configs(context):
        """Get robot configurations from yaml file"""
        map_path = os.path.join(pkg_dir, "config", "map_params",
                                world_name.perform(context)+".yaml")
        robots = []
        for num in range(int(robot_number.perform(context))):
            robots.append({
                'name': f'robot{num}',
                'namespace': f'robot{num}',
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
        return robots

    def get_robot_spawns(context):
        """Generate robot spawn descriptions"""
        robots = get_robot_configs(context)
        spawn_descriptions = []

        for robot in robots:
            spawn_descriptions.append(
                GroupAction([
                    PushRosNamespace(robot['namespace']),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(launch_file_dir,
                                         'robot_state_publisher.launch.py')
                        ),
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
                ])
            )

        # Add global TF node
        spawn_descriptions.append(
            Node(
                package='auna_gazebo',
                executable='global_tf',
                name='global_tf',
                output='screen'
            )
        )

        return spawn_descriptions

    return LaunchDescription([
        # Launch arguments
        robot_number_arg,
        use_sim_time_arg,
        world_name_arg,
        ground_truth_arg,
        # Robot spawns and global TF
        *get_robot_spawns
    ])
