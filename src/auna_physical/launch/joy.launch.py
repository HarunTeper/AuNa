"""Sensor launch file"""

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from auna_common import yaml_launch

def generate_launch_description():
    """Return launch description"""

    # Paths to folders and files
    physical_pkg_dir = get_package_share_directory('auna_physical')
    config_file_dir = os.path.join(physical_pkg_dir, 'config')
    joy_config_file_path = os.path.join(config_file_dir, 'teleop_joy.yaml')

    # Launch arguments
    namespace_arg = DeclareLaunchArgument('namespace', default_value='robot')

    # Launch configurations
    namespace = LaunchConfiguration('namespace')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace=namespace,
        parameters=[yaml_launch.get_yaml_value(joy_config_file_path, ['joy_node', 'ros__parameters'])],
    )

    teleop_joy_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        namespace=namespace,
        parameters=[yaml_launch.get_yaml_value(joy_config_file_path, ['joy_teleop', 'ros__parameters'])],
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(namespace_arg)

    launch_description.add_action(joy_node)
    launch_description.add_action(teleop_joy_node)

    return launch_description
