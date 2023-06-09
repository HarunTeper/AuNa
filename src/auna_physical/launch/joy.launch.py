"""Sensor launch file"""

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Return launch description"""

    # Paths to folders and files
    physical_pkg_dir = get_package_share_directory('auna_physical')
    config_file_dir = os.path.join(physical_pkg_dir, 'config')
    joy_config_file_path = os.path.join(config_file_dir, 'ps4.config.yaml')

    # Launch arguments
    namespace_arg = DeclareLaunchArgument('namespace', default_value='robot')
    joy_dev_arg = DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0')
    config_filepath_arg = DeclareLaunchArgument('joy_config_file', default_value=joy_config_file_path)

    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    joy_dev = LaunchConfiguration('joy_dev')
    joy_config_file = LaunchConfiguration('joy_config_file')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace=namespace,
        parameters=[{
            'dev': joy_dev,
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }]
    )

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        namespace=namespace,
        parameters=[joy_config_file]
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(namespace_arg)
    launch_description.add_action(joy_dev_arg)
    launch_description.add_action(config_filepath_arg)

    launch_description.add_action(joy_node)
    launch_description.add_action(teleop_twist_joy_node)

    return launch_description
