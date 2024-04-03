"""Sensor launch file"""

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from auna_common import yaml_launch

def generate_launch_description():
    """Return launch description"""

    # Paths to folders and files
    physical_pkg_dir = get_package_share_directory('auna_physical')
    config_file_dir = os.path.join(physical_pkg_dir, 'config')
    joy_config_file_path = os.path.join(config_file_dir, 'teleop_joy.yaml')

    # Launch arguments
    namespace_arg = DeclareLaunchArgument('namespace', default_value='robot')
    use_ps4_arg = DeclareLaunchArgument('use_ps4', default_value='false')
    use_g29_arg = DeclareLaunchArgument('use_g29', default_value='false')

    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    use_ps4 = LaunchConfiguration('use_ps4')
    use_g29 = LaunchConfiguration('use_g29')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace=namespace,
        parameters=[yaml_launch.get_yaml_value(joy_config_file_path, ['joy_node', 'ros__parameters'])],
    )

    teleop_joy_node_ps4 = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        namespace=namespace,
        parameters=[yaml_launch.get_yaml_value(joy_config_file_path, ['joy_teleop_ps4', 'ros__parameters'])],
        condition=IfCondition(use_ps4),
    )

    teleop_joy_node_g29 = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        namespace=namespace,
        parameters=[yaml_launch.get_yaml_value(joy_config_file_path, ['joy_teleop_g29', 'ros__parameters'])],
        condition=IfCondition(use_g29),
    )



    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(namespace_arg)
    launch_description.add_action(use_ps4_arg)
    launch_description.add_action(use_g29_arg)

    launch_description.add_action(joy_node)
    launch_description.add_action(teleop_joy_node_ps4)
    launch_description.add_action(teleop_joy_node_g29)

    return launch_description
