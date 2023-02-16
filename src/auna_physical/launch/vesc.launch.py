"""Sensor launch file"""

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Return launch description"""

    # Package Directories
    pkg_dir = get_package_share_directory('auna_physical')

    # Paths to folders and files
    physical_pkg_dir = get_package_share_directory('auna_physical')
    config_file_dir = os.path.join(physical_pkg_dir, 'config')
    joy_config_file_path = os.path.join(config_file_dir, 'ps3.config.yaml')

    # Config files
    vesc_config = os.path.join(pkg_dir,'config','vesc.config.yaml')

    # Launch arguments

    namespace_arg = DeclareLaunchArgument('namespace', default_value='robot')
    vesc_config_arg = DeclareLaunchArgument('vesc_config', default_value=vesc_config)
    joy_config_arg = DeclareLaunchArgument('joy_config', default_value='ps3')
    joy_dev_arg = DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0')
    config_filepath_arg = DeclareLaunchArgument('joy_config', default_value=joy_config_file_path)

    # Launch configurations

    namespace = LaunchConfiguration('namespace')
    joy_dev = LaunchConfiguration('joy_dev')
    joy_config = LaunchConfiguration('vesc_config')
    vesc_config = LaunchConfiguration('vesc_config')

    # Nodes and other launch files
    cmd_vel_to_ackermann_node = Node(
        package='auna_physical',
        executable='cmd_vel_to_ackermann',
        name='cmd_vel_to_ackermann',
        namespace=namespace
    )

    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        namespace=namespace,
        parameters=[vesc_config]
    )

    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        namespace=namespace,
        parameters=[
            {"odom_frame": "odom"},
            {"base_frame": "base_link"},
            {"speed_to_erpm_gain": 1.0},
            {"speed_to_erpm_offset": 0.0},
            {"use_servo_cmd_to_calc_angular_velocity": True},
            {"steering_angle_to_servo_gain": 1.0},
            {"steering_angle_to_servo_offset": 0.0},
            {"wheelbase": 0.32},
            {"publish_tf": True}
        ],
        output='screen'
    )

    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        namespace=namespace,
        parameters=[
            {"speed_to_erpm_gain": 4614.0},
            {"speed_to_erpm_offset": 0.0},
            {"steering_angle_to_servo_gain": -1.2135},
            {"steering_angle_to_servo_offset": 0.530}
        ],
        output='screen'
    )

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
        parameters=[joy_config]
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(namespace_arg)
    launch_description.add_action(vesc_config_arg)
    launch_description.add_action(joy_config_arg)
    launch_description.add_action(joy_dev_arg)
    launch_description.add_action(config_filepath_arg)

    launch_description.add_action(cmd_vel_to_ackermann_node)
    launch_description.add_action(vesc_driver_node)
    launch_description.add_action(vesc_to_odom_node)
    launch_description.add_action(ackermann_to_vesc_node)
    launch_description.add_action(joy_node)
    launch_description.add_action(teleop_twist_joy_node)

    return launch_description
