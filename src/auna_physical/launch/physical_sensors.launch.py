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
    vesc_pkg_dir = get_package_share_directory('vesc_driver')
    lidar_pkg_dir = get_package_share_directory('urg_node2')
    joy_pkg_dir = get_package_share_directory('teleop_twist_joy')
    physical_pkg_dir = get_package_share_directory('auna_physical')

    vesc_config = os.path.join(vesc_pkg_dir,'params','vesc_config.yaml')
    lidar_launch_file_dir = os.path.join(lidar_pkg_dir, 'launch')
    joy_launch_file_dir = os.path.join(joy_pkg_dir, 'launch')
    config_file_dir = os.path.join(physical_pkg_dir, 'config')

    joy_config_file_path = os.path.join(config_file_dir, 'ps3.config.yaml')

    # Nodes and other launch files
    launch_description_content = []

    # Nodes and other launch files
    launch_description_content.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(lidar_launch_file_dir, 'urg_node2.launch.py')),
        )
    )

    launch_description_content.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(joy_launch_file_dir, 'teleop-launch.py')),
            launch_arguments={
                'config_filepath': joy_config_file_path
            }.items(),
        )
    )

    launch_description_content.append(
        Node(
            package='auna_physical',
            executable='cmd_vel_to_ackermann',
            name='cmd_vel_to_ackermann',
            namespace="robot"
        ),
    )

    launch_description_content.append(
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            namespace="robot",
            parameters=[vesc_config]
        ),
    )

    launch_description_content.append(
        Node(
            package='vesc_ackermann',
            executable='vesc_to_odom_node',
            name='vesc_to_odom_node',
            namespace="robot",
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
    )

    launch_description_content.append(
        Node(
            package='vesc_ackermann',
            executable='ackermann_to_vesc_node',
            name='ackermann_to_vesc_node',
            namespace="robot",
            parameters=[
                {"speed_to_erpm_gain": 4614.0},
                {"speed_to_erpm_offset": 0.0},
                {"steering_angle_to_servo_gain": -1.2135},
                {"steering_angle_to_servo_offset": 0.530}
            ],
            output='screen'
        )
    )

    return launch_description_content


def generate_launch_description():
    """Return launch description"""

    # Launch Arguments

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(OpaqueFunction(function=include_launch_description))

    return launch_description
