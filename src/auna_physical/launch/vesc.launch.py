"""Sensor launch file"""

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext

def include_launch_description(context: LaunchContext):
    """Return launch description"""

    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    vesc_config = LaunchConfiguration('vesc_config')

    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]

    # Nodes and other launch files
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        namespace=namespace,
        parameters=[vesc_config]
    )

    if namespace.perform(context) == "":
        vesc_to_odom_node = Node(
            package='vesc_ackermann',
            executable='vesc_to_odom_node',
            name='vesc_to_odom_node',
            namespace=namespace,
            parameters=[
                {"odom_frame": "odom"},
                {"base_frame": "base_link"},
                {"speed_to_erpm_gain": 4614.0},
                {"speed_to_erpm_offset": 0.0},
                {"use_servo_cmd_to_calc_angular_velocity": True},
                {"steering_angle_to_servo_gain": -1.0},
                {"steering_angle_to_servo_offset": 0.530},
                {"wheelbase": 0.32},
                {"publish_tf": True}
            ],
            output='screen',
            remappings=remappings
        )
    else:
        vesc_to_odom_node = Node(
            package='vesc_ackermann',
            executable='vesc_to_odom_node',
            name='vesc_to_odom_node',
            namespace=namespace,
            parameters=[
                {"odom_frame": namespace.perform(context)+"/odom"},
                {"base_frame": namespace.perform(context)+"/base_link"},
                {"speed_to_erpm_gain": 4614.0},
                {"speed_to_erpm_offset": 0.0},
                {"use_servo_cmd_to_calc_angular_velocity": True},
                {"steering_angle_to_servo_gain": -1.0},
                {"steering_angle_to_servo_offset": 0.530},
                {"wheelbase": 0.32},
                {"publish_tf": True}
            ],
            output='screen',
            remappings=remappings
        )

    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        namespace=namespace,
        parameters=[
            {"speed_to_erpm_gain": 4614.0},
            {"speed_to_erpm_offset": 0.0},
            {"steering_angle_to_servo_gain": -1.0},
            {"steering_angle_to_servo_offset": 0.530}
        ],
        output='screen'
    )

    launch_description_content = []
    launch_description_content.append(vesc_driver_node)
    launch_description_content.append(vesc_to_odom_node)
    launch_description_content.append(ackermann_to_vesc_node)

    return launch_description_content


def generate_launch_description():
    """Return launch description"""

    # Package Directories
    pkg_dir = get_package_share_directory('auna_physical')

    # Config files
    vesc_config = os.path.join(pkg_dir,'config','vesc.config.yaml')

    # Launch arguments
    namespace_arg = DeclareLaunchArgument('namespace', default_value='robot')
    vesc_config_arg = DeclareLaunchArgument('vesc_config', default_value=vesc_config)


    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(namespace_arg)
    launch_description.add_action(vesc_config_arg)

    launch_description.add_action(OpaqueFunction(function=include_launch_description))

    return launch_description