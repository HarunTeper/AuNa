"""Sensor launch file"""

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from auna_common import yaml_launch


def include_launch_description(context: LaunchContext):
    """Return launch description"""

    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    vesc_config = LaunchConfiguration('vesc_config')

    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]

    tmp_params_file = yaml_launch.get_yaml(vesc_config.perform(context))
    tmp_params_file = yaml_launch.insert_namespace(tmp_params_file, context.launch_configurations['namespace'])
    tmp_params_file = yaml_launch.get_temp_file(tmp_params_file)

    # Nodes and other launch files
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        namespace=namespace,
        parameters=[yaml_launch.get_yaml_value(tmp_params_file, ['vesc_driver_node', 'ros__parameters'])],
        output='screen'
    )

    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        namespace=namespace,
        parameters=[yaml_launch.get_yaml_value(tmp_params_file, ['vesc_to_odom_node', 'ros__parameters'])],
        output='screen',
        remappings=remappings
    )

    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        namespace=namespace,
        parameters=[yaml_launch.get_yaml_value(tmp_params_file, ['ackermann_to_vesc_node', 'ros__parameters'])],
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
    vesc_config = os.path.join(pkg_dir, 'config', 'vesc.config.yaml')

    # Launch arguments
    namespace_arg = DeclareLaunchArgument('namespace', default_value='robot')
    vesc_config_arg = DeclareLaunchArgument('vesc_config', default_value=vesc_config)


    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(namespace_arg)
    launch_description.add_action(vesc_config_arg)

    launch_description.add_action(OpaqueFunction(function=include_launch_description))

    return launch_description