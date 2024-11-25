"""Sensor launch file"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch_ros.actions import Node
from auna_common import yaml_launch


def include_launch_description(context: LaunchContext):
    """Return launch description"""
    
    # Launch configuration
    namespace = LaunchConfiguration('namespace')

    # Paths to folders and files
    robot_localization_pkg_dir = get_package_share_directory('auna_physical')
    config_file_dir = os.path.join(robot_localization_pkg_dir, 'config')
    config_file = os.path.join(config_file_dir, 'ukf.yaml')

    tmp_params_file = yaml_launch.get_yaml(config_file)
    tmp_params_file = yaml_launch.insert_namespace(tmp_params_file, context.launch_configurations['namespace'])
    tmp_params_file = yaml_launch.get_temp_file(tmp_params_file)

    robot_localization_node = Node(
        package='robot_localization',
        executable='ukf_node',
        name='ukf_filter_node',
        output='screen',
        namespace=namespace,
        parameters=[yaml_launch.get_yaml_value(tmp_params_file, ['ukf_filter_node', 'ros__parameters'])],
        # remapping for tf and tf_static
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')
                    ]
    )

    launch_description_content = []
    launch_description_content.append(robot_localization_node)

    return launch_description_content


def generate_launch_description():
    """Return launch description"""

    # Launch arguments
    namespace_arg = DeclareLaunchArgument('namespace', default_value='robot')

    # Nodes and other launch files

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(namespace_arg)

    launch_description.add_action(OpaqueFunction(function=include_launch_description))

    return launch_description
