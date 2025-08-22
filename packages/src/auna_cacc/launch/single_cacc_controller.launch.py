import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from auna_common import yaml_launch


def include_launch_description(context: LaunchContext):
    """Return launch description"""

    # Launch Argument Configurations
    cacc_config = LaunchConfiguration('cacc_config')
    waypoint_file_path = LaunchConfiguration('waypoint_file')
    
    use_waypoints = os.environ.get('USE_WAYPOINTS', 'true').lower() == 'true'
    robot_index = int(os.environ.get('ROBOT_INDEX', '0'))

    # Create robot namespace based on index
    if int(robot_index) >= 0:
        namespace = f'robot{robot_index}'
    else:
        namespace = ''

    launch_description_content = []

    parameters = [yaml_launch.get_yaml_value(cacc_config.perform(
        context), ['cacc_controller', 'ros__parameters'])]
    launch_description_content.append(PushRosNamespace(namespace))

    # CACC Controller Node
    cacc_controller_node = Node(
        package='auna_cacc',
        executable='cacc_controller',
        name='cacc_controller',
        output='screen',
        parameters=parameters,
    )

    launch_description_content.append(cacc_controller_node)

    return launch_description_content


def generate_launch_description():
    """Return launch description"""

    # Package Directories
    pkg_dir = get_package_share_directory('auna_cacc')

    # Config files
    cacc_config_file_path = os.path.join(
        pkg_dir, 'config', 'cacc_controller.yaml')

    # Launch Arguments
    cacc_config_arg = DeclareLaunchArgument(
        'cacc_config',
        default_value=cacc_config_file_path,
        description='Path to cacc config file'
    )

    use_waypoints_arg = DeclareLaunchArgument(
        'use_waypoints',
        default_value='true',
        description='Use waypoints'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(cacc_config_arg)
    launch_description.add_action(use_waypoints_arg)

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
