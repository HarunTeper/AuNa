import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from auna_common import yaml_launch


def include_launch_description(context: LaunchContext):
    """Return launch description"""

    # Launch Argument Configurations
    cacc_config = LaunchConfiguration('cacc_config')
    waypoint_file_path = LaunchConfiguration('waypoint_file')
    robot_index = LaunchConfiguration('robot_index')
    
    use_waypoints = os.environ.get('USE_WAYPOINTS', 'true').lower() == 'true'
    
    # Create robot namespace based on index
    if int(robot_index.perform(context)) >= 0:
        namespace = f'robot{robot_index.perform(context)}'
    else:
        namespace = ''

    launch_description_content = []

    parameters = [yaml_launch.get_yaml_value(cacc_config.perform(
        context), ['cacc_controller', 'ros__parameters'])]
    if use_waypoints:
        parameters.append({'waypoint_file': waypoint_file_path})

    launch_description_content.append(PushRosNamespace(namespace))

    launch_description_content.append(
        Node(
            package='auna_cacc',
            executable='cacc_controller',
            name='cacc_controller',
            output='screen',
            parameters=parameters
        )
    )

    return launch_description_content


def generate_launch_description():
    """Return launch description"""

    # Package Directories
    pkg_dir = get_package_share_directory('auna_cacc')

    # Config files
    cacc_config_file_path = os.path.join(
        pkg_dir, 'config', 'cacc_controller.yaml')

    # Waypoint files
    waypoint_file_path = os.path.join(pkg_dir, 'config', 'waypoints.csv')

    # Launch Arguments
    robot_index_arg = DeclareLaunchArgument(
        'robot_index',
        default_value='1',
        description='Robot index'
    )
    cacc_config_arg = DeclareLaunchArgument(
        'cacc_config',
        default_value=cacc_config_file_path,
        description='Path to cacc config file'
    )
    waypoint_file_path_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value=waypoint_file_path,
        description='Path to waypoint file'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(robot_index_arg)
    launch_description.add_action(cacc_config_arg)
    launch_description.add_action(waypoint_file_path_arg)

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
