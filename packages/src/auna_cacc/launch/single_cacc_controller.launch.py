import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from auna_common import yaml_launch


def include_launch_description(context: LaunchContext):
    """Return launch description (using environment variables only)."""

    cacc_config = LaunchConfiguration('cacc_config')
    robot_index = int(os.environ.get('ROBOT_INDEX', '1'))

    # Create robot namespace based on index
    if int(robot_index) >= 0:
        namespace = f'robot{robot_index}'
    else:
        namespace = ''

    launch_description_content = []

    param_dict = yaml_launch.get_yaml_value(
        cacc_config.perform(context), ['cacc_controller', 'ros__parameters'])

    use_waypoints_env = os.environ.get('USE_WAYPOINTS')
    if use_waypoints_env is not None:
        use_waypoints_bool = use_waypoints_env.lower() == 'true'
        param_dict['use_waypoints'] = use_waypoints_bool

    parameters = [param_dict]
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

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(cacc_config_arg)

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
