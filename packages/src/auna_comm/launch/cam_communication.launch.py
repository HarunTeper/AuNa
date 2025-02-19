"""Single car omnet module launch file"""

from launch_ros.actions import Node, PushRosNamespace
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterFile
import os


def generate_launch_description():
    """Return launch description"""

    # Get the default config file path
    default_config = os.path.join(
        get_package_share_directory('auna_comm'),
        'config',
        'cam_params.yaml'
    )

    # Launch Argument Configurations
    config_file = LaunchConfiguration('config_file')
    robot_index = LaunchConfiguration('robot_index')
    filter_index = LaunchConfiguration('filter_index')
    log_level = LaunchConfiguration('log_level')

    # Launch Arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the config file'
    )
    robot_index_arg = DeclareLaunchArgument(
        'robot_index',
        default_value='0',
        description='Robot identifier index'
    )
    filter_index_arg = DeclareLaunchArgument(
        'filter_index',
        default_value='0',
        description='Robot cam filter index'
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='debug',
        description='Logging level'
    )

    cam_communication_cmd = Node(
        package='auna_comm',
        executable='cam_communication',
        name='cam_communication',
        parameters=[
            config_file,
            {'filter_index': filter_index},
            {'robot_index': robot_index}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen'
    )

    # Remove GroupAction and return Node directly
    launch_description = LaunchDescription()
    launch_description.add_action(config_file_arg)
    launch_description.add_action(robot_index_arg)
    launch_description.add_action(filter_index_arg)
    launch_description.add_action(log_level_arg)
    launch_description.add_action(cam_communication_cmd)

    return launch_description
