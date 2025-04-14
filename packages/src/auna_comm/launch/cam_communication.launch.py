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

    # Launch Argument Configurations
    robot_index = LaunchConfiguration('robot_index')
    filter_index = LaunchConfiguration('filter_index')
    log_level = LaunchConfiguration('log_level')
    enable_cam_logging = LaunchConfiguration('enable_cam_logging')  # Added
    cam_log_file_path = LaunchConfiguration('cam_log_file_path')  # Added
    # Launch Arguments
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
        default_value='info',
        description='Logging level'
    )
    # Added declaration for enable_cam_logging
    enable_cam_logging_arg = DeclareLaunchArgument(
        'enable_cam_logging',
        default_value='false',  # Default value if launched standalone
        description='Enable CAM message logging'
    )
    # Added declaration for cam_log_file_path
    cam_log_file_path_arg = DeclareLaunchArgument(
        'cam_log_file_path',
        # Default value if launched standalone
        default_value='/home/vscode/workspace/cam_messages.log',
        description='Path for CAM message log file'
    )

    cam_communication_cmd = Node(
        package='auna_comm',
        executable='cam_communication',
        name='cam_communication',
        parameters=[
            {'filter_index': filter_index},
            {'robot_index': robot_index},
            # Pass logging parameters from LaunchConfigurations to the C++ node
            {'enable_cam_logging': enable_cam_logging},
            {'cam_log_file_path': cam_log_file_path}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen'
    )

    # Remove GroupAction and return Node directly
    launch_description = LaunchDescription()
    launch_description.add_action(robot_index_arg)
    launch_description.add_action(filter_index_arg)
    launch_description.add_action(log_level_arg)
    launch_description.add_action(
        enable_cam_logging_arg)  # Add action for new arg
    launch_description.add_action(
        cam_log_file_path_arg)  # Add action for new arg
    launch_description.add_action(cam_communication_cmd)

    return launch_description
