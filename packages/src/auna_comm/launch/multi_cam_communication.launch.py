"""Multiple cars omnet module launch file"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, GroupAction, LogInfo
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import PushRosNamespace, Node


def include_launch_description(context: LaunchContext):
    """Return launch description"""

    # Package Directories
    pkg_dir = get_package_share_directory('auna_comm')

    # Paths to folders and files
    omnet_launch_file_dir = os.path.join(pkg_dir, 'launch')

    # Launch Argument Configurations
    robot_number = LaunchConfiguration('robot_number', default='2')
    config_file = LaunchConfiguration('config_file')
    namespace = LaunchConfiguration('namespace')
    enable_cam_logging = LaunchConfiguration(
        'enable_cam_logging', default='false')
    cam_log_file_path = LaunchConfiguration(
        'cam_log_file_path', default='/home/vscode/workspace/cam_messages.log')

    # Get resolved values
    ns_value = context.perform_substitution(namespace)
    robot_number_value = int(context.perform_substitution(robot_number))
    enable_cam_logging_value = context.perform_substitution(enable_cam_logging)
    cam_log_file_path_value = context.perform_substitution(cam_log_file_path)

    # Convert string to boolean for enable_cam_logging parameter
    enable_cam_logging_bool = enable_cam_logging_value.lower() == 'true'

    # Nodes and other launch files
    launch_description_content = []

    # Log launch info
    launch_description_content.append(
        LogInfo(
            msg=f"Starting CAM Communication for {robot_number_value} robots with logging: {enable_cam_logging_value}")
    )

    for num in range(robot_number_value):
        # Define the namespace for this robot using the provided namespace parameter
        robot_ns = f'{ns_value}{num}'

        # Define the filter index (robot_index - 1)
        filter_idx = str(num-1)

        # Create unique log file path for each robot
        robot_log_file = cam_log_file_path_value.replace(
            '.log', f'_robot{num}.log')

        # Create group for this robot's nodes
        group_actions = [
            PushRosNamespace(robot_ns),

            # Include cam_communication node
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    omnet_launch_file_dir, 'cam_communication.launch.py')),
                launch_arguments={
                    'robot_index': str(num),
                    'filter_index': filter_idx,
                    'config_file': config_file,
                    # Pass the original string value ('true'/'false')
                    'enable_cam_logging': enable_cam_logging_value,
                    'cam_log_file_path': robot_log_file,
                }.items(),
            )
        ]

        # Only add cam_receiver node for robots other than the first one (not robot0)
        if num > 0:
            group_actions.append(
                Node(
                    package='auna_comm',
                    executable='cam_receiver',
                    name='cam_receiver',
                    parameters=[{
                        'filter_index': int(filter_idx),
                        'robot_namespace': robot_ns
                    }],
                    output='screen'
                )
            )

        launch_description_content.append(GroupAction(group_actions))

    return launch_description_content


def generate_launch_description():
    """Return launch description"""

    default_config = os.path.join(
        get_package_share_directory('auna_comm'),
        'config',
        'cam_params.yaml'
    )

    # Launch Arguments
    robot_number_arg = DeclareLaunchArgument(
        'robot_number',
        default_value='2',
        description='Number of spawned robots'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the config file'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace'
    )

    enable_cam_logging_arg = DeclareLaunchArgument(
        'enable_cam_logging',
        default_value='false',
        description='Enable CAM message logging'
    )

    cam_log_file_path_arg = DeclareLaunchArgument(
        'cam_log_file_path',
        default_value='/home/vscode/workspace/cam_messages.log',
        description='Path for CAM message log file'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(robot_number_arg)
    launch_description.add_action(config_file_arg)
    launch_description.add_action(namespace_arg)
    launch_description.add_action(enable_cam_logging_arg)
    launch_description.add_action(cam_log_file_path_arg)
    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
