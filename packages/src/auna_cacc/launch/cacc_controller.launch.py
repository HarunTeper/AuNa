import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, GroupAction
from launch_ros.actions import PushRosNamespace
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from auna_common import yaml_launch


def include_launch_description(context: LaunchContext):
    """Return launch description"""

    # Launch Argument Configurations
    robot_number = LaunchConfiguration('robot_number', default='2')
    namespace = LaunchConfiguration('namespace', default='robot')
    enable_logging = LaunchConfiguration('enable_logging', default='true')
    log_file_path = LaunchConfiguration(
        'log_file_path', default='/home/vscode/workspace/cacc_log.csv')
    cacc_config = LaunchConfiguration('cacc_config')
    waypoint_file = LaunchConfiguration('waypoint_file')

    # Get resolved values
    ns_value = context.perform_substitution(namespace)
    robot_number_value = int(context.perform_substitution(robot_number))
    enable_logging_value = context.perform_substitution(enable_logging)
    log_file_path_value = context.perform_substitution(log_file_path)

    # Convert string to boolean for enable_logging parameter
    enable_logging_bool = enable_logging_value.lower() == 'true'

    launch_description_content = []

    # Explicitly log the number of robots detected
    launch_description_content.append(
        LogInfo(
            msg=f"CACC launch: Detected {robot_number_value} robots with base namespace '{ns_value}'")
    )

    # If we have multiple robots, start CACC controllers ONLY for follower robots (not the lead robot)
    launch_description_content.append(
        LogInfo(
            msg=f"Starting CACC controllers for {robot_number_value-1} follower robots")
    )

    # Start from 1 (skip robot0) since only follower robots need CACC controllers
    for num in range(1, robot_number_value):
        # Create namespace string properly by concatenating the resolved namespace value
        robot_ns = f"{ns_value}{num}"

        # Create unique log file path for each robot if logging is enabled
        robot_log_file = log_file_path_value.replace(
            '.csv', f'_{robot_ns}.csv')

        # Log explicitly which robot we're starting a controller for
        launch_description_content.append(
            LogInfo(
                msg=f"Starting CACC controller for {robot_ns} (follower robot)")
        )

        # Use PushRosNamespace within a GroupAction to properly namespace the node
        launch_description_content.append(
            GroupAction([
                PushRosNamespace(robot_ns),
                Node(
                    package='auna_cacc',
                    executable='cacc_controller',
                    name='cacc_controller',
                    output='screen',
                    parameters=[
                        yaml_launch.get_yaml_value(cacc_config.perform(
                            context), ['cacc_controller', 'ros__parameters']),
                        {
                            'enable_data_logging': enable_logging_bool,
                            'log_file_path': robot_log_file,
                            'waypoint_file': waypoint_file
                        }
                    ]
                )
            ])
        )

    return launch_description_content


def generate_launch_description():
    """Return launch description"""

    # Package Directories
    pkg_dir = get_package_share_directory('auna_cacc')

    # Config files
    cacc_config_file_path = os.path.join(
        pkg_dir, 'config', 'cacc_controller.yaml')

    # Waypoint file
    waypoint_file_path = os.path.join(
        pkg_dir, 'config', 'waypoints.csv')

    # Add additional logging at the beginning
    initial_log = LogInfo(
        msg="Initializing CACC controller launch - will only start for follower robots"
    )

    # Launch Arguments
    robot_number_arg = DeclareLaunchArgument(
        'robot_number',
        default_value='2',
        description='Number of spawned robots'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot',
        description='Namespace for spawned robots'
    )

    enable_logging_arg = DeclareLaunchArgument(
        'enable_logging',
        default_value='true',
        description='Enable data logging for CACC controllers'
    )

    log_file_path_arg = DeclareLaunchArgument(
        'log_file_path',
        default_value='/home/vscode/workspace/cacc_log.csv',
        description='Base path for log files'
    )

    cacc_config_arg = DeclareLaunchArgument(
        'cacc_config',
        default_value=cacc_config_file_path,
        description='Path to cacc config file'
    )

    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value=waypoint_file_path,
        description='Path to waypoint file'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(initial_log)
    launch_description.add_action(robot_number_arg)
    launch_description.add_action(namespace_arg)
    launch_description.add_action(enable_logging_arg)
    launch_description.add_action(log_file_path_arg)
    launch_description.add_action(cacc_config_arg)
    launch_description.add_action(waypoint_file_arg)

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
