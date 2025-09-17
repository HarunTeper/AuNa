"""Single robot communication launch file"""

import os
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext


def include_launch_description(context: LaunchContext):
    """Return launch description"""

    communication_type = os.environ.get('COMMUNICATION_TYPE', 'cam')
    if communication_type != 'cam':
        return [LogInfo(msg="COMMUNICATION_TYPE is not 'cam', skipping CAM nodes.")]
    # Project uses 1-based ROBOT_INDEX in compose (1=leader). Followers should filter leader=robot_index-1
    robot_index = int(os.environ.get('ROBOT_INDEX', '1'))

    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace')
    log_level = LaunchConfiguration('log_level')
    enable_cam_logging = LaunchConfiguration('enable_cam_logging')
    cam_log_file_path = LaunchConfiguration('cam_log_file_path')

    # Get resolved values
    namespace_value = context.perform_substitution(namespace)

    # Create unique log file path for each robot
    robot_log_file = context.perform_substitution(
        cam_log_file_path).replace('.log', f'_{namespace_value}.log')

    cam_communication_cmd = Node(
        package='auna_comm',
        executable='cam_communication',
        name='cam_communication',
        namespace=namespace,
        parameters=[
            {'robot_index': robot_index},
            {'enable_cam_logging': enable_cam_logging},
            {'cam_log_file_path': robot_log_file}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen'
    )

    # For leader (robot_index==1), don't start a receiver. Followers listen to preceding vehicle.
    launch_nodes = [cam_communication_cmd]
    if robot_index > 1:
        cam_receiver_cmd = Node(
            package='auna_comm',
            executable='cam_receiver',
            name='cam_receiver',
            namespace=namespace,
            parameters=[{
                'filter_index': robot_index - 1,
                'robot_namespace': namespace_value
            }],
            output='screen'
        )
        launch_nodes.append(cam_receiver_cmd)

    return launch_nodes


def generate_launch_description():
    """Return launch description"""
    # Determine default namespace from ROBOT_INDEX so docker-compose spawns namespaced nodes
    robot_index_env = os.environ.get('ROBOT_INDEX', '0')
    default_namespace = f'robot{robot_index_env}'

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value=default_namespace,
        description='Robot namespace'
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
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

    launch_description.add_action(namespace_arg)
    launch_description.add_action(log_level_arg)
    launch_description.add_action(enable_cam_logging_arg)
    launch_description.add_action(cam_log_file_path_arg)
    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
