"""Launch file for complete CACC system with CAM communication and controller"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_launch_description(context: LaunchContext):
    """Return launch description"""

    # Package Directories
    auna_comm_pkg_dir = get_package_share_directory('auna_comm')
    auna_cacc_pkg_dir = get_package_share_directory('auna_cacc')

    # Launch Argument Configurations
    robot_number = LaunchConfiguration('robot_number')
    namespace = LaunchConfiguration('namespace')
    config_file = LaunchConfiguration('config_file')

    # Get resolved values
    robot_number_value = context.perform_substitution(robot_number)
    namespace_value = context.perform_substitution(namespace)

    # Launch the CAM communication system
    cam_comm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            auna_comm_pkg_dir, 'launch', 'multi_cam_communication.launch.py')),
        launch_arguments={
            'robot_number': robot_number_value,
            'namespace': namespace_value,
            'config_file': config_file
        }.items(),
    )

    # Launch the CACC controller system
    cacc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            auna_cacc_pkg_dir, 'launch', 'cacc_controller.launch.py')),
        launch_arguments={
            'robot_number': robot_number_value,
            'namespace': namespace_value
        }.items(),
    )

    return [cam_comm_launch, cacc_launch]


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

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot',
        description='Namespace for spawned robots'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the config file'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(robot_number_arg)
    launch_description.add_action(namespace_arg)
    launch_description.add_action(config_file_arg)

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
