import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch_ros.actions import Node


def include_launch_description(context: LaunchContext):
    """Return launch description"""

    # Package Directories
    pkg_dir = get_package_share_directory('auna_physical')
    cam_pkg_dir = get_package_share_directory('auna_comm')
    robomaster_pkg_dir = get_package_share_directory('robomaster_bringup')

    # Paths to folders and files
    launch_file_dir = os.path.join(pkg_dir, 'launch')
    cam_launch_file_dir = os.path.join(cam_pkg_dir, 'launch')
    robomaster_launch_file_dir = os.path.join(robomaster_pkg_dir, 'launch')

    # Launch Configuration
    namespace = LaunchConfiguration('namespace')
    robot_index = LaunchConfiguration('robot_index')

    cmds = []

    robomaster_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(robomaster_launch_file_dir, 'minimal.launch.py')),
        launch_arguments={
            'namespace': namespace.perform(context)+robot_index.perform(context),
        }.items(),
    )
    nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'f110_navigation.launch.py')),
        launch_arguments={
            'namespace': namespace.perform(context)+robot_index.perform(context),
        }.items(),
    )
    vicon_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'vicon_tf_converter.launch.py')),
        launch_arguments={
            'namespace': namespace.perform(context)+robot_index.perform(context),
        }.items(),
    )
    cam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cam_launch_file_dir, 'cam_communication.launch.py')),
        launch_arguments={
            'namespace': namespace.perform(context)+robot_index.perform(context),
            'robot_index': robot_index,
            'filter_index': str(int(robot_index.perform(context))-1),
        }.items(),
    )

    cmds.append(robomaster_bringup_cmd)
    cmds.append(nav_cmd)
    cmds.append(vicon_cmd)
    cmds.append(cam_cmd)

    return cmds


def generate_launch_description():
    """Return launch description"""

    # Launch arguments
    namespace_arg = DeclareLaunchArgument('namespace', default_value='robot')
    robot_index_arg = DeclareLaunchArgument('robot_index', default_value='0')

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(namespace_arg)
    launch_description.add_action(robot_index_arg)

    launch_description.add_action(OpaqueFunction(function=include_launch_description))

    return launch_description
