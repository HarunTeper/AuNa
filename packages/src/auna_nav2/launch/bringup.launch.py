"""Launch all the navigation nodes."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, SetRemap


def generate_launch_description():
    """Launch all the navigation nodes."""

    # Get the launch directory
    bringup_dir = get_package_share_directory('auna_nav2')
    config_dir = os.path.join(bringup_dir, 'config', 'nav2_params')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    enable_slam = LaunchConfiguration('enable_slam')
    enable_localization = LaunchConfiguration('enable_localization')
    enable_navigation = LaunchConfiguration('enable_navigation')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(config_dir, 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_enable_slam_cmd = DeclareLaunchArgument(
        'enable_slam', default_value='false',
        description='Enable SLAM')

    declare_enable_localization_cmd = DeclareLaunchArgument(
        'enable_localization', default_value='true',
        description='Enable Localization')

    declare_enable_navigation_cmd = DeclareLaunchArgument(
        'enable_navigation', default_value='true',
        description='Enable Navigation')

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(namespace=namespace),
        SetRemap('/tf', 'tf'),
        SetRemap('/tf_static', 'tf_static'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam.launch.py')),
            condition=IfCondition(enable_slam),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir,
                                                       'localization.launch.py')),
            condition=IfCondition(enable_localization),
            launch_arguments={'namespace': namespace,
                              'map': map_yaml_file,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation.launch.py')),
            condition=IfCondition(enable_navigation),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_lifecycle_mgr': 'false',
                              'map_subscribe_transient_local': 'true'}.items()),
    ])

    # Create the launch description and populate
    launch_description = LaunchDescription()

    # Set environment variables
    launch_description.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    launch_description.add_action(declare_namespace_cmd)
    launch_description.add_action(declare_slam_cmd)
    launch_description.add_action(declare_map_yaml_cmd)
    launch_description.add_action(declare_use_sim_time_cmd)
    launch_description.add_action(declare_params_file_cmd)
    launch_description.add_action(declare_autostart_cmd)
    launch_description.add_action(declare_enable_slam_cmd)
    launch_description.add_action(declare_enable_localization_cmd)
    launch_description.add_action(declare_enable_navigation_cmd)

    # Add the actions to launch all of the navigation nodes
    launch_description.add_action(bringup_cmd_group)

    return launch_description
