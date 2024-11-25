""" Launch file for navigation. """

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from auna_common import xml_launch


def include_launch_description(context: LaunchContext):
    """Return launch description"""
    # Get the launch directory
    pkg_dir = get_package_share_directory('auna_nav2')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    default_bt_xml_file = os.path.join(
        pkg_dir, 'behavior_trees', 'navigate_through_poses_w_replanning_and_recovery.xml')

    namespaced_bt_xml_file = xml_launch.insert_namespace(
        default_bt_xml_file, namespace.perform(context))

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator',
                       'waypoint_follower']

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'default_nav_through_poses_bt_xml': namespaced_bt_xml_file
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    cmds = []

    cmds.append(Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings))

    cmds.append(Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings))

    cmds.append(Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings))

    cmds.append(Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
        remappings=remappings))

    cmds.append(Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params],
        remappings=remappings))

    cmds.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}]))

    return cmds


def generate_launch_description():
    """Return launch description"""

    # Get the launch directory
    pkg_dir = get_package_share_directory('auna_nav2')
    config_dir = os.path.join(pkg_dir, 'config', 'nav2_params')

    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(config_dir, 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use')

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(namespace_arg)
    launch_description.add_action(use_sim_time_arg)
    launch_description.add_action(autostart_arg)
    launch_description.add_action(params_file_arg)

    launch_description.add_action(OpaqueFunction(function=include_launch_description))

    return launch_description