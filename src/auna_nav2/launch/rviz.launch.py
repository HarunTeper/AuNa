""" Launch RViz2 with the default view for the navigation stack. """

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString
from launch.launch_context import LaunchContext


def include_launch_description(context: LaunchContext):
    """Return launch description"""

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    rviz_config_file = LaunchConfiguration('rviz_config')

    # Launch rviz
    if namespace.perform(context) == '':
        namespaced_rviz_config_file = ReplaceString(
            source_file=rviz_config_file,
            replacements={'<robot_namespace>': ('')})
        base_frame = ''
    else:
        namespaced_rviz_config_file = ReplaceString(
                source_file=rviz_config_file,
                replacements={'<robot_namespace>': ('/', namespace)})
        base_frame = namespace.perform(context)+'/'+'base_link  '

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        namespace=namespace,
        arguments=['-d', namespaced_rviz_config_file],
        output='screen',
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/goal_pose', 'goal_pose'),
                    ('/clicked_point', 'clicked_point'),
                    ('/initialpose', 'initialpose')],
        parameters=[{'base_frame': base_frame}]
    )

    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))

    launch_description_content = []

    # Add any conditioned actions
    launch_description_content.append(start_rviz_cmd)

    # Add other nodes and processes we need
    launch_description_content.append(exit_event_handler)

    return launch_description_content

def generate_launch_description():
    """Launch RViz2 with the default view for the navigation stack. """
    # Get the launch directory
    pkg_dir = get_package_share_directory('auna_nav2')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='navigation',
        description=('Top-level namespace. The value will be used to replace the '
                     '<robot_namespace> keyword on the rviz config file.'))

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_dir, 'rviz', 'config_navigation_namespace.rviz'),
        description='Full path to the RVIZ config file to use')

    # Create the launch description and populate
    launch_description = LaunchDescription()

    # Declare the launch options
    launch_description.add_action(declare_namespace_cmd)
    launch_description.add_action(declare_rviz_config_file_cmd)
    launch_description.add_action(OpaqueFunction(function=include_launch_description))

    return launch_description
