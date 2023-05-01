"""Lidar sensor launch file"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import (DeclareLaunchArgument, EmitEvent, RegisterEventHandler)
from launch.event_handlers import OnProcessStart
from launch.events import matches_action

def generate_launch_description():
    """Return launch description"""

    # File Paths
    config_file_path = os.path.join(get_package_share_directory('auna_physical'),'config','lidar_params.yaml')

    # Read configuration files
    with open(config_file_path, 'r') as file:
        config_params = yaml.safe_load(file)['urg_node2']['ros__parameters']

    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    scan_topic_name = LaunchConfiguration('scan_topic_name')
    urg_node_name = LaunchConfiguration('node_name')
    auto_start = LaunchConfiguration('auto_start')

    #Launch arguments
    autostart_arg = DeclareLaunchArgument('auto_start', default_value='true')
    node_name_arg = DeclareLaunchArgument('node_name', default_value='urg_node2')
    scan_topic_name_arg = DeclareLaunchArgument('scan_topic_name', default_value='scan')
    namespace_arg = DeclareLaunchArgument('namespace', default_value='robot')

    # Nodes and other launch files
    lifecycle_node = LifecycleNode(
        package='urg_node2',
        executable='urg_node2_node',
        name=urg_node_name,
        remappings=[('scan', scan_topic_name)],
        parameters=[config_params],
        namespace=namespace,
        output='screen',
    )

    urg_node2_node_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=lifecycle_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(auto_start),
    )

    urg_node2_node_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=lifecycle_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(lifecycle_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(auto_start),
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(autostart_arg)
    launch_description.add_action(node_name_arg)
    launch_description.add_action(scan_topic_name_arg)
    launch_description.add_action(namespace_arg)
    launch_description.add_action(lifecycle_node)
    launch_description.add_action(urg_node2_node_configure_event_handler)
    launch_description.add_action(urg_node2_node_activate_event_handler)

    return launch_description
