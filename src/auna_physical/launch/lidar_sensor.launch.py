"""Lidar sensor launch file"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.launch_context import LaunchContext
from auna_common import yaml_launch

def include_launch_description(context: LaunchContext):
    """Return launch description"""

    # File Paths
    config_file_path = os.path.join(get_package_share_directory('auna_physical'),'config','lidar_params.yaml')

    tmp_params_file = yaml_launch.get_yaml(config_file_path)
    tmp_params_file = yaml_launch.insert_namespace(tmp_params_file, context.launch_configurations['namespace'])
    tmp_params_file = yaml_launch.get_temp_file(tmp_params_file)

    with open(tmp_params_file, 'r') as file:
        config_params = yaml.safe_load(file)['urg_node2']['ros__parameters']

    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    scan_topic_name = LaunchConfiguration('scan_topic_name')
    urg_node_name = LaunchConfiguration('node_name')
    auto_start = LaunchConfiguration('auto_start')

    # Nodes and other launch files
    lifecycle_node = LifecycleNode(
        package='urg_node',
        executable='urg_node_driver',
        name=urg_node_name,
        remappings=[('scan', scan_topic_name)],
        parameters=[config_params],
        namespace=namespace,
        output='screen',
    )

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]


    if namespace.perform(context) == "":
        static_transform_node = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            namespace=namespace,
            arguments=["0.21", "0", "0.135", "0", "0", "0", "base_link", "laser"],
            remappings=remappings
        )
    else:
        static_transform_node = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            namespace=namespace,
            arguments=["0.21", "0", "0.135", "0", "0", "0", namespace.perform(context)+"/base_link", namespace.perform(context)+"/laser"],
            remappings=remappings
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


    launch_description_content = []

    launch_description_content.append(lifecycle_node)
    launch_description_content.append(static_transform_node)
    launch_description_content.append(urg_node2_node_configure_event_handler)
    launch_description_content.append(urg_node2_node_activate_event_handler)

    return launch_description_content


def generate_launch_description():
    """Return launch description"""

    #Launch arguments
    autostart_arg = DeclareLaunchArgument('auto_start', default_value='true')
    node_name_arg = DeclareLaunchArgument('node_name', default_value='urg_node2')
    scan_topic_name_arg = DeclareLaunchArgument('scan_topic_name', default_value='scan')
    namespace_arg = DeclareLaunchArgument('namespace', default_value='robot')

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(autostart_arg)
    launch_description.add_action(node_name_arg)
    launch_description.add_action(scan_topic_name_arg)
    launch_description.add_action(namespace_arg)

    launch_description.add_action(OpaqueFunction(function=include_launch_description))

    return launch_description
