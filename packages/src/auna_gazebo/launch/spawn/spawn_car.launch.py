"""Spawn robot launch file"""
from launch_ros.actions import Node, SetRemap, PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, EmitEvent, RegisterEventHandler
from launch.events import Shutdown
from launch.event_handlers import OnExecutionComplete
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression


def generate_launch_description():
    """Return launch description"""
    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace')

    # Launch Arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='ROS2 robot namespace'
    )

    # Namespace verification
    verification_error = LogInfo(
        msg="ERROR: Namespace must be provided! Please set the 'namespace' launch argument.",
        condition=IfCondition(PythonExpression(["'", namespace, "' == ''"])))

    shutdown_on_error = EmitEvent(
        event=Shutdown(reason="No namespace provided"),
        condition=IfCondition(PythonExpression(["'", namespace, "' == ''"])))

    # Group command with remappings and namespace
    group_cmd = GroupAction([
        PushRosNamespace(namespace),
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),
        Node(
            package='auna_gazebo',
            executable='ground_truth_localization',
            name='ground_truth_localization',
            output='screen',
            arguments={namespace}
        )
    ], condition=IfCondition(PythonExpression(["'", namespace, "' != ''"])))

    return LaunchDescription([
        namespace_arg,
        verification_error,
        shutdown_on_error,
        group_cmd
    ])
