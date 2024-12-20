"""Spawn robot launch file"""
from launch_ros.actions import Node, SetRemap, PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, GroupAction
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import TextSubstitution


def generate_launch_description():
    """Return launch description"""

    namespace = LaunchConfiguration('namespace')

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        description='ROS2 robot namespace (must not be empty)',
    )

    group_cmd = GroupAction([
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),
        Node(
            package='auna_gazebo',
            executable='ground_truth_localization',
            name='ground_truth_localization',
            output='screen',
            arguments=[namespace],
        )
    ])

    return LaunchDescription([
        namespace_arg,
        group_cmd
    ])
