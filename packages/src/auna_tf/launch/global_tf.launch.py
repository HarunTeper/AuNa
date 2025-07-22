"""Localization pose publisher launch file"""
from launch_ros.actions import Node, SetRemap
from launch.actions import GroupAction
from launch import LaunchDescription


def generate_launch_description():
    """Return launch description"""

    group_cmd = GroupAction([
        Node(
            package='auna_tf',
            executable='global_tf',
            name='global_tf',
            output='screen'
        )
    ])

    return LaunchDescription([
        group_cmd
    ])
