"""Spawn robot launch file"""
from launch_ros.actions import Node, SetRemap
from launch.actions import GroupAction
from launch import LaunchDescription


def generate_launch_description():
    """Return launch description"""

    group_cmd = GroupAction([
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),
        Node(
            package='auna_gazebo',
            executable='ground_truth_localization',
            name='ground_truth_localization',
            output='screen',
            # arguments=[namespace],
        )
    ])

    return LaunchDescription([
        group_cmd
    ])
