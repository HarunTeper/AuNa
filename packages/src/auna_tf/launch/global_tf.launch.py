"""Localization pose publisher launch file"""
from launch_ros.actions import Node, SetRemap
from launch.actions import GroupAction, DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Return launch description"""

    # Declare use_sim_time argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    group_cmd = GroupAction([
        Node(
            package='auna_tf',
            executable='global_tf',
            name='global_tf',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])

    return LaunchDescription([
        use_sim_time_arg,
        group_cmd
    ])
