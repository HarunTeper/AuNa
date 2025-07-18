from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='auna_ground_truth',
            executable='ground_truth_pose_publisher_main',
            name='ground_truth_pose_publisher',
            output='screen',
        ),
    ])
