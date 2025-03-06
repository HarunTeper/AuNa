"""Launch file for the CAM receiver node"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Return launch description"""

    # Launch Arguments
    filter_index_arg = DeclareLaunchArgument(
        'filter_index',
        default_value='0',
        description='Station ID to filter messages by'
    )

    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot1',
        description='Namespace for republished CAM messages'
    )

    # Create node
    cam_receiver_node = Node(
        package='auna_comm',
        executable='cam_receiver',
        name='cam_receiver',
        parameters=[{
            'filter_index': LaunchConfiguration('filter_index'),
            'robot_namespace': LaunchConfiguration('robot_namespace')
        }],
        output='screen'
    )

    # Launch Description
    return LaunchDescription([
        filter_index_arg,
        robot_namespace_arg,
        cam_receiver_node
    ])
