import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('auna_waypoints')
    waypoint_file = os.path.join(pkg_dir, 'config', 'waypoints.csv')

    return LaunchDescription([
        Node(
            package='auna_waypoints',
            executable='cacc_waypoint_publisher',
            name='cacc_waypoint_publisher',
            output='screen',
            parameters=[{'waypoint_file': waypoint_file}]
        )
    ])
