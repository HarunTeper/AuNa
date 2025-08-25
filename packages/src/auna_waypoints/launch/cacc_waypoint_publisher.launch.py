import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('auna_waypoints')
    default_waypoint_file = os.path.join(pkg_dir, 'config', 'racetrack_decorated', 'waypoints.csv')

    # Get robot index from environment variable, fallback to launch argument
    robot_index = os.environ.get('ROBOT_INDEX', '1')

    robot_index_arg = DeclareLaunchArgument(
        'robot_index',
        default_value=robot_index,
        description='Index of the robot (e.g., 1 for robot1)'
    )

    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value=default_waypoint_file,
        description='Path to the waypoint file'
    )

    robot_namespace = ['robot', LaunchConfiguration('robot_index')]

    return LaunchDescription([
        robot_index_arg,
        waypoint_file_arg,
        Node(
            package='auna_waypoints',
            executable='cacc_waypoint_publisher',
            name='cacc_waypoint_publisher',
            output='screen',
            parameters=[
                {'waypoint_file': LaunchConfiguration('waypoint_file')},
                {'namespace': robot_namespace}
            ]
        )
    ])
