import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directory and default waypoint file
    pkg_dir = get_package_share_directory('auna_waypoints')
    waypoint_file = os.path.join(
        pkg_dir, 'config', 'racetrack_decorated', 'nav2_racetrack_waypoints.yaml')

    # Get robot index from environment variable, fallback to launch argument
    robot_index = os.environ.get('ROBOT_INDEX', '1')

    # Launch arguments
    robot_index_arg = DeclareLaunchArgument(
        'robot_index',
        default_value=robot_index,
        description='Index of the robot (e.g., 1 for robot1)'
    )

    # Create namespace from robot_index
    robot_namespace = ['robot', LaunchConfiguration('robot_index')]

    # Nav2 waypoint publisher node
    nav2_waypoint_publisher_node = Node(
        package='auna_waypoints',
        executable='nav2_waypoint_publisher',
        name='nav2_waypoint_publisher',
        output='screen',
        parameters=[
            {'waypoint_file': waypoint_file},
            {'namespace': robot_namespace},
        ]
    )

    return LaunchDescription([
        robot_index_arg,
        nav2_waypoint_publisher_node
    ])
