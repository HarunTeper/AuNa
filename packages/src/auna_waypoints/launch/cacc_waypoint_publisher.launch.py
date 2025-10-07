import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('auna_waypoints')
    world_name = os.environ.get('WORLD_NAME', 'racetrack_decorated')
    # Prefer precomputed yaw files if present, otherwise fall back to plain waypoints
    waypoints_dir = os.path.join(pkg_dir, 'config', world_name)
    default_waypoint_file = os.path.join(
        waypoints_dir, 'waypoints_with_yaw_central.csv')

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value=world_name,
        description='Name of the world'
    )

    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value=default_waypoint_file,
        description='Path to the waypoint file'
    )

    return LaunchDescription([
        world_name_arg,
        waypoint_file_arg,
        Node(
            package='auna_waypoints',
            executable='cacc_waypoint_publisher',
            name='cacc_waypoint_publisher',
            output='screen',
            parameters=[
                {'waypoint_file': LaunchConfiguration('waypoint_file')}
            ]
        )
    ])
