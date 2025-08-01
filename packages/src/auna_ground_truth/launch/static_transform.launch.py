"""Static transform launch file: gazebo_world -> map (identity)"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_gazebo_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'gazebo_world', 'map'],
        output='screen'
    )

    return LaunchDescription([
        static_transform_publisher
    ])
