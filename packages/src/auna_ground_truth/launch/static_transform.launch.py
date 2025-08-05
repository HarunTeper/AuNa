"""Static transform launch file: gazebo_world -> map (identity)"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare use_sim_time argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_gazebo_world_to_map',
        arguments=['0', '0', '0', '0', '0', '0', 'gazebo_world', 'map'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        static_transform_publisher
    ])
