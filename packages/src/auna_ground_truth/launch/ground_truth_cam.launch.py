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

    return LaunchDescription([
        use_sim_time_arg,
        Node(
            package='auna_ground_truth',
            executable='ground_truth_cam_main',
            name='ground_truth_cam',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
