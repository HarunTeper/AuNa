"""Robot state publisher launch file"""
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    """Return launch description"""
    # Package Directories
    pkg_dir = get_package_share_directory('auna_gazebo')

    # Paths to folders and files
    default_model_path = os.path.join(pkg_dir, 'models/race_car/model.urdf')

    # Launch Configurations
    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch Arguments
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Define the robot state publisher
    robot_state_group = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command([
                'xacro ', model
            ]),
            'use_sim_time': use_sim_time,
            'use_tf_static': False
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
    )

    return LaunchDescription([
        # Launch Arguments
        model_arg,
        use_sim_time_arg,
        # Nodes
        robot_state_group
    ])
