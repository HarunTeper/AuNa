"""Gazebo world launch file"""
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument, AppendEnvironmentVariable
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_launch_description(context: LaunchContext):
    """Return launch description"""

    bridge_params = LaunchConfiguration('params_file_name')
    expand_gz_topic_names = LaunchConfiguration('expand_gz_topic_names')
    namespace = LaunchConfiguration('namespace')

    bridge_params_path = os.path.join(
        get_package_share_directory('auna_gazebo'),
        'config',
        'gazebo_params',
        context.launch_configurations['params_file_name'] + '.yaml'
    )

    gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_ros_bridge',
        namespace=namespace,
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params_path}'
        ],
        parameters=[{"expand_gz_topic_names": expand_gz_topic_names}],
        output='screen',
    )

    return [
        gazebo_ros_bridge_cmd
    ]


def generate_launch_description():
    """Return launch description"""

    params_file_name_arg = DeclareLaunchArgument(
        'params_file_name',
        default_value='bridge',
        description='Path to the parameters file'
    )

    expand_gz_topic_names_arg = DeclareLaunchArgument(
        'expand_gz_topic_names',
        default_value='false',
        description='Expand Gazebo topic names to ROS2 topic names'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='test',
        description='Namespace for the nodes'
    )

    return LaunchDescription([
        params_file_name_arg,
        expand_gz_topic_names_arg,
        namespace_arg,
        OpaqueFunction(function=include_launch_description)
    ])
