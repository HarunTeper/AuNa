import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_launch_description(context: LaunchContext):
    """Return launch description"""

    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace')

    launch_description_content = []

    launch_description_content.append(
        Node(
            package='auna_cacc',
            executable='cacc_controller',
            name='cacc_controller',
            namespace=namespace,
            output='screen'
        )
    )

    return launch_description_content


def generate_launch_description():
    """Return launch description"""

    # Launch Arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot',
        description='Number of spawned robots'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(namespace_arg)

    launch_description.add_action(OpaqueFunction(function=include_launch_description))

    return launch_description
