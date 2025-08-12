"""Single car omnet module launch file"""

import os
from launch_ros.actions import Node, PushRosNamespace
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from ament_index_python.packages import get_package_share_directory


def include_launch_description(context: LaunchContext):
    """Return launch description"""

    launch_description_content = []

    # Get robot_index and construct namespace
    robot_index = context.launch_configurations.get('robot_index', '0')
    namespace = f"robot{robot_index}"

    # Package Directories
    pkg_dir = get_package_share_directory('auna_waypoints')

    # Get MAP_NAME from environment variable, default to 'default' if not set
    map_name = os.environ.get('MAP_NAME', 'default')

    # Config files
    waypoints = os.path.join(pkg_dir, 'config', map_name, 'waypoints.csv')

    waypoint_publisher_node = Node(
        package='auna_waypoints',
        executable='waypoint_publisher',
        name='waypoint_publisher',
        parameters=[{'waypoint_file': waypoints}],
        output='screen',
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]
    )

    push_ns = PushRosNamespace(namespace)
    launch_description_content.append(push_ns)

    # Add the waypoint_publisher node to the launch description content
    launch_description_content.append(waypoint_publisher_node)

    return launch_description_content


def generate_launch_description():
    """Return launch description"""
    # Launch Description
    launch_description = LaunchDescription()

    # Declare robot_index argument
    launch_description.add_action(DeclareLaunchArgument(
        'robot_index',
        default_value='0',
        description='Index of the robot to namespace the node.'
    ))

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
