import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.launch_context import LaunchContext
from launch_ros.actions import Node, PushRosNamespace


def include_launch_description(context: LaunchContext):
    """Create namespaced Nav2 waypoint publisher node."""

    launch_description_content = []

    # Build namespace from robot_index (e.g., robot0, robot1, ...)
    robot_index = context.launch_configurations.get('robot_index', '0')
    namespace = f"robot{robot_index}"

    # Package directory and default waypoint file
    pkg_dir = get_package_share_directory('auna_waypoints')
    waypoint_file = os.path.join(
        pkg_dir, 'config', 'nav2_racetrack_waypoints.yaml')

    node = Node(
        package='auna_waypoints',
        executable='nav2_waypoint_publisher',
        name='nav2_waypoint_publisher',
        output='screen',
        parameters=[
            {'waypoint_file': waypoint_file},
            {'namespace': namespace},
            {'wait_for_bt_active': True},
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    # Don't use PushRosNamespace - let the code handle namespacing via the namespace parameter
    launch_description_content.append(node)

    return launch_description_content


def generate_launch_description():
    ld = LaunchDescription()

    # Allow caller to choose which robot index to use for namespacing
    ld.add_action(DeclareLaunchArgument(
        'robot_index',
        default_value='0',
        description='Index of the robot to namespace the node (e.g., robot0).',
    ))

    ld.add_action(OpaqueFunction(function=include_launch_description))

    return ld
