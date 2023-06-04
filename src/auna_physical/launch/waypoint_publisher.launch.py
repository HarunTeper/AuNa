"""Single car omnet module launch file"""

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Return launch description"""

    # Package Directories
    pkg_dir = get_package_share_directory('auna_physical')

    # Config files
    waypoints = os.path.join(pkg_dir, 'config', 'fitted_waypoints.csv')

    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace')

    # Launch Arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot',
        description='Robot namespace for ROS nodes and topics'
    )

    # Nodes and other launch files
    waypoint_publisher_cmd = Node(
        package='auna_physical',
        executable='waypoint_publisher',
        name='waypoint_publisher',
        namespace=namespace,
        parameters=[{'namespace': LaunchConfiguration('namespace'), 'waypoint_file': waypoints}],
        output='screen',
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(namespace_arg)

    launch_description.add_action(waypoint_publisher_cmd)
    return launch_description
