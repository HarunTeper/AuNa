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
    waypoints = os.path.join(pkg_dir, 'config', 'flw_waypoints.csv')

    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace')
    filter_distance = LaunchConfiguration('filter_distance')
    plot_results = LaunchConfiguration('plot_results')
    swap_xy = LaunchConfiguration('swap_xy')

    # Launch Arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot',
        description='Robot namespace for ROS nodes and topics'
    )
    filter_distance_arg = DeclareLaunchArgument(
        'filter_distance',
        default_value='1.0',
        description='Distance between waypoints to filter'
    )
    plot_results_arg = DeclareLaunchArgument(
        'plot_results',
        default_value='False',
        description='Plot the results of the curve fitting'
    )
    swap_xy_arg = DeclareLaunchArgument(
        'swap_xy',
        default_value='False',
        description='Swap the x and y coordinates of the waypoints'
    )

    # Nodes and other launch files
    waypoint_publisher_cmd = Node(
        package='auna_physical',
        executable='curve_fitting.py',
        name='curve_fitting',
        namespace=namespace,
        parameters=[{'namespace': LaunchConfiguration('namespace'),
                     'waypoint_file': waypoints,
                     'filter_distance': filter_distance,
                     'plot_results': plot_results,
                     'swap_xy': swap_xy}],
        output='screen',
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(namespace_arg)
    launch_description.add_action(filter_distance_arg)
    launch_description.add_action(plot_results_arg)
    launch_description.add_action(swap_xy_arg)

    launch_description.add_action(waypoint_publisher_cmd)
    return launch_description
