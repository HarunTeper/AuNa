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
    interpolation_distance = LaunchConfiguration('interpolation_distance')
    plot_results = LaunchConfiguration('plot_results')
    swap_xy = LaunchConfiguration('swap_xy')

    # Launch Arguments
    interpolation_distance_arg = DeclareLaunchArgument(
        'interpolation_distance',
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
    curve_fitting_node = Node(
        package='auna_physical',
        executable='curve_fitting.py',
        name='curve_fitting',
        parameters=[{'waypoint_file': waypoints,
                     'interpolation_distance': interpolation_distance,
                     'plot_results': plot_results,
                     'swap_xy': swap_xy}],
        output='screen',
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(interpolation_distance_arg)
    launch_description.add_action(plot_results_arg)
    launch_description.add_action(swap_xy_arg)

    launch_description.add_action(curve_fitting_node)
    return launch_description
