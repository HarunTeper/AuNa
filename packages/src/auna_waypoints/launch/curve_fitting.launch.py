# Copyright 2025 Harun Teper
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


"""Single car omnet module launch file."""
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Return launch description."""
    # Get MAP_NAME from environment variable, default to 'racetrack_decorated' if not set
    map_name = os.environ.get('MAP_NAME', 'racetrack_decorated')

    # auna_common paths
    auna_common_path = "/home/ubuntu/workspace/auna_common"
    waypoints_file = os.path.join(
        auna_common_path,
        'waypoints',
        map_name,
        'raw_waypoints.csv'
    )

    # Launch Argument Configurations
    interpolation_distance = LaunchConfiguration('interpolation_distance')
    plot_results = LaunchConfiguration('plot_results')
    swap_xy = LaunchConfiguration('swap_xy')
    output_file = LaunchConfiguration('output_file')
    reverse_order = LaunchConfiguration('reverse_order')
    is_closed_loop = LaunchConfiguration('is_closed_loop')

    # Launch Arguments
    interpolation_distance_arg = DeclareLaunchArgument(
        'interpolation_distance',
        default_value='0.1',
        description='Distance between waypoints to filter'
    )
    plot_results_arg = DeclareLaunchArgument(
        'plot_results',
        default_value='True',
        description='Plot the results of the curve fitting'
    )
    swap_xy_arg = DeclareLaunchArgument(
        'swap_xy',
        default_value='True',
        description='Swap the x and y coordinates of the waypoints'
    )
    output_file_arg = DeclareLaunchArgument(
        'output_file',
        default_value='cacc_waypoints',
        description='Output filename (without extension)'
    )
    reverse_order_arg = DeclareLaunchArgument(
        'reverse_order',
        default_value='True',
        description='Reverse the order of waypoints (change direction)'
    )
    is_closed_loop_arg = DeclareLaunchArgument(
        'is_closed_loop',
        default_value='True',
        description='Whether the waypoints form a closed loop (racetrack)'
    )

    # Nodes and other launch files
    curve_fitting_node = Node(
        package='auna_waypoints',
        executable='curve_fitting.py',
        name='curve_fitting',
        parameters=[{
            'waypoint_file': waypoints_file,
            'interpolation_distance': interpolation_distance,
            'plot_results': plot_results,
            'swap_xy': swap_xy,
            'output_file': output_file,
            'reverse_order': reverse_order,
            'is_closed_loop': is_closed_loop
        }],
        output='screen',
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(interpolation_distance_arg)
    launch_description.add_action(plot_results_arg)
    launch_description.add_action(swap_xy_arg)
    launch_description.add_action(output_file_arg)
    launch_description.add_action(reverse_order_arg)
    launch_description.add_action(is_closed_loop_arg)

    launch_description.add_action(curve_fitting_node)
    return launch_description
