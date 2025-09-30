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


"""Ground truth launch file."""
from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch_ros.actions import PushRosNamespace, SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def include_ground_truth_launches(context: LaunchContext):
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_dir = get_package_share_directory('auna_ground_truth')
    launch_dir = os.path.join(pkg_dir, 'launch')

    robot_index = int(os.environ.get('ROBOT_INDEX', '0'))

    ground_truth_transform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                launch_dir,
                'ground_truth_transform.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time}.items())
    ground_truth_pose_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                launch_dir,
                'ground_truth_pose_publisher.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time}.items())
    ground_truth_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'ground_truth_cam.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    # Individual components
    tf_remap = SetRemap(src='/tf', dst='tf')
    tf_static_remap = SetRemap(src='/tf_static', dst='tf_static')

    group_cmd = GroupAction([
        PushRosNamespace(f'robot{robot_index}'),
        tf_remap,
        tf_static_remap,
        ground_truth_transform,
        ground_truth_pose_publisher,
        ground_truth_cam
    ])

    return [group_cmd]


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    return LaunchDescription([
        use_sim_time_arg,
        OpaqueFunction(function=include_ground_truth_launches)
    ])
