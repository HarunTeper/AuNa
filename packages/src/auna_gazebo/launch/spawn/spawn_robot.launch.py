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


"""Single robot spawn launch file"""

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import SetRemap, PushRosNamespace
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_context import LaunchContext
from auna_common import yaml_launch


def include_launch_description(context: LaunchContext):
    """Return launch description"""
    # Package Directories
    pkg_dir = get_package_share_directory('auna_gazebo')
    spawn_launch_file_dir = os.path.join(pkg_dir, 'launch', 'spawn')

    # Launch Configurations
    robot_index = int(os.environ.get('ROBOT_INDEX', '0'))
    
    # Parameters
    name = f'robot{robot_index}'
    namespace = f'robot{robot_index}'
    urdf_namespace = f'robot{robot_index}'
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.0')
    
    world_name = os.environ.get('WORLD_NAME', 'racetrack_decorated')
    
    map_path = os.path.join(
        pkg_dir, "config", "map_params", f"{world_name}.yaml")
    num = int(robot_index)

    x_pose_value = yaml_launch.get_yaml_value(map_path, ["spawn", "offset", "x"]) + \
        num * yaml_launch.get_yaml_value(map_path, ["spawn", "linear", "x"])
    y_pose_value = yaml_launch.get_yaml_value(map_path, ["spawn", "offset", "y"]) + \
        num * yaml_launch.get_yaml_value(map_path, ["spawn", "linear", "y"])
    z_pose_value = yaml_launch.get_yaml_value(map_path, ["spawn", "offset", "z"]) + \
        num * yaml_launch.get_yaml_value(map_path, ["spawn", "linear", "z"])

    x_pose = str(x_pose_value)
    y_pose = str(y_pose_value)
    z_pose = str(z_pose_value)

    # Individual components
    tf_remap = SetRemap(src='/tf', dst='tf')
    tf_static_remap = SetRemap(src='/tf_static', dst='tf_static')

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(spawn_launch_file_dir,
                         '_robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(spawn_launch_file_dir,
                         '_spawn_robot_entity.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose,
            'urdf_namespace': urdf_namespace,
            'name': name
        }.items()
    )

    # localization_pose_publisher = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(spawn_launch_file_dir,
    #                      '_localization_pose_publisher.launch.py')
    #     )
    # )

    # ground_truth_cam = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(spawn_launch_file_dir,
    #                      '_ground_truth_cam.launch.py')
    #     )
    # )

    # ground_truth_transform = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(spawn_launch_file_dir,
    #                      '_ground_truth_transform.launch.py')
    #     ),
    #     condition=IfCondition(PythonExpression(ground_truth))
    # )

    # ground_truth_pose_publisher = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(spawn_launch_file_dir,
    #                      '_ground_truth_pose_publisher.launch.py')
    #     ),
    #     condition=IfCondition(PythonExpression(ground_truth))
    # )

    # # EKF Localization
    # ekf_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(ekf_launch_file_dir, 'ekf.launch.py')
    #     ),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #     }.items(),
    # ),

    # Main robot launch group with namespace and remappings
    robot_launch_group = GroupAction([
        PushRosNamespace(namespace),
        tf_remap,
        tf_static_remap,
        robot_state_publisher,
        spawn_robot,
        # delete_entity_service
        # localization_pose_publisher,
        # ground_truth_cam,
        # ground_truth_transform,
        # ground_truth_pose_publisher,
    ])
    
    launch_actions = []
    launch_actions.append(robot_launch_group)
    return launch_actions


def generate_launch_description():

    # Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    return LaunchDescription([
        use_sim_time_arg,
        OpaqueFunction(function=include_launch_description)
    ])
