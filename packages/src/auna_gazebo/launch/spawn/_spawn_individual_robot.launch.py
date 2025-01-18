"""Single robot spawn launch file"""

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.actions import LogInfo, EmitEvent
from launch.events import Shutdown


def generate_launch_description():
    """Return launch description"""
    # Package Directories
    pkg_dir = get_package_share_directory('auna_gazebo')

    # Paths to folders and files
    launch_file_dir = os.path.join(pkg_dir, 'launch', 'spawn')
    ground_truth_launch_dir = os.path.join(
        pkg_dir, 'launch', 'ground_truth_localization')

    # Launch Configurations
    name = LaunchConfiguration('name')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    ground_truth = LaunchConfiguration('ground_truth')

    # Launch Arguments
    launch_args = [
        DeclareLaunchArgument(
            'name',
            default_value='robot',
            description='Robot name in Gazebo'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='robot',
            description='Robot namespace for ROS nodes and topics'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'x_pose',
            default_value='0.0',
            description='Robot spawn x position'
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value='0.0',
            description='Robot spawn y position'
        ),
        DeclareLaunchArgument(
            'z_pose',
            default_value='0.0',
            description='Robot spawn z position'
        ),
        DeclareLaunchArgument(
            'ground_truth',
            default_value='False',
            description='Use ground truth pose'
        )
    ]

    # Main robot launch group with namespace and remappings
    robot_launch_group = GroupAction([
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),

        # Robot state publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir,
                             '_robot_state_publisher.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'namespace': namespace
            }.items()
        ),
        # Spawn robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, '_spawn_robot_entity.launch.py')
            ),
            launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose,
                'z_pose': z_pose,
                'namespace': namespace,
                'name': name
            }.items()
        ),
        # Localization pose publisher
        Node(
            package='auna_gazebo',
            executable='localization_pose_publisher',
            name='localization_pose_publisher',
            arguments={namespace},
            output='screen'
        ),
        # Ground truth cam
        Node(
            package='auna_gazebo',
            executable='ground_truth_cam',
            name='ground_truth_cam',
            arguments={name},
            output='screen'
        ),
        # Ground truth localization (conditional)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ground_truth_launch_dir,
                             'ground_truth_localization.launch.py')
            ),
            condition=IfCondition(PythonExpression([ground_truth]))
        ),
    ])

    return LaunchDescription([
        *launch_args,
        robot_launch_group
    ])
