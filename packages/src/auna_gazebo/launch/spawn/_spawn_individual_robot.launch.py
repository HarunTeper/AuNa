"""Single robot spawn launch file"""

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetRemap
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression


def generate_launch_description():
    """Return launch description"""
    # Package Directories
    pkg_dir = get_package_share_directory('auna_gazebo')

    # Paths to folders and files
    spawn_launch_file_dir = os.path.join(
        pkg_dir, 'launch', 'spawn')  # Renamed for clarity
    ground_truth_launch_dir = os.path.join(
        pkg_dir, 'launch', 'ground_truth_localization')
    ekf_launch_file_dir = os.path.join(
        pkg_dir, 'launch', 'ekf')  # Path for EKF launch files

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
                os.path.join(spawn_launch_file_dir,  # Use renamed variable
                             '_robot_state_publisher.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items()
        ),
        # Spawn robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                # Use renamed variable
                os.path.join(spawn_launch_file_dir,
                             '_spawn_robot_entity.launch.py')
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(spawn_launch_file_dir,  # Use renamed variable
                             '_localization_pose_publisher.launch.py')
            )
        ),
        # Ground truth cam
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                # Use renamed variable
                os.path.join(spawn_launch_file_dir,
                             '_ground_truth_cam.launch.py')
            )
        ),
        # Ground truth localization (conditional)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(spawn_launch_file_dir,  # This path seems to be used in the original file for this
                             '_ground_truth_localization.launch.py')
            ),
            # Launch if ground_truth is 'True'
            condition=IfCondition(PythonExpression(
                [ground_truth, " == 'True'"]))
        ),
        # EKF Localization (conditional, opposite to ground_truth)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ekf_launch_file_dir, 'localization_ekf.launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                # ekf_config_file will use its default from localization_ekf.launch.py
            }.items(),
            # Launch if ground_truth is 'False'
            condition=IfCondition(PythonExpression(
                [ground_truth, " == 'False'"]))
        ),
    ])

    return LaunchDescription([
        *launch_args,
        robot_launch_group
    ])
