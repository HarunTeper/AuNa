"""Single robot spawn launch file"""

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetRemap, PushRosNamespace
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression, LocalSubstitution
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.launch_context import LaunchContext
from launch.event_handlers import OnShutdown


def include_launch_description(context: LaunchContext):
    """Return launch description"""
    # Package Directories
    pkg_dir = get_package_share_directory('auna_gazebo')
    spawn_launch_file_dir = os.path.join(pkg_dir, 'launch', 'spawn')

    # Launch Configurations
    name = LaunchConfiguration('name')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    ground_truth = LaunchConfiguration('ground_truth')
    debug_ekf = LaunchConfiguration('debug_ekf')

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
            'name': name
        }.items()
    )

    localization_pose_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(spawn_launch_file_dir,
                         '_localization_pose_publisher.launch.py')
        )
    )

    ground_truth_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(spawn_launch_file_dir,
                         '_ground_truth_cam.launch.py')
        )
    )

    ground_truth_transform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(spawn_launch_file_dir,
                         '_ground_truth_transform.launch.py')
        ),
        condition=IfCondition(PythonExpression(ground_truth))
    )

    ground_truth_pose_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(spawn_launch_file_dir,
                         '_ground_truth_pose_publisher.launch.py')
        ),
        condition=IfCondition(PythonExpression(ground_truth))
    )

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
        # tf_remap,
        # tf_static_remap,
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
    name_arg = DeclareLaunchArgument(
        'name',
        default_value='robot',
        description='Robot name in Gazebo'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot',
        description='Robot namespace for ROS nodes and topics'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Robot spawn x position'
    )
    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Robot spawn y position'
    )
    z_pose_arg = DeclareLaunchArgument(
        'z_pose',
        default_value='0.0',
        description='Robot spawn z position'
    )
    declare_ground_truth_arg = DeclareLaunchArgument(
        'ground_truth',
        default_value='False',
        description='Use ground truth pose'
    )

    return LaunchDescription([
        name_arg,
        namespace_arg,
        use_sim_time_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        declare_ground_truth_arg,
        OpaqueFunction(function=include_launch_description)
    ])
