# packages/src/auna_gazebo/launch/ekf/localization_ekf.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition


def generate_launch_description():
    auna_gazebo_pkg_share = get_package_share_directory('auna_gazebo')
    ekf_local_config_path = os.path.join(
        auna_gazebo_pkg_share, 'config', 'ekf', 'ekf_local.yaml')
    ekf_global_config_path = os.path.join(
        auna_gazebo_pkg_share, 'config', 'ekf', 'ekf_global.yaml')
    navsat_config_path = os.path.join(
        auna_gazebo_pkg_share, 'config', 'ekf', 'navsat.yaml')

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_debug_ekf_arg = DeclareLaunchArgument(
        'debug_ekf',
        default_value='true',
        description='Enable EKF debugging output'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_node',  # Unique name
        output='screen',
        parameters=[
            ekf_local_config_path,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('odometry/filtered', 'odometry/local_odom'),  # Unique output topic
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_node',  # Unique name
        output='screen',
        parameters=[
            ekf_global_config_path,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('odometry/filtered', 'odometry/global_odom'),  # Unique output topic
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[
            navsat_config_path,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('odometry/filtered', 'odometry/local_odom'),
            ('odometry/gps', 'odometry/gps')
        ]
    )

    # Namespace group for proper topic scoping
    ekf_group = GroupAction([
        ekf_local_node,
        ekf_global_node,
        navsat_transform_node
    ])

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_debug_ekf_arg,
        ekf_group
    ])
