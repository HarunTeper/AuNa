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
    ekf_odom_config_path = os.path.join(
        auna_gazebo_pkg_share, 'config', 'ekf', 'ekf_odom.yaml')
    ekf_map_config_path = os.path.join(
        auna_gazebo_pkg_share, 'config', 'ekf', 'ekf_map.yaml')

    # Validate odom config file exists
    if not os.path.exists(ekf_odom_config_path):
        print(
            f"[ERROR] EKF odom config file not found at: {ekf_odom_config_path}")
    else:
        print(f"[INFO] EKF odom config file found: {ekf_odom_config_path}")

    # Validate map config file exists
    if not os.path.exists(ekf_map_config_path):
        print(
            f"[ERROR] EKF map config file not found at: {ekf_map_config_path}")
    else:
        print(f"[INFO] EKF map config file found: {ekf_map_config_path}")

    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the robot'
    )
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
    # Placeholder arguments for future dynamic sensor enabling/disabling logic
    declare_use_gps_arg = DeclareLaunchArgument(
        'use_gps', default_value='true', description='Placeholder: Enable GPS input (managed by EKF YAML)'
    )
    declare_use_imu_arg = DeclareLaunchArgument(
        'use_imu', default_value='true', description='Placeholder: Enable IMU input (managed by EKF YAML)'
    )
    declare_use_odom_arg = DeclareLaunchArgument(
        'use_odom', default_value='true', description='Placeholder: Enable wheel odometry input (managed by EKF YAML)'
    )

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    debug_ekf = LaunchConfiguration('debug_ekf')

    # EKF node for odom -> base_link
    # Frame names in ekf_odom.yaml (odom, base_link) will be automatically namespaced by PushRosNamespace
    ekf_node_odom = Node(
        executable='/home/vscode/workspace/packages/install/robot_localization/lib/robot_localization/ekf_node',
        name='ekf_filter_node_odom',  # Unique name
        output='screen',
        parameters=[
            ekf_odom_config_path,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('odometry/filtered', 'odometry/filtered_odom'),  # Unique output topic
            # Input topics (imu, odom) will be namespaced by PushRosNamespace if relative in YAML
        ],
        arguments=[
            '--ros-args',
            '--log-level',
            PythonExpression(['"ekf_filter_node_odom:=debug" if "',
                             debug_ekf, '" == "true" else "ekf_filter_node_odom:=info"']),
            '--log-level',
            PythonExpression(['"robot_localization.ekf_filter_node_odom:=debug" if "',
                              debug_ekf, '" == "true" else "robot_localization.ekf_filter_node_odom:=info"'])
        ]
    )

    # EKF node for map -> odom
    # map_frame and world_frame: map are global. odom_frame and base_link_frame from YAML will be namespaced.
    ekf_node_map = Node(
        executable='/home/vscode/workspace/packages/install/robot_localization/lib/robot_localization/ekf_node',
        name='ekf_filter_node_map',  # Unique name
        output='screen',
        parameters=[
            ekf_map_config_path,
            {'use_sim_time': use_sim_time},
            {'map_frame': 'map'},  # Explicitly global
            {'world_frame': 'map'},  # Explicitly global
        ],
        remappings=[
            ('odometry/filtered', 'odometry/filtered_map'),  # Unique output topic
            # Input topics (imu, gps/fix) will be namespaced by PushRosNamespace if relative in YAML
        ],
        arguments=[
            '--ros-args',
            '--log-level',
            PythonExpression(['"ekf_filter_node_map:=debug" if "',
                             debug_ekf, '" == "true" else "ekf_filter_node_map:=info"']),
            '--log-level',
            PythonExpression(['"robot_localization.ekf_filter_node_map:=debug" if "',
                              debug_ekf, '" == "true" else "robot_localization.ekf_filter_node_map:=info"'])
        ]
    )

    # Topic monitoring node to debug inputs
    topic_monitor = ExecuteProcess(
        cmd=['ros2', 'topic', 'list'],
        output='screen',
        condition=IfCondition(debug_ekf)
    )

    # EKF status monitoring
    ekf_status_monitor = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'echo', '--once',
                     PythonExpression(['"/', namespace, '/odometry/filtered_map" if "', namespace, '" != "" else "/odometry/filtered_map"'])],  # Monitor final map output
                output='screen',
                condition=IfCondition(debug_ekf)
            )
        ]
    )

    # Namespace group for proper topic scoping
    ekf_group = GroupAction([
        ekf_node_odom,
        ekf_node_map,
        topic_monitor,
        ekf_status_monitor
    ])

    return LaunchDescription([
        declare_namespace_arg,
        declare_use_sim_time_arg,
        # declare_ekf_config_file_arg, # Removed as we now have two specific configs
        declare_debug_ekf_arg,
        declare_use_gps_arg,
        declare_use_imu_arg,
        declare_use_odom_arg,
        ekf_group
    ])
