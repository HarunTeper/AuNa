# packages/src/auna_gazebo/launch/ekf/localization_ekf.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    auna_gazebo_pkg_share = get_package_share_directory('auna_gazebo')
    default_ekf_config_path = os.path.join(
        auna_gazebo_pkg_share, 'config', 'ekf', 'ekf.yaml')

    print(
        f"[localization_ekf.launch.py] Using EKF config path: {default_ekf_config_path}")

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
    declare_ekf_config_file_arg = DeclareLaunchArgument(
        'ekf_config_file',
        default_value=default_ekf_config_path,
        description='Full path to EKF configuration file'
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

    use_sim_time = LaunchConfiguration('use_sim_time')
    ekf_config_file = LaunchConfiguration('ekf_config_file')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('odometry/filtered', 'odometry/filtered_ekf')
        ]
    )

    return LaunchDescription([
        declare_namespace_arg,
        declare_use_sim_time_arg,
        declare_ekf_config_file_arg,
        declare_use_gps_arg,
        declare_use_imu_arg,
        declare_use_odom_arg,
        ekf_node
    ])
