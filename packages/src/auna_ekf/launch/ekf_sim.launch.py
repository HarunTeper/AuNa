# packages/src/auna_ekf/launch/ekf.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext

def include_launch_description(context: LaunchContext):

    """Return launch description"""
    # Launch Configurations
    robot_index = LaunchConfiguration('robot_index')

    # Individual components
    tf_remap = SetRemap(src='/tf', dst='tf')
    tf_static_remap = SetRemap(src='/tf_static', dst='tf_static')
    
    auna_ekf_pkg_share = get_package_share_directory('auna_ekf') # Modified
    ekf_local_config_path = os.path.join(
        auna_ekf_pkg_share, 'config', 'ekf', 'ekf_local.yaml') # Modified

    use_sim_time = LaunchConfiguration('use_sim_time')
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_local_config_path,
            {'use_sim_time': use_sim_time}
        ],
    )

    # Namespace group for proper topic scoping
    ekf_group = GroupAction([
        PushRosNamespace('robot' + robot_index.perform(context)),
        tf_remap,
        tf_static_remap,
        ekf_node
    ])
    
    launch_actions = []
    launch_actions.append(ekf_group)
    return launch_actions
    
def generate_launch_description():
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    return LaunchDescription([
        declare_use_sim_time_arg,
        OpaqueFunction(function=include_launch_description)
    ])