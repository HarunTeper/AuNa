"""Localization pose publisher launch file"""
from launch_ros.actions import Node, SetRemap
from launch.actions import GroupAction, DeclareLaunchArgument, OpaqueFunction
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.launch_context import LaunchContext

def include_launch_description(context: LaunchContext):
    """Return launch description"""
    # Launch Configurations
    robot_index = LaunchConfiguration('robot_index')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Individual components
    tf_remap = SetRemap(src='/tf', dst='tf')
    tf_static_remap = SetRemap(src='/tf_static', dst='tf_static')

    localization_pose_publisher = Node(
        package='auna_tf',
        executable='localization_pose_publisher',
        name='localization_pose_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    group_cmd = GroupAction([
        PushRosNamespace('robot' + robot_index.perform(context)),
        tf_remap,
        tf_static_remap,
        localization_pose_publisher
    ])
    
    
    launch_actions = []
    launch_actions.append(group_cmd)
    return launch_actions

def generate_launch_description():
    """Return launch description"""

    # Launch Arguments
    robot_index_arg = DeclareLaunchArgument(
        'robot_index',
        default_value='1',
        description='Index of the robot to spawn'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    return LaunchDescription([
        robot_index_arg,
        use_sim_time_arg,
        OpaqueFunction(function=include_launch_description)
    ])
