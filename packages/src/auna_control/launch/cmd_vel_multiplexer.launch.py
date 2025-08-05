from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    param_file = LaunchConfiguration('param_file')
    topic_file = LaunchConfiguration('topic_file')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    default_param_file = PathJoinSubstitution([
        FindPackageShare('auna_control'),
        'config',
        'params.yaml'
    ])
    default_topic_file = PathJoinSubstitution([
        FindPackageShare('auna_control'),
        'config',
        'topics.yaml'
    ])

    declare_param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=default_param_file,
        description='Path to the cmd_vel_multiplexer parameter YAML file'
    )

    declare_topic_file_arg = DeclareLaunchArgument(
        'topic_file',
        default_value=default_topic_file,
        description='Path to the cmd_vel_multiplexer topic YAML file'
    )

    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the node (empty for none)'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    cmd_vel_multiplexer_node = Node(
        package='auna_control',
        executable='cmd_vel_multiplexer_node',
        namespace=namespace,
        name='cmd_vel_multiplexer_node',
        output='screen',
        parameters=[param_file, {"topic_file": topic_file}, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_param_file_arg,
        declare_topic_file_arg,
        declare_namespace_arg,
        declare_use_sim_time_arg,
        cmd_vel_multiplexer_node,
    ])
