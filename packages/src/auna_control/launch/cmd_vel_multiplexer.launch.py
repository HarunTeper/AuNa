from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    param_file = LaunchConfiguration('param_file')
    topic_file = LaunchConfiguration('topic_file')
    namespace = LaunchConfiguration('namespace')

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

    cmd_vel_multiplexer_node = Node(
        package='auna_control',
        executable='cmd_vel_multiplexer_node',
        namespace=namespace,
        name='cmd_vel_multiplexer_node',
        output='screen',
        parameters=[param_file, {"topic_file": topic_file}],
    )

    return LaunchDescription([
        declare_param_file_arg,
        declare_topic_file_arg,
        declare_namespace_arg,
        cmd_vel_multiplexer_node,
    ])
