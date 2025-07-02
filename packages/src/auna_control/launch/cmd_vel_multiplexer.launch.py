from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = LaunchConfiguration('config_file')
    namespace = LaunchConfiguration('namespace')
    default_config = PathJoinSubstitution([
        FindPackageShare('auna_control'),
        'config',
        'params.yaml'
    ])

    declare_config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the cmd_vel_multiplexer parameter YAML file'
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
        parameters=[config_path],
    )

    return LaunchDescription([
        declare_config_arg,
        declare_namespace_arg,
        cmd_vel_multiplexer_node,
    ])
