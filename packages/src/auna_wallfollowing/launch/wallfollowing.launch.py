from launch_ros.actions import Node, PushRosNamespace
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext


def include_launch_description(context: LaunchContext):
    """Return launch description"""

    launch_description_content = []

    # Get robot_index and construct namespace
    robot_index = context.launch_configurations.get('robot_index', '0')
    namespace = f"robot{robot_index}"

    wallfollowing_node = Node(
        package='auna_wallfollowing',
        executable='wallfollowing',
        name='wallfollowing_node',
        output='screen',
        parameters=[context.launch_configurations.get('cacc_config', {})],
    )

    push_ns = PushRosNamespace(namespace)
    launch_description_content.append(push_ns)

    # Add the wallfollowing node to the launch description content
    launch_description_content.append(wallfollowing_node)

    return launch_description_content   


def generate_launch_description():
    """Return launch description"""
    # Launch Description
    launch_description = LaunchDescription()

    # Declare robot_index argument
    launch_description.add_action(DeclareLaunchArgument(
        'robot_index',
        default_value='0',
        description='Index of the robot to namespace the node.'
    ))

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
