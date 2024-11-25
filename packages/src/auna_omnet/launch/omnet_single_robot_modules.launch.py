"""Single car omnet module launch file"""

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Return launch description"""

    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace')

    # Launch Arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace for ROS nodes and topics'
    )

    # Nodes and other launch files
    omnet_transmitter_cmd = Node(
            package='auna_omnet',
            executable='omnet_transmitter',
            name='omnet_transmitter',
            namespace=namespace,
            arguments={namespace},
            output='screen'
        )

    omnet_receiver_cmd = Node(
            package='auna_omnet',
            executable='omnet_receiver',
            name='omnet_receiver',
            namespace=namespace,
            arguments={namespace},
            output='screen'
        )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(namespace_arg)

    launch_description.add_action(omnet_transmitter_cmd)
    launch_description.add_action(omnet_receiver_cmd)
    return launch_description
