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
    mqtt_waypoint_receiver_cmd = Node(
            package='auna_mqtt',
            executable='mqtt_waypoint_receiver',
            name='mqtt_waypoint_receiver',
            namespace=namespace,
            parameters=[{'namespace': LaunchConfiguration('namespace')}],
            output='screen',
            remappings=[('/tf', 'tf'),
                        ('/tf_static', 'tf_static')]
        )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(namespace_arg)

    launch_description.add_action(mqtt_waypoint_receiver_cmd)
    return launch_description
