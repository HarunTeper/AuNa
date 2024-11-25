"""Single car omnet module launch file"""

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Return launch description"""

    # Launch Argument Configurations
    namespace = LaunchConfiguration('namespace')
    robot_index = LaunchConfiguration('robot_index')
    filter_index = LaunchConfiguration('filter_index')

    # Launch Arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot',
        description='Robot namespace for ROS nodes and topics'
    )
    robot_index_arg = DeclareLaunchArgument(
        'robot_index',
        default_value='0',
        description='Robot identifier index'
    )
    filter_index_arg = DeclareLaunchArgument(
        'filter_index',
        default_value='0',
        description='Robot cam filter index'
    )

    # Nodes and other launch files
    cam_communication_cmd = Node(
            package='auna_comm',
            executable='cam_communication',
            name='cam_communication',
            namespace=namespace,
            parameters=[{'filter_index': filter_index},
                        {'robot_index': robot_index}],
            output='screen'
        )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(namespace_arg)
    launch_description.add_action(robot_index_arg)
    launch_description.add_action(filter_index_arg)

    launch_description.add_action(cam_communication_cmd)
    return launch_description
