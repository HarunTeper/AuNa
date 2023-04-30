"""Spawn car launch file"""

from launch_ros.actions import Node, SetRemap
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch import LaunchDescription



def generate_launch_description():
    """Return launch description"""

    # Launch Argument Configurations
    name = LaunchConfiguration('name')
    namespace = LaunchConfiguration('namespace')

    # Launch Arguments
    name_arg = DeclareLaunchArgument(
        'name',
        default_value='robot',
        description='Gazebo robot object name'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='ROS2 robot namespace'
    )

    group_cmd = GroupAction([
        SetRemap(src='/tf',dst='tf'),
        SetRemap(src='/tf_static',dst='tf_static'),
        Node(
            package='auna_gazebo',
            executable='gazebo_pose',
            name='gazebo_pose',
            namespace=namespace,
            output='screen',
            arguments={name},
        )
    ])

    # Nodes and other launch files
    

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(name_arg)
    launch_description.add_action(namespace_arg)

    launch_description.add_action(group_cmd)

    return launch_description
