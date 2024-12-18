"""Spawn robot launch file"""

from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression


def generate_launch_description():
    """Return launch description"""

    # Launch Argument Configurations
    name = LaunchConfiguration('name')
    namespace = LaunchConfiguration('namespace')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    r_orientation = LaunchConfiguration('r_orientation')
    p_orientation = LaunchConfiguration('p_orientation')
    y_orientation = LaunchConfiguration('y_orientation')

    # Launch Arguments
    name_arg = DeclareLaunchArgument(
        'name',
        default_value='robot',
        description='Gazebo robot object name'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='ROS2 robot namespace (must not be empty)'
    )
    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Robot spawn x position'
    )
    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Robot spawn y position'
    )
    z_pose_arg = DeclareLaunchArgument(
        'z_pose',
        default_value='0.0',
        description='Robot spawn z position'
    )
    r_orientation_arg = DeclareLaunchArgument(
        'r_orientation',
        default_value='0.0',
        description='Robot spawn r orientation angle'
    )
    p_orientation_arg = DeclareLaunchArgument(
        'p_orientation',
        default_value='0.0',
        description='Robot spawn p orientation angle'
    )
    y_orientation_arg = DeclareLaunchArgument(
        'y_orientation',
        default_value='0.0',
        description='Robot spawn y orientation angle'
    )

    # Group spawner command with namespace handling
    spawner_group = GroupAction([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=[
                '-entity', name,
                '-topic', 'robot_description',
                '-x', x_pose,
                '-y', y_pose,
                '-z', z_pose,
                '-R', r_orientation,
                '-P', p_orientation,
                '-Y', y_orientation,
                '-robot_namespace', namespace,
            ],
            condition=IfCondition(PythonExpression(
                ["'", namespace, "' != ''"]))
        )
    ])

    return LaunchDescription([
        name_arg,
        namespace_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        r_orientation_arg,
        p_orientation_arg,
        y_orientation_arg,
        spawner_group
    ])
