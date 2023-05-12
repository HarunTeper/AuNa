"""Single car spawn launch file"""

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression



def generate_launch_description():
    """Return launch description"""

    # Package Directories
    pkg_dir = get_package_share_directory('auna_gazebo')

    # Paths to folders and files
    launch_file_dir = os.path.join(pkg_dir, 'launch', 'spawn')
    gazebo_pose_launch_file_dir = os.path.join(pkg_dir, 'launch', 'pose')

    # Launch Argument Configurations
    name = LaunchConfiguration('name')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    ground_truth = LaunchConfiguration('ground_truth')

    # Launch Arguments
    name_arg = DeclareLaunchArgument(
        'name',
        default_value='robot',
        description='Robot name in Gazebo'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot',
        description='Robot namespace for ROS nodes and topics'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
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
    ground_truth_arg = DeclareLaunchArgument(
        'ground_truth',
        default_value='False',
        description='Use ground truth pose'
    )

    # Nodes and other launch files
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'spawn_robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'namespace': namespace}.items()
    )
    spawn_car_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'spawn_car.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose, 'z_pose': z_pose, 'namespace': namespace, 'name': name}.items()
    )
    localization_pose_cmd = Node(
        package='auna_gazebo',
        executable='localization_pose',
        name='localization_pose',
        namespace=namespace,
        arguments={namespace},
        output='screen',
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')
        ]
    )
    simulation_pose_cmd = Node(
        package='auna_gazebo',
        executable='simulation_pose',
        name='simulation_pose',
        namespace=namespace,
        arguments={name},
        output='screen'
    )
    gazebo_pose_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_pose_launch_file_dir, 'gazebo_pose.launch.py')),
        launch_arguments={'namespace': namespace}.items()
    )
    gazebo_pose_group_cmd = GroupAction(
        condition=IfCondition(PythonExpression([ground_truth])),
        actions=[gazebo_pose_cmd]
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(name_arg)
    launch_description.add_action(namespace_arg)
    launch_description.add_action(use_sim_time_arg)
    launch_description.add_action(x_pose_arg)
    launch_description.add_action(y_pose_arg)
    launch_description.add_action(z_pose_arg)
    launch_description.add_action(ground_truth_arg)

    launch_description.add_action(robot_state_publisher_cmd)
    launch_description.add_action(spawn_car_cmd)
    launch_description.add_action(localization_pose_cmd)
    launch_description.add_action(simulation_pose_cmd)
    launch_description.add_action(gazebo_pose_group_cmd)

    return launch_description
