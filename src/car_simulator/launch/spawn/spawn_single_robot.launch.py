import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():

    # Package Directories
    pkg_dir = get_package_share_directory('car_simulator')

    # Paths to folders and files
    launch_file_dir = os.path.join(pkg_dir, 'launch', 'spawn')

    # Launch Argument Configurations
    name = LaunchConfiguration('name', default='robot')
    namespace = LaunchConfiguration('namespace', default='robot')
    urdf_namespace = LaunchConfiguration('urdf_namespace', default='robot/')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.0')

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
    urdf_namespace_arg = DeclareLaunchArgument(
        'urdf_namespace',
        default_value='robot/',
        description='Robot namespace for urdf. If namespace is empty, leave empty. Else, set to namespace but with a / at the end'
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

    # Nodes and other launch files
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'spawn_robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,'namespace':namespace, 'urdf_namespace':urdf_namespace}.items()
    )
    spawn_car_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'spawn_car.launch.py')),
        launch_arguments={'x_pose': x_pose,'y_pose': y_pose,'z_pose': z_pose, 'namespace':namespace, 'name':name}.items()
    )
    localization_pose_cmd = Node(
        package='car_simulator',
        executable='localization_pose',
        name='localization_pose',
        namespace = urdf_namespace,
        arguments= {urdf_namespace},
        output='screen'
    )
    simulation_pose_cmd = Node(
        package='car_simulator',
        executable='simulation_pose',
        name='simulation_pose',
        namespace = namespace,
        arguments= {namespace},
        output='screen'
    )

    # Launch Description
    ld = LaunchDescription()

    ld.add_action(name_arg)
    ld.add_action(namespace_arg)
    ld.add_action(urdf_namespace_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(x_pose_arg)
    ld.add_action(y_pose_arg)
    ld.add_action(z_pose_arg)

    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_car_cmd)
    ld.add_action(localization_pose_cmd)
    ld.add_action(simulation_pose_cmd)

    return ld
