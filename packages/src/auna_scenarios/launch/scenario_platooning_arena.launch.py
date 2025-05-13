import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Return launch description"""

    # --- Package Directories ---

    gazebo_pkg_dir = get_package_share_directory('auna_gazebo')
    comm_pkg_dir = get_package_share_directory('auna_comm')

    # --- Paths to Folders and Files ---

    gazebo_launch_file_dir = os.path.join(gazebo_pkg_dir, 'launch', 'gazebo')
    spawn_launch_file_dir = os.path.join(gazebo_pkg_dir, 'launch', 'spawn')
    comm_launch_file_dir = os.path.join(comm_pkg_dir, 'launch')

    # --- Launch Configuration Variables ---

    robot_number = LaunchConfiguration('robot_number', default='2')
    world_name = LaunchConfiguration('world_name', default='arena')
    namespace = LaunchConfiguration('namespace', default='robot')

    # --- Launch Arguments ---

    robot_number_arg = DeclareLaunchArgument(
        'robot_number',
        default_value='1',
        description='Number of spawned robots'
    )
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='arena',
        description='Gazebo world file name'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot',
        description='Namespace of the robot'
    )

    # --- Launch Description Actions ---

    world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            gazebo_launch_file_dir, 'gazebo_world.launch.py')),
        launch_arguments={
            'world_name': world_name
        }.items(),
    )
    spawn_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            spawn_launch_file_dir, 'spawn_multi_robot.launch.py')),
        launch_arguments={
            'robot_number': robot_number,
            'world_name': world_name,
            'namespace': namespace,
            'ground_truth': 'False'
        }.items(),
    )
    comm_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            comm_launch_file_dir, 'multi_cam_communication.launch.py')),
        launch_arguments={
            'robot_number': robot_number,
            'namespace': namespace,
        }.items(),
    )

    # --- Launch Description ---

    launch_description = LaunchDescription()

    launch_description.add_action(robot_number_arg)
    launch_description.add_action(world_name_arg)
    launch_description.add_action(namespace_arg)

    launch_description.add_action(world_cmd)
    launch_description.add_action(spawn_cmd)
    # launch_description.add_action(comm_cmd)

    return launch_description
