import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    # Get launch arguments
    map_type = LaunchConfiguration('map_type').perform(context)
    robot_number = LaunchConfiguration('robot_number').perform(context)
    nav2 = LaunchConfiguration('nav2').perform(context)
    communication = LaunchConfiguration('communication').perform(context)
    cacc = LaunchConfiguration('cacc').perform(context)
    cacc_waypoints = LaunchConfiguration('cacc_waypoints').perform(context)

    # Log the launch arguments
    print(f"map_type: {map_type}")
    print(f"robot_number: {robot_number}")
    print(f"nav2: {nav2}")
    print(f"communication: {communication}")
    print(f"cacc: {cacc}")
    print(f"cacc_waypoints: {cacc_waypoints}")

    # Enforce constraints
    if cacc_waypoints.lower() == 'false' and nav2.lower() == 'false':
        raise ValueError("If cacc_waypoints is 'false', nav2 must be 'true'")

    # Package Directories
    gazebo_pkg_dir = get_package_share_directory('auna_gazebo')
    nav2_pkg_dir = get_package_share_directory('auna_nav2')
    comm_pkg_dir = get_package_share_directory('auna_comm')
    omnet_pkg_dir = get_package_share_directory('auna_omnet')
    cacc_pkg_dir = get_package_share_directory('auna_cacc')

    # Paths to folders and files
    gazebo_launch_file_dir = os.path.join(gazebo_pkg_dir, 'launch', 'gazebo')
    spawn_launch_file_dir = os.path.join(gazebo_pkg_dir, 'launch', 'spawn')
    nav2_launch_file_dir = os.path.join(nav2_pkg_dir, 'launch')
    comm_launch_file_dir = os.path.join(comm_pkg_dir, 'launch')
    omnet_launch_file_dir = os.path.join(omnet_pkg_dir, 'launch')
    cacc_launch_file_dir = os.path.join(cacc_pkg_dir, 'launch')

    # Launch description content
    launch_description_content = []

    # Gazebo
    launch_description_content.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                gazebo_launch_file_dir, 'gazebo_world.launch.py')),
            launch_arguments={'world_name': map_type}.items(),
        )
    )

    # Robot Spawning
    launch_description_content.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                spawn_launch_file_dir, 'spawn_multi_robot.launch.py')),
            launch_arguments={
                'robot_number': robot_number,
                'world_name': map_type,
                'namespace': 'robot',
                'ground_truth': 'False'
            }.items(),
        )
    )

    # Nav2
    if nav2.lower() == 'true':
        launch_description_content.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    nav2_launch_file_dir, 'navigation_multi_robot.launch.py')),
                launch_arguments={
                    'namespace': 'robot',
                    'robot_number': robot_number,
                    'world_name': map_type,
                    'enable_slam': 'False',
                    'enable_localization': 'True',
                    'enable_navigation': 'True',
                    'enable_rviz': 'True',
                    'enable_map_server': 'True',
                }.items(),
            )
        )

    # Communication
    if communication == 'omnet':
        launch_description_content.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    omnet_launch_file_dir, 'omnet_multi_robot_modules.launch.py')),
                launch_arguments={'robot_number': robot_number}.items(),
            )
        )
        for i in range(int(robot_number)):
            launch_description_content.append(
                Node(
                    package='auna_omnet',
                    executable='omnet_cam_filter',
                    name='omnet_cam_filter',
                    namespace=f'robot{i}',
                    arguments=[str(i)],
                    output='screen'
                )
            )
    elif communication == 'auna_comm':
        launch_description_content.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    comm_launch_file_dir, 'multi_cam_communication.launch.py')),
                launch_arguments={'robot_number': robot_number}.items(),
            )
        )

    # CACC
    if cacc.lower() == 'true':
        launch_description_content.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    cacc_launch_file_dir, 'multi_cacc_controller.launch.py')),
                launch_arguments={
                    'robot_number': robot_number,
                    'use_waypoints': cacc_waypoints
                }.items(),
            )
        )

    return launch_description_content


def generate_launch_description():
    """Return launch description"""

    # Launch Arguments
    map_type_arg = DeclareLaunchArgument(
        'map_type',
        default_value='racetrack_decorated',
        description='Map type to use'
    )
    robot_number_arg = DeclareLaunchArgument(
        'robot_number',
        default_value='2',
        description='Number of spawned robots'
    )
    nav2_arg = DeclareLaunchArgument(
        'nav2',
        default_value='true',
        description='Enable Nav2 for the leader robot'
    )
    communication_arg = DeclareLaunchArgument(
        'communication',
        default_value='auna_comm',
        description='Communication system to use (omnet or auna_comm)'
    )
    cacc_arg = DeclareLaunchArgument(
        'cacc',
        default_value='false',
        description='Enable CACC system'
    )
    cacc_waypoints_arg = DeclareLaunchArgument(
        'cacc_waypoints',
        default_value='true',
        description='Enable waypoints for CACC'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(map_type_arg)
    launch_description.add_action(robot_number_arg)
    launch_description.add_action(nav2_arg)
    launch_description.add_action(communication_arg)
    launch_description.add_action(cacc_arg)
    launch_description.add_action(cacc_waypoints_arg)

    launch_description.add_action(OpaqueFunction(function=launch_setup))

    return launch_description
