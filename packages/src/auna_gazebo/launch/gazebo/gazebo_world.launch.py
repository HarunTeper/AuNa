"""Gazebo world launch file"""
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument, AppendEnvironmentVariable
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_launch_description(context: LaunchContext):
    """Return launch description"""
    # Package Directories
    pkg_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_auna_gazebo = get_package_share_directory('auna_gazebo')

    # Construct world path using resolved context
    world = os.path.join(pkg_auna_gazebo, 'worlds', str(
        context.launch_configurations['world_name'])+'.world')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-s -r -v4 ', world],
            'on_exit_shutdown': 'true',
        }.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-g -v4 ']
        }.items()
    )

    robot_name_publisher_cmd = Node(
        package='auna_gazebo',
        executable='robot_name_publisher',
        name='robot_name_publisher',
        output='screen'
    )

    return [
        gzserver_cmd,
        gzclient_cmd,
        robot_name_publisher_cmd,
    ]


def generate_launch_description():
    """Return launch description"""

    world_arg = DeclareLaunchArgument(
        'world_name',
        default_value='racetrack_decorated',
        description='Gazebo world file name in /worlds folder'
    )

    gazebo_state_arg = DeclareLaunchArgument(
        'gazebo_ros_state',
        default_value='true',
        description='Enable/disable Gazebo ROS state'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Enable/disable Gazebo GUI'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Set up Gazebo resource path to find model files
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('auna_gazebo'), 'models')
    )

    # Also set the IGN_GAZEBO_RESOURCE_PATH for newer Gazebo versions
    set_env_vars_ign_resources = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        os.path.join(get_package_share_directory('auna_gazebo'), 'models')
    )

    # Set GAZEBO_MODEL_PATH for model discovery
    set_env_vars_model_path = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.join(get_package_share_directory('auna_gazebo'), 'models')
    )

    return LaunchDescription([
        world_arg,
        gazebo_state_arg,
        gui_arg,
        use_sim_time_arg,
        set_env_vars_resources,
        set_env_vars_ign_resources,
        set_env_vars_model_path,
        OpaqueFunction(function=include_launch_description)
    ])
