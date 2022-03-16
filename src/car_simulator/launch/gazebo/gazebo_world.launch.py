import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def include_world_description(context : LaunchContext):

    # Package Directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Paths to folders and files
    world = os.path.join(get_package_share_directory('car_simulator'),'worlds',str(context.launch_configurations['world_name'])+'.world')

    state = LaunchConfiguration('gazebo_ros_state', default='true')
    
    # Nodes and other launch files
    gazebo_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world,
        'gazebo_ros_state': state,
        }.items()
    )

    # Create launch description actions
    world_description = []
    world_description.append(gazebo_launch_cmd)

    return world_description

def generate_launch_description():

    # Launch Arguments
    world_arg = DeclareLaunchArgument(
        'world_name',
        default_value='racetrack_decorated',
        description='Gazebo world file name in /worlds folder'
    )

    # Launch Description
    ld = LaunchDescription()

    ld.add_action(world_arg)

    ld.add_action(OpaqueFunction(function=include_world_description))

    return ld
