"""Multiple cars omnet module launch file"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, GroupAction
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_launch_description(context: LaunchContext):
    """Return launch description"""

    # Check if the comm type is omnet
    if os.environ.get('AUNA_COMM_TYPE') != 'auna_omnet':
        return []

    # Package Directories
    pkg_dir = get_package_share_directory('auna_omnet')

    # Paths to folders and files
    omnet_launch_file_dir = os.path.join(pkg_dir, 'launch')

    # Launch Argument Configurations
    robot_number = LaunchConfiguration('robot_number', default='2')

    # Nodes and other launch files
    launch_description_content = []

    for num in range(int(robot_number.perform(context))):
        launch_description_content.append(
            GroupAction([
                PushRosNamespace(f'robot{num}'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(omnet_launch_file_dir,
                                     'omnet_single_robot_modules.launch.py')
                    )
                )
            ])
        )

    return launch_description_content


def generate_launch_description():
    """Return launch description"""

    # Launch Arguments
    robot_number_arg = DeclareLaunchArgument(
        'robot_number',
        default_value='2',
        description='Number of spawned robots'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(robot_number_arg)

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
