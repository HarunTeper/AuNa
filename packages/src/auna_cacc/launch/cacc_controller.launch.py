import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, GroupAction
from launch_ros.actions import PushRosNamespace
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration


def include_launch_description(context: LaunchContext):
    """Return launch description"""

    # Launch Argument Configurations
    robot_number = LaunchConfiguration('robot_number', default='2')
    namespace = LaunchConfiguration('namespace', default='robot')

    # Get resolved values
    ns_value = context.perform_substitution(namespace)
    robot_number_value = int(context.perform_substitution(robot_number))

    launch_description_content = []

    # Explicitly log the number of robots detected
    launch_description_content.append(
        LogInfo(
            msg=f"CACC launch: Detected {robot_number_value} robots with base namespace '{ns_value}'")
    )

    # Only start CACC controllers if there is more than one robot
    if robot_number_value <= 1:
        launch_description_content.append(
            LogInfo(
                msg="CACC controllers will NOT be started - CACC requires at least 2 robots")
        )
        return launch_description_content

    # If we have multiple robots, start CACC controllers ONLY for follower robots (not the lead robot)
    launch_description_content.append(
        LogInfo(
            msg=f"Starting CACC controllers for {robot_number_value-1} follower robots")
    )

    # Start from 1 (skip robot0) since only follower robots need CACC controllers
    for num in range(1, robot_number_value):
        # Create namespace string properly by concatenating the resolved namespace value
        robot_ns = f"{ns_value}{num}"

        # Log explicitly which robot we're starting a controller for
        launch_description_content.append(
            LogInfo(
                msg=f"Starting CACC controller for {robot_ns} (follower robot)")
        )

        # Use PushRosNamespace within a GroupAction to properly namespace the node
        launch_description_content.append(
            GroupAction([
                PushRosNamespace(robot_ns),
                Node(
                    package='auna_cacc',
                    executable='cacc_controller',
                    name='cacc_controller',
                    output='screen'
                )
            ])
        )

    # Log that we're not starting a controller for the lead robot
    launch_description_content.append(
        LogInfo(msg=f"No CACC controller started for {ns_value}0 (lead robot)")
    )

    return launch_description_content


def generate_launch_description():
    """Return launch description"""

    # Add additional logging at the beginning
    initial_log = LogInfo(
        msg="Initializing CACC controller launch - will only start for follower robots"
    )

    # Launch Arguments
    robot_number_arg = DeclareLaunchArgument(
        'robot_number',
        default_value='2',
        description='Number of spawned robots'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot',
        description='Namespace for spawned robots'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(initial_log)
    launch_description.add_action(robot_number_arg)
    launch_description.add_action(namespace_arg)

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
