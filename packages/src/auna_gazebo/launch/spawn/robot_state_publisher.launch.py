"""Robot state publisher launch file"""

import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_context import LaunchContext


def include_launch_description(context: LaunchContext):
    """Return launch description"""

    # Launch Argument Configurations
    model = LaunchConfiguration('model')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Configures the tf tree frames to include the namespace at the beginning, if the robot has a namespace
    urdf_namespace = [str(context.launch_configurations['namespace'])+'/', ''][str(context.launch_configurations['namespace']) == '']

    remapping_tf = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')
    ]

    # Nodes and other launch files
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'robot_description':
                Command([
                    'xacro ', model,
                    ' namespace:=', urdf_namespace]),
                'use_sim_time': use_sim_time,
                'use_tf_static' : False
            },
        ],
        remappings=remapping_tf
    )

    # Create launch description actions
    launch_description_content = []
    launch_description_content.append(robot_state_publisher_node)

    return launch_description_content


def generate_launch_description():
    """Return launch description"""

    # Package Directories
    pkg_dir = get_package_share_directory('auna_gazebo')

    # Paths to folders and files
    default_model_path = os.path.join(pkg_dir, 'models/race_car/model.urdf')

    # Launch Arguments
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf file'
    )
    namespace_arg = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Robot namespace'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(model_arg)
    launch_description.add_action(namespace_arg)
    launch_description.add_action(use_sim_time_arg)

    launch_description.add_action(OpaqueFunction(function=include_launch_description))

    return launch_description
