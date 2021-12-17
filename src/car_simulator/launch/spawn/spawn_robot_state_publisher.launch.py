import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration,Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():

    # Package Directories
    pkg_dir = get_package_share_directory('car_simulator')

    # Paths to folders and files
    default_model_path = os.path.join(pkg_dir, 'models/race_car/model.urdf')

    # Launch Argument Configurations
    model = LaunchConfiguration('model', default=default_model_path)
    namespace = LaunchConfiguration('namespace', default='')
    urdf_namespace = LaunchConfiguration('urdf_namespace', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

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
    urdf_namespace_arg = DeclareLaunchArgument(
        name='urdf_namespace',
        default_value='',
        description='Robot namespace for urdf. If namespace is empty, leave empty. Else, set to namespace but with a / at the end'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    remapping_tf = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Nodes and other launch files
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[
            {'robot_description': Command([
                'xacro ', model, 
                ' namespace:=', urdf_namespace])},
            {'use_sim_time':use_sim_time},
        ],
        remappings=remapping_tf
    )

    # Launch Description
    ld = LaunchDescription()

    ld.add_action(model_arg)
    ld.add_action(namespace_arg)
    ld.add_action(urdf_namespace_arg)
    ld.add_action(use_sim_time_arg)

    ld.add_action(robot_state_publisher_node)

    return ld
