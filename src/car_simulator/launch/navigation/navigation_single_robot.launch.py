
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def include_navigation_description(context: LaunchContext):

    # Package Directories
    pkg_dir = get_package_share_directory('car_simulator')
    nav2_bringup_pkg_dir = get_package_share_directory('nav2_bringup')

    # Paths to folders and files
    nav_launch_file_dir = os.path.join(nav2_bringup_pkg_dir, 'launch')
    default_map_file = os.path.join(pkg_dir, 'maps', 'map.yaml'),
    default_params_file = os.path.join(pkg_dir,'config', 'nav2_params', 'nav2_params.yaml')
    default_rviz_config_file = os.path.join(pkg_dir, 'rviz','config_navigation_namespace.rviz')
    default_default_bt_xml_filename_file = os.path.join(get_package_share_directory('nav2_bt_navigator'),'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    # Launch Argument Configurations
    autostart = LaunchConfiguration('autostart', default='true')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename', default=default_default_bt_xml_filename_file)
    map = LaunchConfiguration('map', default=default_map_file)
    namespace = LaunchConfiguration('namespace', default='robot')
    params_file = LaunchConfiguration('params_file', default=default_params_file)
    rviz_config = LaunchConfiguration('rviz_config', default = default_rviz_config_file)
    slam = LaunchConfiguration('slam', default = 'False')
    use_namespace = LaunchConfiguration('use_namespace', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Nodes and other launch files
    bringup_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_launch_file_dir, 'bringup_launch.py')),
        launch_arguments={
            'autostart': autostart,
            'default_bt_xml_filename': default_bt_xml_filename,
            'map': map,
            'namespace': namespace,
            'params_file': params_file,
            'slam': slam,
            'use_namespace': use_namespace,
            'use_sim_time': use_sim_time,
        }.items(),
    )
    rviz_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_launch_file_dir, 'rviz_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'rviz_config': rviz_config,
            'use_namespace': use_namespace,
        }.items(),
    )
    
    # Nodes and other launch files
    cmds = []

    cmds.append(bringup_launch_cmd)
    cmds.append(rviz_launch_cmd)

    return cmds

def generate_launch_description():

    # # Package Directories
    pkg_dir = get_package_share_directory('car_simulator')

    # Paths to folders and files
    default_map_file = os.path.join(pkg_dir, 'maps', 'racetrack_decorated', 'map.yaml'),
    default_params_file = os.path.join(pkg_dir,'config', 'nav2_params', 'nav2_params.yaml')
    default_rviz_config_file = os.path.join(pkg_dir, 'rviz','config_navigation_namespace.rviz')
    default_default_bt_xml_filename_file = os.path.join(get_package_share_directory('nav2_bt_navigator'),'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    # Launch Arguments
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the stacks'
    )
    default_bt_xml_filename_arg = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=default_default_bt_xml_filename_file,
        description='Full path to the behavior tree xml file to use'
    )
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map file to load'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot',
        description='Robot namespace for ROS nodes and topics'
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameter file for navigation nodes'
    ) 
    rviz_config_arg = DeclareLaunchArgument(
        name='rviz_config',
        default_value=default_rviz_config_file,
        description='Absolute path to rviz config file'
    )
    slam_arg = DeclareLaunchArgument(
        name='slam',
        default_value='False',
        description='Launch SLAM node to generate a map during navigation'
    )
    use_namespace_arg = DeclareLaunchArgument(
        name='use_namespace',
        default_value='true',
        description='Use namespace for navigation nodes'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    ld = LaunchDescription()

    ld.add_action(autostart_arg)
    ld.add_action(default_bt_xml_filename_arg)
    ld.add_action(map_arg)
    ld.add_action(namespace_arg)
    ld.add_action(params_file_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(slam_arg)
    ld.add_action(use_namespace_arg)
    ld.add_action(use_sim_time_arg)

    ld.add_action(OpaqueFunction(function=include_navigation_description))

    return ld