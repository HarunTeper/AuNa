import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_context import LaunchContext
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from auna_common import yaml_launch


def include_launch_description(context: LaunchContext):
    """Return launch description"""

    # Package Directories
    pkg_dir = get_package_share_directory('auna_nav2')

    # Paths to folders and files
    nav_launch_file_dir = os.path.join(pkg_dir, 'launch')

    # Launch Argument Configurations
    autostart = LaunchConfiguration('autostart')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_file = LaunchConfiguration('map')
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_slam = LaunchConfiguration('enable_slam')
    enable_localization = LaunchConfiguration('enable_localization')
    enable_navigation = LaunchConfiguration('enable_navigation')
    enable_rviz = LaunchConfiguration('enable_rviz')
    enable_map_server = LaunchConfiguration('enable_map_server')

    tmp_params_file = yaml_launch.get_yaml(params_file.perform(context))
    tmp_params_file = yaml_launch.insert_namespace(tmp_params_file, context.launch_configurations['namespace'])
    tmp_params_file = yaml_launch.get_temp_file(tmp_params_file)

    # Nodes and other launch files

    cmd_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_launch_file_dir, 'bringup.launch.py')),
            launch_arguments={
                'autostart': autostart,
                'default_bt_xml_filename': default_bt_xml_filename,
                'map': map_file,
                'namespace': namespace,
                'params_file': tmp_params_file,
                'use_sim_time': use_sim_time,
                'enable_slam': enable_slam,
                'enable_localization': enable_localization,
                'enable_navigation': enable_navigation,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_launch_file_dir, 'map_server.launch.py')),
            condition=IfCondition(enable_map_server),
            launch_arguments={
                'namespace': namespace,
                'map': map_file,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_launch_file_dir, 'rviz.launch.py')),
            condition=IfCondition(enable_rviz),
            launch_arguments={
                'namespace': namespace,
                'rviz_config': rviz_config,
            }.items(),
        )
    ])

    # Nodes and other launch files
    cmds = []

    cmds.append(cmd_group)

    return cmds


def generate_launch_description():
    """Return launch description"""

    # # Package Directories
    pkg_dir = get_package_share_directory('auna_nav2')

    # Paths to folders and files
    default_map_file = os.path.join(pkg_dir, 'maps', 'racetrack_decorated', 'map.yaml'),
    default_params_file = os.path.join(pkg_dir, 'config', 'nav2_params', 'nav2_params.yaml')
    default_rviz_config_file = os.path.join(pkg_dir, 'rviz', 'config_navigation_namespace.rviz')
    default_default_bt_xml_filename_file = os.path.join(get_package_share_directory('nav2_bt_navigator'), 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

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
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_enable_slam_arg = DeclareLaunchArgument(
        name='enable_slam',
        default_value='False',
        description='Enable SLAM'
    )
    use_enable_localization_arg = DeclareLaunchArgument(
        name='enable_localization',
        default_value='True',
        description='Enable Localization'
    )
    use_enable_navigation_arg = DeclareLaunchArgument(
        name='enable_navigation',
        default_value='True',
        description='Enable Navigation'
    )
    use_enable_rviz_arg = DeclareLaunchArgument(
        name='enable_rviz',
        default_value='True',
        description='Enable RViz'
    )
    use_enable_map_server_arg = DeclareLaunchArgument(
        name='enable_map_server',
        default_value='True',
        description='Enable Map Server'
    )

    launch_description = LaunchDescription()

    launch_description.add_action(autostart_arg)
    launch_description.add_action(default_bt_xml_filename_arg)
    launch_description.add_action(map_arg)
    launch_description.add_action(namespace_arg)
    launch_description.add_action(params_file_arg)
    launch_description.add_action(rviz_config_arg)
    launch_description.add_action(use_sim_time_arg)
    launch_description.add_action(use_enable_slam_arg)
    launch_description.add_action(use_enable_localization_arg)
    launch_description.add_action(use_enable_navigation_arg)
    launch_description.add_action(use_enable_rviz_arg)
    launch_description.add_action(use_enable_map_server_arg)

    launch_description.add_action(OpaqueFunction(function=include_launch_description))

    return launch_description
