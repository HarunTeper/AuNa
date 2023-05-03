import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from auna_common import yaml_launch


def include_launch_description(context: LaunchContext):
    """Return launch description"""

    # Package Directories
    pkg_dir = get_package_share_directory('auna_nav2')
    gazebo_pkg_dir = get_package_share_directory('auna_gazebo')

    # Paths to folders and files
    nav_launch_file_dir = os.path.join(pkg_dir, 'launch')
    default_map_file = os.path.join(pkg_dir, 'maps', context.launch_configurations['world_name'], 'map.yaml'),

    # Launch Argument Configurations
    autostart = LaunchConfiguration('autostart')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_file = LaunchConfiguration('map', default=default_map_file)
    params_file_name = LaunchConfiguration('params_file_name')
    robot_number = LaunchConfiguration('robot_number')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world_name')
    enable_slam = LaunchConfiguration('enable_slam')
    enable_localization = LaunchConfiguration('enable_localization')
    enable_navigation = LaunchConfiguration('enable_navigation')
    enable_rviz = LaunchConfiguration('enable_rviz')
    enable_map_server = LaunchConfiguration('enable_map_server')

    # Names and poses of the robots
    map_path = os.path.join(gazebo_pkg_dir, "config", "map_params", world_name.perform(context)+".yaml")
    robots = []
    for num in range(int(robot_number.perform(context))):
        robots.append({
            'name': 'robot'+str(num),
            'namespace': 'robot'+str(num),
            'x_pose': yaml_launch.get_yaml_value(map_path, ["spawn", "offset", "x"])+num*yaml_launch.get_yaml_value(map_path, ["spawn", "linear", "x"]),
            'y_pose': yaml_launch.get_yaml_value(map_path, ["spawn", "offset", "y"])+num*yaml_launch.get_yaml_value(map_path, ["spawn", "linear", "y"]),
            'z_pose': yaml_launch.get_yaml_value(map_path, ["spawn", "offset", "z"])+num*yaml_launch.get_yaml_value(map_path, ["spawn", "linear", "z"]),
            }
        )

    # Create our own temporary YAML files that include substitutions and use them to create the parameter file launch configurations
    robot_params_file_args = []
    for num in range(int(robot_number.perform(context))):
        param_substitutions = {
            'initial_pose.x': robots[num]['x_pose'],
            'initial_pose.y': robots[num]['y_pose'],
            'initial_pose.z': robots[num]['z_pose'],
        }
        tmp_params_file = yaml_launch.get_yaml(os.path.join(pkg_dir, 'config', 'nav2_params', params_file_name.perform(context)+".yaml"))
        tmp_params_file = yaml_launch.substitute_values(tmp_params_file, param_substitutions)
        tmp_params_file = yaml_launch.insert_namespace(tmp_params_file, robots[num]['namespace'])
        tmp_params_file = yaml_launch.get_temp_file(tmp_params_file)
        robot_params_file_args.append(tmp_params_file)

    # Nodes and other launch files
    launch_description_content = []

    for num in range(int(robot_number.perform(context))):
        launch_description_content.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav_launch_file_dir, 'navigation_single_robot.launch.py')),
                launch_arguments={
                    'autostart': autostart,
                    'default_bt_xml_filename': default_bt_xml_filename,
                    'map': map_file,
                    'namespace': robots[num]['namespace'],
                    'params_file': robot_params_file_args[num],
                    'rviz_config': rviz_config,
                    'use_namespace': 'true',
                    'use_sim_time': use_sim_time,
                    'world_name': world_name,
                    'enable_slam': enable_slam,
                    'enable_localization': enable_localization,
                    'enable_navigation': enable_navigation,
                    'enable_rviz': enable_rviz,
                    'enable_map_server': enable_map_server,
                }.items(),
            )
        )

    return launch_description_content


def generate_launch_description():
    """Return launch description"""

    # Package Directories
    pkg_dir = get_package_share_directory('auna_nav2')

    # Paths to folders and files
    default_rviz_config_file = os.path.join(pkg_dir, 'rviz', 'config_navigation_namespace.rviz')
    default_bt_xml_filename_file = os.path.join(get_package_share_directory('nav2_bt_navigator'), 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    # Launch Arguments
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the stacks'
    )
    default_bt_xml_filename_arg = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=default_bt_xml_filename_file,
        description='Full path to the behavior tree xml file to use'
    )
    params_file_name_arg = DeclareLaunchArgument(
        'params_file_name',
        default_value='nav2_params',
        description='Name of the parameter file in /config/nav2_params/ without .yaml'
    )
    robot_number_arg = DeclareLaunchArgument(
        'robot_number',
        default_value='2',
        description='Number of spawned robots'
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
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='racetrack_decorated',
        description='Gazebo world file name in /worlds folder'
    )
    enable_slam_arg = DeclareLaunchArgument(
        'enable_slam',
        default_value='False',
        description='Enable SLAM'
    )
    enable_localization_arg = DeclareLaunchArgument(
        'enable_localization',
        default_value='True',
        description='Enable Localization'
    )
    enable_navigation_arg = DeclareLaunchArgument(
        'enable_navigation',
        default_value='True',
        description='Enable Navigation'
    )
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='True',
        description='Enable RViz'
    )
    enable_map_server_arg = DeclareLaunchArgument(
        'enable_map_server',
        default_value='True',
        description='Enable Map Server'
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(autostart_arg)
    launch_description.add_action(default_bt_xml_filename_arg)
    launch_description.add_action(params_file_name_arg)
    launch_description.add_action(robot_number_arg)
    launch_description.add_action(rviz_config_arg)
    launch_description.add_action(use_sim_time_arg)
    launch_description.add_action(world_name_arg)
    launch_description.add_action(enable_slam_arg)
    launch_description.add_action(enable_localization_arg)
    launch_description.add_action(enable_navigation_arg)
    launch_description.add_action(enable_rviz_arg)
    launch_description.add_action(enable_map_server_arg)

    launch_description.add_action(OpaqueFunction(function=include_launch_description))

    return launch_description
