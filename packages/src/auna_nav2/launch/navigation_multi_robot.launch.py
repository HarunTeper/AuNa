# Copyright 2025 Harun Teper
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


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
    default_map_file = os.path.join(
        pkg_dir, 'maps', context.launch_configurations['world_name'], 'map.yaml'),

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
    namespace = LaunchConfiguration('namespace')

    # Names and poses of the robots
    map_path = os.path.join(gazebo_pkg_dir, "config",
                            "map_params", world_name.perform(context)+".yaml")

    namespace = namespace.perform(context)
    robot_number = int(robot_number.perform(context))
    print(
        f"navigation_multi_robot_launch: Spawning {robot_number} nav nodes with namespace {namespace}")
    if namespace:
        robots = []
        for num in range(robot_number):
            robot_namespace = f'{namespace}{num}'
            robots.append({
                'name': robot_namespace,
                'namespace': robot_namespace,
                'x_pose': yaml_launch.get_yaml_value(map_path, ["spawn", "offset", "x"])+num*yaml_launch.get_yaml_value(map_path, ["spawn", "linear", "x"]),
                'y_pose': yaml_launch.get_yaml_value(map_path, ["spawn", "offset", "y"])+num*yaml_launch.get_yaml_value(map_path, ["spawn", "linear", "y"]),
                'z_pose': yaml_launch.get_yaml_value(map_path, ["spawn", "offset", "z"])+num*yaml_launch.get_yaml_value(map_path, ["spawn", "linear", "z"]),
            })
    else:
        robots = [{
            'name': '',
            'namespace': '',
            'x_pose': yaml_launch.get_yaml_value(map_path, ["spawn", "offset", "x"]),
            'y_pose': yaml_launch.get_yaml_value(map_path, ["spawn", "offset", "y"]),
            'z_pose': yaml_launch.get_yaml_value(map_path, ["spawn", "offset", "z"]),
        }]

    # Create our own temporary YAML files that include substitutions and use them to create the parameter file launch configurations
    robot_params_file_args = []
    for num in range(robot_number):
        param_substitutions = {
            'initial_pose.x': robots[num]['x_pose'],
            'initial_pose.y': robots[num]['y_pose'],
            'initial_pose.z': robots[num]['z_pose'],
        }
        tmp_params_file = yaml_launch.get_yaml(os.path.join(
            pkg_dir, 'config', 'nav2_params', params_file_name.perform(context)+".yaml"))
        tmp_params_file = yaml_launch.substitute_values(
            tmp_params_file, param_substitutions)
        # tmp_params_file = yaml_launch.insert_namespace(
        #     tmp_params_file, robots[num]['namespace'])
        tmp_params_file = yaml_launch.get_temp_file(tmp_params_file)
        robot_params_file_args.append(tmp_params_file)

    # Nodes and other launch files
    launch_description_content = []

    # Launch Map Server Globally (if enabled)
    # The map_server should not be namespaced per robot.
    if enable_map_server.perform(context).lower() == 'true':
        map_server_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                nav_launch_file_dir, 'map_server.launch.py')),
            launch_arguments={
                'namespace': '',  # Launch map_server in global namespace
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'map': map_file,
                # Assuming map_server uses some common params
                'params_file': os.path.join(pkg_dir, 'config', 'nav2_params', params_file_name.perform(context)+".yaml")
            }.items()
        )
        launch_description_content.append(map_server_launch)

    # Launch RViz Globally (if enabled)
    # This RViz instance will be for global view, subscribing to global /map
    if enable_rviz.perform(context).lower() == 'true':
        # Determine the global RViz config file (e.g., config_arena.rviz)
        # This might need adjustment if rviz_config is meant to be namespaced per robot
        global_rviz_config_file = os.path.join(
            pkg_dir, 'rviz', 'config_arena.rviz')  # Example, adjust as needed

        rviz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                nav_launch_file_dir, 'rviz.launch.py')),
            launch_arguments={
                'namespace': '',  # Launch global RViz in global namespace
                'rviz_config': global_rviz_config_file
            }.items()
        )
        launch_description_content.append(rviz_launch)

    for num in range(robot_number):
        print(
            f"navigation_multi_robot_launch: Nav nodes {num} namespace: {robots[num]['namespace']}")
        launch_description_content.extend([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    nav_launch_file_dir, 'navigation_single_robot.launch.py')),
                launch_arguments={
                    'autostart': autostart,
                    'default_bt_xml_filename': default_bt_xml_filename,
                    'map': map_file,  # Single robot might still need map path for its costmaps
                    'namespace': robots[num]['namespace'],
                    'params_file': robot_params_file_args[num],
                    'rviz_config': rviz_config,  # This would be for a per-robot RViz if enabled below
                    'use_namespace': 'true',
                    'use_sim_time': use_sim_time,
                    'world_name': world_name,
                    'enable_slam': enable_slam,
                    'enable_localization': enable_localization,
                    'enable_navigation': enable_navigation,
                    'enable_rviz': 'false',  # Set to 'false' if global RViz is used, or manage per-robot
                    'enable_map_server': 'false',  # Map server is now global
                }.items(),
            )
        ])

    return launch_description_content


def generate_launch_description():
    """Return launch description"""

    # Package Directories
    pkg_dir = get_package_share_directory('auna_nav2')

    # Paths to folders and files
    default_rviz_config_file = os.path.join(
        pkg_dir, 'rviz', 'config_navigation_namespace.rviz')
    default_bt_xml_filename_file = os.path.join(get_package_share_directory(
        'nav2_bt_navigator'), 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    # Launch Arguments
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the stacks'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        description='Namespace prefix for the robots'
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
    launch_description.add_action(namespace_arg)
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

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
