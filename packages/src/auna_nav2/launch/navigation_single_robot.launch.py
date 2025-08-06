import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch_ros.actions import PushRosNamespace
from launch.launch_context import LaunchContext
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml, ReplaceString


def include_launch_description(context: LaunchContext):
    """Return launch description"""

    # Package Directories
    pkg_dir = get_package_share_directory('auna_nav2')
    gazebo_pkg_dir = get_package_share_directory('auna_gazebo')

    # Paths to folders and files
    nav_launch_file_dir = os.path.join(pkg_dir, 'launch')

    # Launch Argument Configurations
    autostart = LaunchConfiguration('autostart')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_slam = LaunchConfiguration('enable_slam')
    enable_localization = LaunchConfiguration('enable_localization')
    enable_navigation = LaunchConfiguration('enable_navigation')
    enable_rviz = LaunchConfiguration('enable_rviz')
    robot_index = LaunchConfiguration('robot_index')

    # Get robot index from context and world name from environment
    robot_idx = int(robot_index.perform(context))
    world_name_str = os.environ.get('WORLD_NAME', 'racetrack_decorated')

    # Create robot namespace based on index
    if robot_idx >= 0:
        robot_namespace = f'robot{robot_idx}'
    else:
        robot_namespace = ''

    # Calculate initial pose based on robot index and world configuration
    map_config_path = os.path.join(gazebo_pkg_dir, "config",
                                   "map_params", f"{world_name_str}.yaml")

    if robot_idx >= 0 and os.path.exists(map_config_path):
        with open(map_config_path, 'r') as f:
            map_config = yaml.safe_load(f)
        x_pose = map_config["spawn"]["offset"]["x"] + \
            robot_idx * map_config["spawn"]["linear"]["x"]
        y_pose = map_config["spawn"]["offset"]["y"] + \
            robot_idx * map_config["spawn"]["linear"]["y"]
        z_pose = map_config["spawn"]["offset"]["z"] + \
            robot_idx * map_config["spawn"]["linear"]["z"]
    else:
        # Default pose if no world config or invalid robot index
        x_pose = 0.0
        y_pose = 0.0
        z_pose = 0.01

    # Create parameter substitutions for initial pose and nav2 parameters
    param_substitutions = {
        'initial_pose.x': str(x_pose),
        'initial_pose.y': str(y_pose),
        'initial_pose.z': str(z_pose),
        'use_sim_time': use_sim_time.perform(context),
        'yaml_filename': map_file.perform(context),
    }

    # Process parameter file with substitutions and namespace
    # First handle namespace replacement if needed
    params_file_path = params_file.perform(context)
    if robot_namespace:
        params_with_namespace = ReplaceString(
            source_file=params_file_path,
            replacements={'<robot_namespace>': robot_namespace})
    else:
        params_with_namespace = params_file_path

    # Apply parameter substitutions using Nav2's RewrittenYaml
    configured_params = RewrittenYaml(
        source_file=params_with_namespace,
        root_key=robot_namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # Debug: print parameter file path
    # Print the actual path or content of the configured params
    if hasattr(configured_params, 'perform'):
        params_path = configured_params.perform(context)
        print(f"Using configured params: {params_path}")
        if os.path.exists(params_path):
            with open(params_path, 'r') as f:
                print(f.read())
        else:
            print(f"File does not exist: {params_path}")
    else:
        print(f"Using configured params: {configured_params}")
        if os.path.exists(configured_params):
            with open(configured_params, 'r') as f:
                print(f.read())
        else:
            print(f"File does not exist: {configured_params}")

    actions = []
    if robot_namespace != "":
        actions.append(PushRosNamespace(robot_namespace))
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                nav_launch_file_dir, 'bringup.launch.py')),
            launch_arguments={
                'autostart': autostart,
                'default_bt_xml_filename': default_bt_xml_filename,
                'map': map_file,
                'params_file': configured_params,
                'use_sim_time': use_sim_time,
                'enable_slam': enable_slam,
                'enable_localization': enable_localization,
                'enable_navigation': enable_navigation,
            }.items(),
        )
    )
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                nav_launch_file_dir, 'rviz.launch.py')),
            condition=IfCondition(enable_rviz),
            launch_arguments={
                'rviz_config': rviz_config,
            }.items(),
        )
    )
    cmd_group = GroupAction(actions)

    # Nodes and other launch files
    cmds = []

    cmds.append(cmd_group)

    return cmds


def generate_launch_description():
    """Return launch description"""

    # # Package Directories
    pkg_dir = get_package_share_directory('auna_nav2')

    # Paths to folders and files
    map_name = os.environ.get('MAP_NAME', 'default')
    default_map_file = os.path.join(
        pkg_dir, 'maps', map_name, 'map.yaml')
    default_params_file = os.path.join(
        pkg_dir, 'config', 'nav2_params', 'nav2_params.yaml')
    default_rviz_config_file = os.path.join(
        pkg_dir, 'rviz', 'config_navigation_namespace.rviz')
    default_default_bt_xml_filename_file = os.path.join(get_package_share_directory(
        'nav2_bt_navigator'), 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

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
    robot_index_arg = DeclareLaunchArgument(
        'robot_index',
        default_value='0',
        description='Robot index number (0, 1, 2, etc.)'
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

    launch_description = LaunchDescription()

    launch_description.add_action(autostart_arg)
    launch_description.add_action(default_bt_xml_filename_arg)
    launch_description.add_action(map_arg)
    launch_description.add_action(robot_index_arg)
    launch_description.add_action(params_file_arg)
    launch_description.add_action(rviz_config_arg)
    launch_description.add_action(use_sim_time_arg)
    launch_description.add_action(use_enable_slam_arg)
    launch_description.add_action(use_enable_localization_arg)
    launch_description.add_action(use_enable_navigation_arg)
    launch_description.add_action(use_enable_rviz_arg)

    launch_description.add_action(OpaqueFunction(
        function=include_launch_description))

    return launch_description
