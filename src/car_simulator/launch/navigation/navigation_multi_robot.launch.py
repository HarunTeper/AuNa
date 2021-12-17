#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os
from ament_index_python.packages import get_package_share_directory
from car_simulator import yaml_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def include_navigation_description(context: LaunchContext):

    # Package Directories
    pkg_dir = get_package_share_directory('car_simulator')

    # Paths to folders and files
    nav_launch_file_dir = os.path.join(pkg_dir, 'launch', 'navigation')
    default_map_file = os.path.join(pkg_dir, 'maps', context.launch_configurations['world_name'], 'map.yaml'),
    default_rviz_config_file = os.path.join(pkg_dir, 'rviz','config_navigation_namespace.rviz')
    default_bt_xml_filename_file = os.path.join(get_package_share_directory('nav2_bt_navigator'), 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    # Launch Argument Configurations
    autostart = LaunchConfiguration('autostart', default='true')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename', default=default_bt_xml_filename_file)
    map = LaunchConfiguration('map', default=default_map_file)
    params_file_name = LaunchConfiguration('params_file_name', default='nav2_params_namespace')
    robot_number = LaunchConfiguration('robot_number', default='2')
    rviz_config = LaunchConfiguration('rviz_config', default = default_rviz_config_file)
    slam = LaunchConfiguration('slam', default = 'false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='racetrack')

    # Names and poses of the robots
    robots = []
    for num in range(int(robot_number.perform(context))):
        robots.append({
            'name': 'robot'+str(num), 
            'namespace': 'robot'+str(num), 
            'x_pose': yaml_launch.get_yaml_value("map_params",world_name.perform(context), ["spawn","offset","x"])+num*yaml_launch.get_yaml_value("map_params",world_name.perform(context), ["spawn","linear","x"]),
            'y_pose': yaml_launch.get_yaml_value("map_params",world_name.perform(context), ["spawn","offset","y"])+num*yaml_launch.get_yaml_value("map_params",world_name.perform(context), ["spawn","linear","y"]),
            'z_pose': yaml_launch.get_yaml_value("map_params",world_name.perform(context), ["spawn","offset","z"])+num*yaml_launch.get_yaml_value("map_params",world_name.perform(context), ["spawn","linear","z"]),
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
        
        tmp_params_file = yaml_launch.get_yaml('nav2_params',params_file_name.perform(context))
        tmp_params_file = yaml_launch.substitute_values(tmp_params_file, param_substitutions)
        tmp_params_file = yaml_launch.insert_namespace(tmp_params_file,robots[num]['namespace'])
        tmp_params_file = yaml_launch.get_temp_file(tmp_params_file)
        robot_params_file_args.append(tmp_params_file)

    # Nodes and other launch files
    cmds = []

    for num in range(int(robot_number.perform(context))):
        cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav_launch_file_dir, 'navigation_single_robot.launch.py')),
                launch_arguments={
                    'autostart': autostart,
                    'default_bt_xml_filename': default_bt_xml_filename,
                    'map': map,
                    'namespace': robots[num]['namespace'],
                    'params_file': robot_params_file_args[num],
                    'rviz_config': rviz_config,
                    'slam': slam,
                    'use_namespace': 'true',
                    'use_sim_time': use_sim_time,
                    'world_name': world_name,
                }.items(),
            )
        )

    return cmds

def generate_launch_description():

    # Package Directories
    pkg_dir = get_package_share_directory('car_simulator')

    # Paths to folders and files
    default_rviz_config_file = os.path.join(pkg_dir, 'rviz','config_navigation_namespace.rviz')
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
        default_value='nav2_params_namespace',
        description='Name of the parameter file in /config/nav2_params/'
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
    slam_arg = DeclareLaunchArgument(
        name='slam',
        default_value='False',
        description='Launch SLAM node to generate a map during navigation'
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
    
    # Launch Description
    ld = LaunchDescription()

    ld.add_action(autostart_arg)
    ld.add_action(default_bt_xml_filename_arg)
    ld.add_action(params_file_name_arg)
    ld.add_action(robot_number_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(slam_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(world_name_arg)

    ld.add_action(OpaqueFunction(function=include_navigation_description))

    return ld