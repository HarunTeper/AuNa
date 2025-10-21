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


"""Single robot spawn launch file."""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import SetRemap, PushRosNamespace, Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import GroupAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_context import LaunchContext


def include_launch_description(context: LaunchContext):
    """Return launch description."""
    # Package Directories
    pkg_dir = get_package_share_directory('auna_gazebo')
    spawn_launch_file_dir = os.path.join(pkg_dir, 'launch', 'spawn')

    # Launch Configurations
    robot_index = int(os.environ.get('ROBOT_INDEX', '0'))

    # Parameters
    name = f'robot{robot_index}'
    namespace = f'robot{robot_index}'
    urdf_namespace = f'robot{robot_index}'
    use_sim_time = LaunchConfiguration('use_sim_time')

    world_name = os.environ.get('WORLD_NAME', 'racetrack_decorated')

    # auna_common paths
    auna_common_path = "/home/ubuntu/workspace/auna_common"
    world_params_file = os.path.join(
        auna_common_path,
        'config',
        'world_params',
        f'{world_name}.yaml'
    )

    # Load spawn parameters from YAML
    with open(world_params_file, 'r') as f:
        world_params = yaml.safe_load(f)

    spawn_config = world_params['spawn']
    num = int(robot_index)

    x_pose_value = spawn_config['offset']['x'] + num * spawn_config['linear']['x']
    y_pose_value = spawn_config['offset']['y'] + num * spawn_config['linear']['y']
    z_pose_value = spawn_config['offset']['z'] + num * spawn_config['linear']['z']

    x_pose = str(x_pose_value)
    y_pose = str(y_pose_value)
    z_pose = str(z_pose_value)

    # Individual components
    tf_remap = SetRemap(src='/tf', dst='tf')
    tf_static_remap = SetRemap(src='/tf_static', dst='tf_static')

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(spawn_launch_file_dir,
                         '_robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'urdf_namespace': urdf_namespace,
        }.items()
    )

    # Spawn robot using new Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', name,
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-allow_renaming', 'true',
        ],
        output='screen'
    )

    # Bridge for standard topics
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        arguments=[
            # Ackermann CMD (ROS -> Gz)
            f'/model/{urdf_namespace}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',

            # Odom (Gz -> ROS)
            f'/model/{urdf_namespace}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            # TF (Gz -> ROS)
            f'/model/{urdf_namespace}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

            # IMU (Gz -> ROS)
            f'/model/{urdf_namespace}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',

            # LIDAR (Gz -> ROS)
            f'/model/{urdf_namespace}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            # GPS/NavSat (Gz -> ROS)
            f'/model/{urdf_namespace}/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',

            # Joint States (Gz -> ROS)
            f'/model/{urdf_namespace}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            (f'/model/{urdf_namespace}/cmd_vel', 'cmd_vel_twist'),
            (f'/model/{urdf_namespace}/odom', 'odom'),
            (f'/model/{urdf_namespace}/tf', 'tf'),
            (f'/model/{urdf_namespace}/imu', 'imu'),
            (f'/model/{urdf_namespace}/scan', 'scan'),
            (f'/model/{urdf_namespace}/navsat', 'gps/fix'),
            (f'/model/{urdf_namespace}/joint_states', 'joint_states'),
        ],
        output='screen'
    )

    # Specialized bridge for Camera topics
    cam_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='image_bridge',
        arguments=[
            f'/model/{urdf_namespace}/camera',
        ],
        remappings=[
            (f'/model/{urdf_namespace}/camera', 'camera/image_raw'),
            (f'/model/{urdf_namespace}/camera_info', 'camera/camera_info'),
        ],
        output='screen'
    )

    # Main robot launch group with namespace and remappings
    robot_launch_group = GroupAction([
        PushRosNamespace(namespace),
        tf_remap,
        tf_static_remap,
        robot_state_publisher,
        spawn_robot,
        gz_bridge,
        cam_bridge,
    ])

    launch_actions = []
    launch_actions.append(robot_launch_group)
    return launch_actions


def generate_launch_description():

    # Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    return LaunchDescription([
        use_sim_time_arg,
        OpaqueFunction(function=include_launch_description)
    ])
