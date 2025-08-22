
"""Ground truth launch file"""

from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch_ros.actions import PushRosNamespace, SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def include_ground_truth_launches(context: LaunchContext):
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_dir = get_package_share_directory('auna_ground_truth')
    launch_dir = os.path.join(pkg_dir, 'launch')
    
    robot_index = int(os.environ.get('ROBOT_INDEX', '0'))

    ground_truth_transform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'ground_truth_transform.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    ground_truth_pose_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'ground_truth_pose_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    ground_truth_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'ground_truth_cam.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    # Individual components
    tf_remap = SetRemap(src='/tf', dst='tf')
    tf_static_remap = SetRemap(src='/tf_static', dst='tf_static')

    group_cmd = GroupAction([
        PushRosNamespace(f'robot{robot_index}'),
        tf_remap,
        tf_static_remap,
        ground_truth_transform,
        ground_truth_pose_publisher,
        ground_truth_cam
    ])

    return [group_cmd]

def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        OpaqueFunction(function=include_ground_truth_launches)
    ])
