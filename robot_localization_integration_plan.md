# Robot Localization (EKF) Integration Plan for AUNA Gazebo

## 1. Overall Goal

Integrate `robot_localization` (using an EKF - Extended Kalman Filter) to fuse IMU, wheel odometry, and GPS data for each robot, operating within its own namespace. The initial setup will provide a launch file and a configuration file that fuses all three sensors. The ability to disable specific sensors will initially be managed by modifying the EKF configuration file (YAML) or by preparing alternative YAML files for different sensor combinations.

## 2. Proposed File Structure and Changes

### 2.1. EKF Configuration File

*   **Path:** `packages/src/auna_gazebo/config/ekf/ekf.yaml`
*   **Content:** This file will contain the parameters for the `ekf_node` from `robot_localization`.
    *   Uses `odom` as `odom_frame` and `world_frame`, and `base_link` as `base_link_frame`.
    *   Subscribes to `imu`, `odom`, and `gps/fix` (relative to the node's namespace).
    *   Configures `*_config` matrices for sensor data selection.
    *   Sets `publish_tf` to true.
    *   Includes example process noise and initial estimate covariances.
    *   Sets `use_sim_time` to true.

```yaml
# packages/src/auna_gazebo/config/ekf/ekf.yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: false
    sensor_timeout: 0.1

    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    imu0: imu
    imu0_config: [false, false, false, # X, Y, Z
                  true,  true,  true,  # Roll, Pitch, Yaw
                  false, false, false, # dX, dY, dZ
                  true,  true,  true,  # dRoll, dPitch, dYaw
                  false, false, false] # ddX, ddY, ddZ
    imu0_differential: false
    imu0_relative: false
    imu0_queue_size: 5
    imu0_remove_gravitational_acceleration: true

    odom0: odom
    odom0_config: [true,  true,  false, # X, Y, Z
                   false, false, false, # Roll, Pitch, Yaw
                   true,  true,  false, # dX, dY, dZ
                   false, false, true,  # dRoll, dPitch, dYaw
                   false, false, false] # ddX, ddY, ddZ
    odom0_differential: false
    odom0_relative: false
    odom0_queue_size: 10

    pose0: gps/fix
    pose0_config: [true,  true,  false, # X, Y, Z
                   false, false, false, # Roll, Pitch, Yaw
                   false, false, false, # dX, dY, dZ
                   false, false, false, # dRoll, dPitch, dYaw
                   false, false, false] # ddX, ddY, ddZ
    pose0_differential: false
    pose0_relative: false
    pose0_queue_size: 5

    publish_tf: true

    process_noise_covariance: [
      1.0, 0,   0,   0,   0,   0,   0,     0,     0,     0,     0,     0,     0,   0,   0,
      0,   1.0, 0,   0,   0,   0,   0,     0,     0,     0,     0,     0,     0,   0,   0,
      0,   0,   0.03,0,   0,   0,   0,     0,     0,     0,     0,     0,     0,   0,   0,
      0,   0,   0,   0.03,0,   0,   0,     0,     0,     0,     0,     0,     0,   0,   0,
      0,   0,   0,   0,   0.03,0,   0,     0,     0,     0,     0,     0,     0,   0,   0,
      0,   0,   0,   0,   0,   0.06,0,     0,     0,     0,     0,     0,     0,   0,   0,
      0,   0,   0,   0,   0,   0,   0.025, 0,     0,     0,     0,     0,     0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,     0.025, 0,     0,     0,     0,     0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,     0,     0.01,  0,     0,     0,     0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,     0,     0,     0.01,  0,     0,     0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,     0,     0,     0,     0.01,  0,     0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,     0,     0,     0,     0,     0.02,  0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,     0,     0,     0,     0,     0,     0.01,0,   0,
      0,   0,   0,   0,   0,   0,   0,     0,     0,     0,     0,     0,     0,   0.01,0,
      0,   0,   0,   0,   0,   0,   0,     0,     0,     0,     0,     0,     0,   0,   0.015
    ]

    initial_estimate_covariance: [
      1.0e-9, 0,      0,      0,      0,      0,      0,        0,        0,        0,        0,        0,        0,      0,      0,
      0,      1.0e-9, 0,      0,      0,      0,      0,        0,        0,        0,        0,        0,        0,      0,      0,
      0,      0,      1.0e-9, 0,      0,      0,      0,        0,        0,        0,        0,        0,        0,      0,      0,
      0,      0,      0,      1.0e-9, 0,      0,      0,        0,        0,        0,        0,        0,        0,      0,      0,
      0,      0,      0,      0,      1.0e-9, 0,      0,        0,        0,        0,        0,        0,        0,      0,      0,
      0,      0,      0,      0,      0,      1.0e-9, 0,        0,        0,        0,        0,        0,        0,      0,      0,
      0,      0,      0,      0,      0,      0,      1.0,      0,        0,        0,        0,        0,        0,      0,      0,
      0,      0,      0,      0,      0,      0,      0,        1.0,      0,        0,        0,        0,        0,      0,      0,
      0,      0,      0,      0,      0,      0,      0,        0,        1.0,      0,        0,        0,        0,      0,      0,
      0,      0,      0,      0,      0,      0,      0,        0,        0,        1.0,      0,        0,        0,      0,      0,
      0,      0,      0,      0,      0,      0,      0,        0,        0,        0,        1.0,      0,        0,      0,      0,
      0,      0,      0,      0,      0,      0,      0,        0,        0,        0,        0,        1.0,      0,      0,      0,
      0,      0,      0,      0,      0,      0,      0,        0,        0,        0,        0,        0,        1.0e-9, 0,      0,
      0,      0,      0,      0,      0,      0,      0,        0,        0,        0,        0,        0,        0,      1.0e-9, 0,
      0,      0,      0,      0,      0,      0,      0,        0,        0,        0,        0,        0,        0,      0,      1.0e-9
    ]
```

### 2.2. EKF Launch File

*   **Path:** `packages/src/auna_gazebo/launch/ekf/localization_ekf.launch.py`
*   **Content:** This Python launch file will:
    *   Accept `namespace`, `use_sim_time`, and `ekf_config_file` arguments.
    *   Include placeholder arguments for `use_gps`, `use_imu`, `use_odom`.
    *   Launch `ekf_node` within the specified namespace.
    *   Remap `odometry/filtered` to `odometry/filtered_ekf`.

```python
# packages/src/auna_gazebo/launch/ekf/localization_ekf.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    auna_gazebo_pkg_share = get_package_share_directory('auna_gazebo')
    default_ekf_config_path = os.path.join(auna_gazebo_pkg_share, 'config', 'ekf', 'ekf.yaml')

    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the robot'
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_ekf_config_file_arg = DeclareLaunchArgument(
        'ekf_config_file',
        default_value=default_ekf_config_path,
        description='Full path to EKF configuration file'
    )
    # Placeholder arguments for future dynamic sensor enabling/disabling logic
    declare_use_gps_arg = DeclareLaunchArgument(
        'use_gps', default_value='true', description='Placeholder: Enable GPS input (managed by EKF YAML)'
    )
    declare_use_imu_arg = DeclareLaunchArgument(
        'use_imu', default_value='true', description='Placeholder: Enable IMU input (managed by EKF YAML)'
    )
    declare_use_odom_arg = DeclareLaunchArgument(
        'use_odom', default_value='true', description='Placeholder: Enable wheel odometry input (managed by EKF YAML)'
    )

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    ekf_config_file = LaunchConfiguration('ekf_config_file')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('odometry/filtered', 'odometry/filtered_ekf')
        ]
    )
    
    namespaced_ekf_group = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            ekf_node
        ]
    )

    return LaunchDescription([
        declare_namespace_arg,
        declare_use_sim_time_arg,
        declare_ekf_config_file_arg,
        declare_use_gps_arg,
        declare_use_imu_arg,
        declare_use_odom_arg,
        namespaced_ekf_group
    ])
```

### 2.3. Update `package.xml` for `auna_gazebo`

*   **Path:** `packages/src/auna_gazebo/package.xml`
*   **Change:** Add an execution dependency on `robot_localization`.

```xml
<exec_depend>robot_localization</exec_depend>
```

## 3. Diagram of New/Modified Files

```mermaid
graph TD
    PARENT[packages/src/auna_gazebo] --> CONFIG_DIR[config/ekf]
    PARENT --> LAUNCH_DIR[launch/ekf]
    CONFIG_DIR --> F1[ekf.yaml]
    LAUNCH_DIR --> F2[localization_ekf.launch.py]
    PARENT --> F3[package.xml (modified)]
```

## 4. System Architecture Overview

```mermaid
graph TD
    subgraph Robot Namespace (e.g., /robot0)
        GPS_TOPIC[<namespace>/gps/fix] --> EKF_NODE
        IMU_TOPIC[<namespace>/imu] --> EKF_NODE
        ODOM_TOPIC[<namespace>/odom] --> EKF_NODE
        EKF_NODE[robot_localization EKF Node]
        EKF_NODE -- Publishes TF --> TF_TREE(<namespace>/odom --> <namespace>/base_link)
        EKF_NODE -- Publishes Odometry --> FUSED_ODOM_TOPIC(<namespace>/odometry/filtered_ekf)
    end
    LAUNCH_FILE[localization_ekf.launch.py] -- launches & namespaces --> EKF_NODE
    YAML_CONFIG[ekf.yaml] -- configures --> EKF_NODE
    GAZEBO_SIM[Gazebo Simulation]
    URDF_MODEL[model.urdf] -- defines frames & sensor plugins --> GAZEBO_SIM
    GAZEBO_SIM -- publishes raw sensor data --> GPS_TOPIC
    GAZEBO_SIM -- publishes raw sensor data --> IMU_TOPIC
    GAZEBO_SIM -- publishes raw sensor data --> ODOM_TOPIC
```

## 5. How to Disable Sensors (Initial Approach)

The requirement to disable one or more sensors can be handled as follows:

1.  **Manually Edit `ekf.yaml`**: Before launching, comment out or remove the configuration block for the sensor(s) you wish to disable (e.g., the entire `pose0`, `pose0_config`, etc. section for GPS).
2.  **Create Multiple YAML Files**: Create different YAML files for various sensor combinations (e.g., `ekf_all_sensors.yaml`, `ekf_no_gps.yaml`, `ekf_imu_odom_only.yaml`). Specify the desired configuration file at launch time using the `ekf_config_file` argument.
    *   *Example:* `ros2 launch auna_gazebo localization_ekf.launch.py namespace:=robot0 ekf_config_file:=$(find-pkg-share auna_gazebo)/config/ekf/ekf_no_gps.yaml`

The placeholder `use_gps`, `use_imu`, `use_odom` arguments in the launch file are for potential future enhancements to allow more dynamic sensor selection directly via launch arguments.