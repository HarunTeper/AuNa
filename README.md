# AuNa: Autonomous Navigation System Simulator

[![License](https://img.shields.io/badge/License-See%20Packages-blue.svg)](packages/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Docker](https://img.shields.io/badge/Docker-Compose-blue)](https://docs.docker.com/compose/)

A comprehensive ROS2-based framework for autonomous vehicle simulation, featuring cooperative driving scenarios, multi-robot coordination, and advanced navigation algorithms. The system integrates Gazebo simulation, MATLAB/Simulink control systems, and OMNeT++ communication modeling to provide a complete autonomous navigation research platform.

![Gazebo Simulation](media/gazeboSimulation.gif)

## 🚀 Key Features

- **Multi-Robot Simulation**: Support for multiple autonomous vehicles in coordinated scenarios
- **CACC Implementation**: Cooperative Adaptive Cruise Control for platooning scenarios
- **Advanced Navigation**: Integration with ROS2 Navigation2 stack for path planning and obstacle avoidance
- **Wall Following**: Robust wall-following algorithms using LIDAR data
- **Physical Hardware Support**: Integration with F1/10 race cars and Logitech G29 steering wheels
- **Communication Protocols**: ETSI ITS-G5 CAM (Cooperative Awareness Messages) implementation
- **Flexible Deployment**: Docker-based containerized deployment with multiple scenario profiles
- **Real-time Visualization**: Gazebo 3D simulation with RViz integration
- **MATLAB/Simulink Integration**: Seamless connection for control system development
- **OMNeT++ Network Simulation**: Vehicle-to-vehicle communication modeling

## 📋 System Requirements

### Hardware Requirements
- **CPU**: Multi-core processor (4+ cores recommended)
- **RAM**: 8GB minimum, 16GB recommended
- **GPU**: NVIDIA GPU with CUDA support (optional, for enhanced graphics)
- **Storage**: 20GB available disk space

### Software Dependencies
- **Operating System**: Ubuntu 20.04 LTS or Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill (recommended) or Galactic Geochelone
- **Docker**: Latest version with docker-compose
- **MATLAB/Simulink**: R2021a or later (optional, for control system integration)
- **OMNeT++**: Version 6.0+ (optional, for network simulation)

## 🛠️ Installation

### Method 1: Docker Installation (Recommended)

1. **Clone the repository**:
   ```bash
   git clone https://github.com/HarunTeper/AuNa.git
   cd AuNa
   ```

2. **Configure environment**:
   ```bash
   # Review and edit environment configuration
   # The .env file contains pre-configured settings
   # Modify as needed for your system
   nano .env
   ```

3. **Build Docker containers**:
   ```bash
   docker-compose build
   ```

### Method 2: Native Installation

1. **Install ROS2 Humble**:
   ```bash
   # Add ROS2 apt repository
   sudo apt update && sudo apt install curl gnupg lsb-release
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   
   # Install ROS2
   sudo apt update
   sudo apt install ros-humble-desktop python3-colcon-common-extensions
   ```

2. **Install dependencies**:
   ```bash
   # Navigation2 and Gazebo
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-gazebo-ros-pkgs
   
   # Additional packages
   sudo apt install ros-humble-xacro ros-humble-rmw-cyclonedx-cpp ros-humble-ackermann-msgs
   ```

3. **Build the workspace**:
   ```bash
   cd AuNa/packages
   colcon build --symlink-install
   source install/setup.bash
   ```

## 🚀 Quick Start

### Running a Basic Simulation

1. **Start the simulation environment**:
   ```bash
   # Using Docker (recommended)
   docker-compose --profile sim_scenario_1 up
   
   # Or using native installation
   ros2 launch auna_gazebo gazebo_world.launch.py
   ```

2. **Launch a single robot scenario**:
   ```bash
   # Start robot 1 with navigation stack
   docker-compose --profile sim_robot1 up
   ```

3. **Open visualization**:
   ```bash
   # Launch RViz for monitoring
   rviz2 -d packages/src/auna_nav2/rviz/config_navigation_namespace.rviz
   ```

### Running Platooning Scenario

The platooning scenario demonstrates cooperative adaptive cruise control (CACC) with multiple robots:

```bash
# Start full platooning scenario with 3 robots
docker-compose --profile sim_scenario up

# View the simulation
# Open your browser and navigate to Gazebo GUI or use RViz
```

![OMNeT++ Simulation](media/omnetSimulation.gif)

## 📁 Package Overview

### Core Simulation Packages

| Package | Description |
|---------|-------------|
| `auna_gazebo` | Gazebo integration, world files, and robot models |
| `auna_ground_truth` | Ground truth pose estimation for simulation |
| `auna_tf` | Transform management and coordinate frames |
| `auna_msgs` | Custom message definitions |

### Navigation & Control Packages

| Package | Description |
|---------|-------------|
| `auna_nav2` | Navigation2 stack integration and configuration |
| `auna_cacc` | Cooperative Adaptive Cruise Control implementation |
| `auna_control` | Control panel and command multiplexing |
| `auna_wallfollowing` | Wall-following algorithms using LIDAR |
| `auna_waypoints` | Waypoint management and trajectory planning |

### Communication & Networking

| Package | Description |
|---------|-------------|
| `auna_comm` | V2V communication protocols (CAM messages) |
| `auna_omnet` | OMNeT++ integration for network simulation |

### Hardware Integration

| Package | Description |
|---------|-------------|
| `auna_f110` | F1/10 race car platform support |
| `physical/ros-g29-force-feedback` | Logitech G29 steering wheel integration |

### Utilities

| Package | Description |
|---------|-------------|
| `auna_teleoperation` | Remote control and manual driving |
| `auna_ekf` | Extended Kalman Filter for localization |
| `auna_common` | Shared utilities and common functions |

## 🎯 Usage Examples

### Example 1: Single Robot Navigation

```bash
# 1. Start Gazebo simulation
docker-compose up gazebo

# 2. Spawn robot
docker-compose up spawn_robot_robot1

# 3. Start navigation stack
docker-compose up nav2_stack_robot1

# 4. Set navigation goal through RViz or programmatically
ros2 topic pub /robot1/goal_pose geometry_msgs/PoseStamped "..."
```

### Example 2: Wall Following

```bash
# Start wall following scenario
docker-compose up gazebo spawn_robot_robot1 wallfollowing_robot1

# The robot will automatically start following the nearest wall
```

### Example 3: Multi-Robot CACC Platooning

```bash
# Start complete platooning scenario
docker-compose --profile sim_scenario up

# Robots will automatically form a platoon with CACC control
```

### Example 4: Manual Teleoperation

```bash
# Start robot and teleoperation
docker-compose up spawn_robot_robot1 teleop

# Use keyboard controls:
# W/S: Forward/Backward
# A/D: Left/Right steering
# Space: Emergency stop
```

## 🏗️ System Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Gazebo        │    │   Navigation2   │    │   MATLAB/       │
│   Simulation    │◄──►│   Stack         │◄──►│   Simulink      │
└─────────────────┘    └─────────────────┘    └─────────────────┘
          │                        │                        │
          ▼                        ▼                        ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Robot         │    │   CACC          │    │   Communication │
│   Controllers   │◄──►│   Controller    │◄──►│   (V2V/V2I)     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
          │                        │                        │
          ▼                        ▼                        ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Sensors       │    │   Localization  │    │   OMNeT++       │
│   (LIDAR/Camera)│◄──►│   (EKF/AMCL)    │◄──►│   Network Sim   │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## 🔧 Configuration

### Environment Variables

Key environment variables in `.env`:

```bash
# ROS2 Configuration
ROS_DISTRO=humble                    # ROS2 distribution
RMW_IMPLEMENTATION=rmw_zenoh_cpp     # ROS2 middleware

# Simulation Parameters
WORLD_NAME=racetrack_decorated       # Gazebo world to load
COMMUNICATION_TYPE=cam               # Communication protocol (cam/omnet)
USE_WAYPOINTS=true                   # Enable waypoint navigation

# Docker Configuration
HOST_UID=1000                        # Host user ID for Docker
HOST_GID=1000                        # Host group ID for Docker
```

### Robot Configuration

Modify robot parameters in `packages/src/auna_gazebo/config/`:

- `model_params/`: Robot physical parameters
- `map_params/`: Map-specific configuration

### Navigation Configuration

Tune navigation parameters in `packages/src/auna_nav2/config/nav2_params/`:

- `nav2_params.yaml`: Core navigation parameters
- `planner_params.yaml`: Path planning configuration
- `controller_params.yaml`: Path following parameters

## 🧪 Development

### Creating Custom Scenarios

1. **Create a new world file**:
   ```bash
   cp packages/src/auna_gazebo/worlds/racetrack_decorated.world packages/src/auna_gazebo/worlds/my_world.world
   # Edit my_world.world in Gazebo or text editor
   ```

2. **Add to docker-compose**:
   ```yaml
   my_scenario:
     extends: gazebo
     environment:
       - WORLD_NAME=my_world
   ```

### Adding New Robot Controllers

1. **Create package**:
   ```bash
   cd packages/src
   ros2 pkg create --build-type ament_cmake my_controller
   ```

2. **Implement controller node**:
   ```cpp
   #include "rclcpp/rclcpp.hpp"
   #include "geometry_msgs/msg/twist.hpp"
   
   class MyController : public rclcpp::Node {
       // Implementation
   };
   ```

3. **Add to CMakeLists.txt and package.xml**

### Testing

Run tests for specific packages:

```bash
# Build with tests
colcon build --packages-select auna_cacc --cmake-args -DBUILD_TESTING=ON

# Run tests
colcon test --packages-select auna_cacc
colcon test-result --verbose
```

## 🐛 Troubleshooting

### Common Issues

1. **Docker containers fail to start**:
   ```bash
   # Check Docker daemon
   sudo systemctl status docker
   
   # Rebuild containers
   docker-compose build --no-cache
   ```

2. **Gazebo crashes or has poor performance**:
   ```bash
   # Check GPU drivers
   nvidia-smi  # For NVIDIA GPUs
   
   # Reduce graphics quality in Gazebo settings
   # Or set LIBGL_ALWAYS_SOFTWARE=1
   ```

3. **ROS2 nodes cannot communicate**:
   ```bash
   # Check ROS_DOMAIN_ID
   echo $ROS_DOMAIN_ID
   
   # Verify network configuration
   ros2 node list
   ros2 topic list
   ```

4. **Navigation fails**:
   ```bash
   # Check map and localization
   ros2 topic echo /map
   ros2 topic echo /amcl_pose
   
   # Verify robot model and sensors
   ros2 run robot_state_publisher robot_state_publisher
   ```

### Performance Optimization

- **Reduce simulation load**: Decrease number of robots or sensors
- **Optimize Docker**: Increase memory limits in docker-compose.yml
- **GPU acceleration**: Ensure proper GPU drivers and Docker GPU support

## 📚 Advanced Features

### MATLAB/Simulink Integration

1. **Setup MATLAB ROS2 interface**:
   ```matlab
   % In MATLAB command window
   pyenv('Version','/usr/bin/python3.9');
   ```

2. **Use Simulink models**: Located in `packages/src/auna_cacc/CACC/`

### OMNeT++ Network Simulation

1. **Build OMNeT++ container**:
   ```bash
   docker-compose build omnet
   ```

2. **Run network simulation**:
   ```bash
   docker-compose --profile omnet up
   ```

### Physical Robot Integration

For F1/10 race cars:

```bash
# Connect to physical robot
docker-compose --profile physical_robot1 up

# Ensure proper network configuration for robot communication
```

## 🤝 Contributing

1. **Fork the repository**
2. **Create a feature branch**:
   ```bash
   git checkout -b feature/my-new-feature
   ```
3. **Make changes and commit**:
   ```bash
   git commit -am 'Add some feature'
   ```
4. **Push to the branch**:
   ```bash
   git push origin feature/my-new-feature
   ```
5. **Create a Pull Request**

### Coding Standards

- Follow ROS2 coding standards
- Use clang-format for C++ code formatting
- Include unit tests for new features
- Update documentation for API changes

## 📄 License

This project's licensing is under development. Please refer to individual package licenses for specific components and check with the maintainers for usage permissions.

## 🙏 Acknowledgments

- **ROS2 Community**: For the robust robotics framework
- **Navigation2 Team**: For the navigation stack
- **Gazebo Team**: For the simulation environment
- **ETSI**: For ITS communication standards
- **Contributors**: All researchers and developers who contributed to this project

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/HarunTeper/AuNa/issues)
- **Discussions**: [GitHub Discussions](https://github.com/HarunTeper/AuNa/discussions)
- **Email**: harun.teper@tu-dortmund.de

---

**Happy Autonomous Navigation!** 🚗🤖
