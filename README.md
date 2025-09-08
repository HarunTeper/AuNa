# AuNa: Autonomous Navigation System Simulator

[![License](https://img.shields.io/badge/License-See%20Packages-blue.svg)](packages/)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Docker](https://img.shields.io/badge/docker compose-blue)](https://docs.docker.com/compose/)

A comprehensive ROS2-based framework for autonomous vehicle simulation, featuring cooperative driving scenarios, multi-robot coordination, and advanced navigation algorithms. The system integrates Gazebo simulation, MATLAB/Simulink control systems, and OMNeT++ communication modeling to provide a complete autonomous navigation research platform.

![Gazebo Simulation](media/gazeboSimulation.gif)

## 🚀 Key Features

- **Multi-Robot Simulation**: Support for multiple autonomous vehicles in coordinated scenarios
- **CACC Implementation**: Cooperative Adaptive Cruise Control for platooning scenarios
- **Advanced Navigation**: Integration with ROS2 Navigation2 stack for path planning and obstacle avoidance
- **Wall Following**: Robust wall-following algorithms using LIDAR data
<!-- - **Physical Hardware Support**: Integration with F1/10 race cars and Logitech G29 steering wheels -->
- **Communication Protocols**: ETSI ITS-G5 CAM (Cooperative Awareness Messages) implementation
- **Flexible Deployment**: Docker-based containerized deployment with multiple scenario profiles
- **Real-time Visualization**: Gazebo 3D simulation with RViz integration
<!-- - **MATLAB/Simulink Integration**: Seamless connection for control system development -->
<!-- - **OMNeT++ Network Simulation**: Vehicle-to-vehicle communication modeling -->

## 📋 System Requirements

### Hardware Requirements
- **CPU**: Multi-core processor (4+ cores recommended)
- **RAM**: 8GB minimum, 16GB recommended
- **GPU**: NVIDIA GPU with CUDA support (optional, for enhanced graphics)
- **Storage**: 20GB available disk space

### Software Dependencies
- **Operating System**: Ubuntu 20.04 LTS or newer
<!-- - **ROS2**: Humble Hawksbill (recommended) or Galactic Geochelone -->
- **Docker**: Latest version with docker compose
<!-- - **MATLAB/Simulink**: R2021a or later (optional, for control system integration)
- **OMNeT++**: Version 6.0+ (optional, for network simulation) -->

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

## 🚀 Quick Start

### Running a Basic Simulation With One Robot

1. **Start the simulation environment**:
   ```bash
   # Using Docker (recommended)
   docker compose --profile sim_scenario_1 up

### Running Platooning Scenario

The platooning scenario demonstrates cooperative adaptive cruise control (CACC) with multiple robots:

```bash
# Start full platooning scenario with 3 robots
docker compose --profile sim_scenario up

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

## 🔧 Configuration

### Environment Variables

Key environment variables in `.env`:

```bash
# ROS2 Configuration
ROS_DISTRO=humble                    # ROS2 distribution
RMW_IMPLEMENTATION=rmw_zenoh_cpp     # ROS2 middleware

# Simulation Parameters
WORLD_NAME=racetrack_decorated       # Gazebo world to load (racetrack_decorated/arena)
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

2. **Add to .env**:
   ```yaml
   WORLD_NAME=my_world
   ```

## 🐛 Troubleshooting

### Common Issues

1. **Docker containers fail to start**:
   ```bash
   # Check Docker daemon
   sudo systemctl status docker
   
   # Rebuild containers
   docker compose build --no-cache
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
   ```

### Performance Optimization

- **Reduce simulation load**: Decrease number of robots or sensors
- **Optimize Docker**: Increase memory limits in docker compose.yml
- **GPU acceleration**: Ensure proper GPU drivers and Docker GPU support

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
