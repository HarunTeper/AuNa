# GitHub Copilot Instructions for AuNa Project

## Project Overview

AuNa is a comprehensive ROS2-based framework for autonomous vehicle simulation featuring:

- Multi-robot coordination and platooning scenarios
- Cooperative Adaptive Cruise Control (CACC)
- Advanced navigation with Navigation2 stack
- Wall-following algorithms using LIDAR
- Docker-based deployment with Gazebo simulation

## Key Technologies & Frameworks

- **ROS2 Humble**: Primary robotics framework
- **Gazebo**: 3D simulation environment
- **Navigation2**: Path planning and navigation
- **Docker Compose**: Containerized deployment
- **C++17**: Primary programming language for ROS2 nodes
- **Python 3**: Used for some utilities and launch files
- **CMake**: Build system for ROS2 packages

## Project Structure

```
AuNa/
├── packages/src/              # ROS2 packages source code
│   ├── auna_cacc/            # Cooperative Adaptive Cruise Control
│   ├── auna_comm/            # V2X communication protocols
│   ├── auna_common/          # Common utilities and libraries
│   ├── auna_control/         # Input multiplexing and control
│   ├── auna_ekf/             # Extended Kalman Filter for localization
│   ├── auna_f110/            # F1/10 platform-specific implementations
│   ├── auna_gazebo/          # Gazebo simulation environment
│   ├── auna_ground_truth/    # Ground truth localization
│   ├── auna_its_msgs/        # ITS (Intelligent Transportation Systems) messages
│   ├── auna_msgs/            # Custom ROS2 message definitions
│   ├── auna_nav2/            # Navigation2 integration
│   ├── auna_omnet/           # OMNeT++ network simulation integration
│   ├── auna_teleoperation/   # Manual control and teleoperation
│   ├── auna_template/        # Template package for new development
│   ├── auna_tf/              # Transform frame management
│   ├── auna_wallfollowing/   # Wall-following algorithms using LIDAR
│   ├── auna_waypoints/       # Waypoint management and processing
│   └── physical/             # Physical hardware support (currently empty)
├── dockerfiles/              # Docker configuration
├── tf2_trees/                # TF2 transform tree documentation
├── traces/                   # Simulation and execution traces
└── .github/                  # GitHub workflows and documentation
```

## Code Style Guidelines

### ROS2 C++ Conventions

- Follow ROS2 C++ style guide
- Use `rclcpp` for ROS2 C++ development
- Node class names should end with "Node" (e.g., `CaccControllerNode`)
- Use smart pointers (`std::shared_ptr`, `std::unique_ptr`)
- Member variables use trailing underscore (e.g., `node_`)
- Constants use UPPER_SNAKE_CASE

### Python Conventions

- Follow PEP 8 style guide
- Use `rclpy` for ROS2 Python development
- Node class names should end with "Node"
- Use type hints where appropriate

### Package Structure

Each ROS2 package should follow standard layout (leave out folders if not applicable):

```
package_name/
├── CMakeLists.txt
├── package.xml
├── config/              # Parameter files (.yaml)
├── launch/             # Launch files (.launch.py)
├── src/                # Source code
├── include/            # Header files (C++)
└── rviz/              # RViz configuration files
```

## Development Patterns

### ROS2 Node Development

When creating new ROS2 nodes:

1. Inherit from `rclcpp::Node` for C++ or `rclpy.Node` for Python
2. Initialize publishers/subscribers in constructor
3. Use timers for periodic operations
4. Implement proper parameter handling
5. Add comprehensive logging with appropriate log levels

### Message and Service Usage

- Use existing `auna_msgs` when possible
- For geometric data, prefer `geometry_msgs`
- For sensor data, use appropriate `sensor_msgs`
- Custom services should be defined in `auna_msgs/srv/`

### Launch File Patterns

- Use `.launch.py` format (Python launch files)
- Support parameterization via launch arguments
- Include namespace support for multi-robot scenarios using PushRosNamespace
- Get robot index from environment variable instead of launch arguments
- Use composition when possible for performance

## Multi-Robot Considerations

The system supports multiple robots with namespace-based separation:

- Robot namespaces: `robot1`, `robot2`, `robot3`, etc.
- Topics should be namespace-aware
- Parameters should support robot-specific configuration
- Launch files should get the namespace using the `robot index` environment variable
- Launch files should use PushRosNamespace for node namespace assignment

## Docker Integration

- All development should work within Docker containers
- Use multi-stage builds for optimization
- Mount source code as volumes for development
- Ensure ROS2 workspace is properly sourced

## Build and Testing with Docker Compose

### Docker Compose Profiles

The project uses Docker Compose profiles for different deployment scenarios:

```bash
# Single robot simulation
docker compose --profile sim_scenario_1 up

# Multi-robot simulation (3 robots)
docker compose --profile sim_scenario up

# Development environment
docker compose --profile dev up -d
```

### Environment Variables

Key environment variables for Docker Compose:

- `WORLD_NAME`: Gazebo world to load (default: `racetrack_decorated`)
- `ROBOT_NUMBER`: Number of robots to spawn
- `ENABLE_GPU`: Enable GPU acceleration for Gazebo

### Docker Compose Commands

```bash
# Build all containers
docker compose build

# Build specific service
docker compose build auna

# Clean rebuild
docker compose build --no-cache

# Check container status
docker compose ps

# View logs
docker compose logs auna

# Enter running container
docker compose exec auna bash

# Stop all services
docker compose down

# Stop and remove volumes
docker compose down -v
```

### Testing in Docker Environment

```bash
# Build and test in container
docker compose run --rm auna bash -c "cd packages && colcon build --symlink-install && colcon test"

# Run specific package tests
docker compose run --rm auna bash -c "cd packages && source install/setup.bash && colcon test --packages-select package_name"

# Integration testing with simulation
docker compose --profile sim_scenario_1 up -d
# Wait for services to start
docker compose exec auna bash -c "cd packages && source install/setup.bash && ros2 topic list"
docker compose down
```

## Testing Guidelines

- Write unit tests for complex algorithms
- Use `gtest` for C++ testing
- Use `pytest` for Python testing
- Include integration tests for multi-component features
- Test in simulation before physical deployment

## Common Patterns to Follow

### Publishers and Subscribers

```cpp
// C++ Publisher pattern
auto publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(2));

// C++ Subscriber pattern
auto subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::QoS(2),
    std::bind(&NodeClass::callback, this, std::placeholders::_1));
```

### Parameter Handling

```cpp
// Declare and get parameters
this->declare_parameter("max_speed", 2.0);
double max_speed = this->get_parameter("max_speed").as_double();
```

### Timer Usage

```cpp
// Create timer for periodic operations
timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&NodeClass::timer_callback, this));
```

## Package-Specific Guidelines

### auna_cacc (CACC Controller)

- Implements platooning algorithms
- Focuses on vehicle following and coordination
- Uses CAM (Cooperative Awareness Messages) for communication
- Supports both string stability and safety requirements

### auna_comm (V2X Communication)

- Implements Vehicle-to-Everything (V2X) communication protocols
- Handles CAM (Cooperative Awareness Messages) broadcasting and reception
- Manages communication between multiple robots
- Supports network simulation integration with OMNeT++

### auna_common (Common Utilities)

- Shared utilities and helper functions across packages
- Common data structures and algorithms
- Utility classes for mathematical operations and transformations
- Shared constants and configuration helpers

### auna_control (Control Multiplexer)

- Manages input source switching
- Implements command velocity multiplexing
- Supports different control modes (teleop, nav2, cacc, wallfollowing)
- Provides seamless switching between autonomous and manual control

### auna_ekf (Extended Kalman Filter)

- Implements Extended Kalman Filter for robot localization
- Fuses sensor data (odometry, IMU, GPS) for state estimation
- Provides pose and velocity estimates for navigation
- Supports multi-robot localization scenarios

### auna_f110 (F1/10 Platform)

- F1/10 race car platform-specific implementations
- Hardware abstraction layer for physical F1/10 robots
- Sensor integration and calibration for F1/10 platform

### auna_gazebo (Simulation)

- Contains world files and robot models
- Implements ground truth localization
- Provides multi-robot spawning capabilities
- Manages Gazebo simulation environment and plugins

### auna_ground_truth (Ground Truth Localization)

- Provides perfect localization data from simulation
- Used for testing and validation of algorithms
- Publishes robot poses directly from Gazebo
- Supports multi-robot ground truth publishing

### auna_its_msgs (ITS Messages)

- Intelligent Transportation Systems message definitions
- Standard ITS message formats (CAM, DENM, etc.)
- Protocol-specific message structures
- Compatibility with automotive communication standards

### auna_msgs (Custom Messages)

- Custom ROS2 message and service definitions
- Project-specific data structures
- Service definitions for robot coordination
- Message types for control and communication

### auna_nav2 (Navigation)

- Integrates with Navigation2 stack
- Supports multi-robot navigation
- Includes custom behavior trees
- Path planning and obstacle avoidance algorithms

### auna_omnet (OMNeT++ Integration)

- Integration with OMNeT++ network simulator
- Network simulation for V2X communication
- Co-simulation between ROS2 and OMNeT++
- Network performance analysis and modeling

### auna_teleoperation (Manual Control)

- Manual keyboard and joystick control
- Teleop input processing and command generation
- Support for different input devices
- Safety mechanisms for manual control

### auna_template (Template Package)

- Template structure for creating new ROS2 packages
- Standard package layout and boilerplate code
- Example implementations of common patterns
- Development guidelines and best practices

### auna_tf (Transform Management)

- Transform frame management and broadcasting
- Coordinate frame transformations between robots
- Static and dynamic transform publishers
- Multi-robot coordinate frame coordination

### auna_wallfollowing (Wall Following)

- Wall-following algorithms using LIDAR data
- Reactive navigation for corridor and wall following
- Obstacle detection and avoidance
- Real-time path planning based on sensor data

### auna_waypoints (Waypoint Management)

- Waypoint processing and management
- Route planning and execution
- Waypoint file parsing and validation
- Integration with navigation stack for waypoint following

## Build and Testing Commands

```bash
# Build all packages
cd packages && colcon build --symlink-install

# Build specific package
cd packages && colcon build --packages-select package_name

# Source workspace
source packages/install/setup.bash

# Run tests
cd packages && colcon test --packages-select package_name
```

## Debugging Tips

1. Use ROS2 logging macros: `RCLCPP_INFO`, `RCLCPP_WARN`, `RCLCPP_ERROR`
2. Check topic connections: `ros2 topic list`, `ros2 topic echo`
3. Monitor node status: `ros2 node list`, `ros2 node info`
4. Use RViz for visualization and debugging
5. Check parameter values: `ros2 param list`, `ros2 param get`

## Security Considerations

- Validate input parameters and messages
- Use appropriate QoS settings for different use cases
- Implement proper error handling and recovery
- Consider network security for distributed deployments

## Performance Guidelines

- Use appropriate QoS settings (reliability, durability, history)
- Minimize memory allocations in real-time loops
- Use efficient data structures for algorithms
- Profile critical paths for optimization
- Consider using ROS2 composition for better performance

When suggesting code changes or new features, always consider:

- ROS2 best practices and conventions
- Multi-robot compatibility
- Docker container constraints
- Real-time performance requirements
- Safety and robustness for autonomous vehicles
