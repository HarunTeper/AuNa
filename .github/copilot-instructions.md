# Autonomous Navigation System Simulator (AuNa)

Always reference these instructions first and fallback to search or bash commands only when you encounter unexpected information that does not match the info here.

This repository contains a complete ROS2 Galactic workspace for autonomous vehicle simulation featuring Gazebo integration, MATLAB/Simulink control systems, and OMNeT++/Artery V2X communication protocols.

## Working Effectively

### Environment Requirements
- **CRITICAL**: This project requires Ubuntu 20.04 LTS - the auto_setup.sh script will NOT work on Ubuntu 22.04+ due to ROS2 Galactic availability
- ROS2 Galactic Desktop (NOT Humble/Iron/Jazzy)
- Gazebo 11
- Python 3.9 (for MATLAB integration)
- colcon build system
- MATLAB/Simulink (optional, for control system development)
- OMNeT++ 5.6.2 (optional, for V2X communication simulation)

### Bootstrap, Build, and Test the Repository

**NEVER CANCEL ANY BUILD COMMANDS** - Builds take 15-45 minutes depending on the system. Set timeouts to 60+ minutes.

1. **Initial Setup (Ubuntu 20.04 ONLY)**:
   ```bash
   # Run the complete setup script - takes 30-45 minutes, NEVER CANCEL
   ./auto_setup.sh
   ```
   
   **Manual Setup Alternative** (if auto_setup.sh fails):
   ```bash
   # Install ROS2 Galactic - takes 15-20 minutes, NEVER CANCEL
   sudo apt-get -y install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt-get -y install curl
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt-get update
   sudo apt-get -y install ros-galactic-desktop
   
   echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
   echo "export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp" >> ~/.bashrc
   source ~/.bashrc
   
   # Install build tools and dependencies - takes 10-15 minutes
   sudo apt-get -y install python3-pip python3-colcon-common-extensions ros-galactic-xacro ros-galactic-rmw-cyclonedx-cpp ros-galactic-gazebo-ros-pkgs ros-galactic-navigation2 ros-galactic-nav2-bringup ros-galactic-libg2o
   sudo apt-get -y install libspnav-dev libasio-dev libbluetooth-dev libcwiid-dev libgeographic-dev
   
   # Install Gazebo 11 - takes 5-10 minutes  
   sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
   wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
   sudo apt-get update
   sudo apt-get -y install gazebo11
   ```

2. **Build the Workspace** - takes 15-30 minutes, NEVER CANCEL:
   ```bash
   cd ~/AuNa
   git submodule update --init --recursive
   source /opt/ros/galactic/setup.bash
   colcon build --symlink-install
   ```
   
   **Expected Build Time**: 15-30 minutes on a 4-core system, up to 45 minutes on slower hardware.
   **CRITICAL**: Set timeout to 60+ minutes minimum. The build involves compiling C++ code, generating message types, and linking multiple packages.

3. **Source the Workspace**:
   ```bash
   source install/setup.bash
   ```

### Running and Testing

**Always source the workspace before running any ROS2 commands.**

1. **Test Basic Functionality**:
   ```bash
   # Test simple scenario - single robot on racetrack  
   ros2 launch auna_scenarios scenario_single_robot_racetrack.launch.py
   ```

2. **Run Platooning Scenario** (main use case):
   ```bash
   # Default 2 robots
   ros2 launch auna_scenarios scenario_platooning.launch.py
   
   # Custom number of robots (2-5 recommended)
   ros2 launch auna_scenarios scenario_platooning.launch.py robot_number:=3
   ```

3. **Multi-Robot Navigation**:
   ```bash
   ros2 launch auna_scenarios scenario_multi_robot_racetrack.launch.py robot_number:=3
   ```

### Validation Steps

**ALWAYS manually validate changes with these end-to-end scenarios:**

1. **Basic Build Validation**:
   ```bash
   # Verify all packages built successfully
   colcon list
   
   # Check for any build failures  
   cat log/latest/logger_all.log | grep -i error
   ```

2. **Runtime Validation**:
   ```bash
   # Launch single robot scenario and verify Gazebo opens with robot
   ros2 launch auna_scenarios scenario_single_robot_racetrack.launch.py world_name:=racetrack_decorated
   
   # In another terminal, verify robot topics are publishing
   ros2 topic list | grep -E "(cmd_vel|odom|scan)"
   
   # Verify robot responds to navigation commands
   ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.1}}" --once
   ```

3. **Navigation System Test**:
   ```bash
   # Launch arena navigation scenario
   ros2 launch auna_scenarios scenario_single_robot_arena_navigation.launch.py
   
   # Verify Nav2 is working by sending a goal
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}}}}"
   ```

### Optional: MATLAB/Simulink Integration

**Only proceed if you have MATLAB R2021a+ installed:**

1. **Setup Python 3.9 for MATLAB**:
   ```bash
   sudo add-apt-repository ppa:deadsnakes/ppa
   sudo apt-get update
   sudo apt install python3.9 python3.9-venv libpython3.9
   sudo ln -s /usr/lib/x86_64-linux-gnu/libpython3.9.so.1 /usr/lib/libpython3.9.so.1
   sudo ln -s /usr/lib/x86_64-linux-gnu/libpython3.9.so.1.0 /usr/lib/libpython3.9.so.1.0
   ```

2. **Test MATLAB Integration**:
   ```bash
   # Launch CACC controller (requires MATLAB)
   ros2 launch auna_cacc cacc_controller.launch.py robot_number:=2
   ```

### Optional: OMNeT++/Artery Integration

**Only proceed if you need V2X communication simulation:**

1. **Install OMNeT++** - takes 30-60 minutes, NEVER CANCEL:
   ```bash
   ./omnet_auto_setup.sh
   ```

2. **Install Artery** - takes 20-30 minutes, NEVER CANCEL:
   ```bash
   ./artery_auto_setup.sh
   ```

3. **Test OMNeT++ Integration**:
   ```bash
   cmake --build build --target run_ros2_platooning
   ```

## Common Tasks

### Repository Structure
```
├── auto_setup.sh              # Main setup script for Ubuntu 20.04
├── omnet_auto_setup.sh         # OMNeT++ installation
├── artery_auto_setup.sh        # Artery V2X framework setup  
├── src/
│   ├── auna_gazebo/            # Gazebo simulation components
│   ├── auna_nav2/              # Navigation2 integration
│   ├── auna_scenarios/         # Launch files for different scenarios
│   ├── auna_msgs/              # Custom ROS2 message types
│   ├── auna_cacc/              # CACC controller (MATLAB)
│   ├── auna_omnet/             # OMNeT++ integration
│   └── physical/               # Physical robot interfaces
```

### Available Worlds and Scenarios
```bash
# Available Gazebo worlds (in src/auna_gazebo/worlds/):
# - empty_world.world
# - racetrack.world  
# - arena.world
# - racetrack_decorated.world

# Launch specific worlds:
ros2 launch auna_scenarios scenario_single_robot_racetrack.launch.py world_name:=arena
ros2 launch auna_scenarios scenario_single_robot_racetrack.launch.py world_name:=racetrack_decorated
```

### Key Launch Files
- `scenario_platooning.launch.py` - Main CACC platooning scenario
- `scenario_single_robot_racetrack.launch.py` - Single robot testing  
- `scenario_multi_robot_racetrack.launch.py` - Multi-robot coordination
- `scenario_*_arena_*` - Indoor navigation scenarios
- `rviz.launch.py` - Visualization only

### Build Configuration
- **Build System**: colcon (ROS2 standard)
- **Install Type**: `--symlink-install` (for development)
- **Parallel Jobs**: Defaults to CPU core count
- **Dependencies**: Automatically resolved via package.xml files
- **Code Style**: Uses clang-format with Google style (see .clang-format)

### Code Formatting
```bash
# Format C++ code (if clang-format is installed)
find src/ -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i

# Always format code before committing changes
```

### Troubleshooting

**Build Failures:**
- Ensure Ubuntu 20.04 (NOT 22.04+)
- Verify ROS2 Galactic installation: `ros2 --version`
- Check for missing dependencies: `rosdep install --from-paths src --ignore-src -r -y`
- Clean build if needed: `rm -rf build install log && colcon build --symlink-install`

**Runtime Issues:**
- Always source workspace: `source install/setup.bash`  
- Check Gazebo installation: `gazebo --version` (should be 11.x)
- Verify DISPLAY is set for GUI applications
- Check for port conflicts: `netstat -tlnp | grep :11345`

**Network/Repository Issues:**
- If packages.ros.org is unreachable, wait and retry
- Use local mirror if available
- Check firewall settings for apt repositories

### Performance Notes
- **Memory**: Minimum 8GB RAM, 16GB+ recommended for multi-robot scenarios
- **CPU**: 4+ cores recommended for reasonable build times
- **Storage**: ~10GB for full installation with dependencies
- **GPU**: Not required but helpful for Gazebo visualization

### NEVER Cancel These Operations
- `./auto_setup.sh` (30-45 minutes)
- `colcon build` (15-45 minutes)  
- `./omnet_auto_setup.sh` (30-60 minutes)
- `./artery_auto_setup.sh` (20-30 minutes)
- Always set timeouts to 60+ minutes for build commands

Always use the EXACT timeout values and cancellation warnings specified above to prevent incomplete builds that can corrupt the workspace.