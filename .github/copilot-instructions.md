# Autonomous Navigation System Simulator (AuNa)

Always follow these instructions first and fallback to search or bash commands only when you encounter unexpected information that does not match the info here.

AuNa is a complete ROS2-based autonomous vehicle simulation framework integrating Gazebo, MATLAB/Simulink, and OMNeT++/Artery for cooperative driving scenarios. It features CACC controllers, platooning scenarios, and ETSI-ITS-G5 communication standards.

## Working Effectively

### CRITICAL Platform Requirements
- **Ubuntu 20.04 ONLY** - Setup scripts fail on other versions
- **ROS2 Galactic** - Specific version required
- **Gazebo 11** - Simulation environment
- **Python 3.9** - Required for MATLAB integration

### Bootstrap, Build, and Test Repository

**NEVER CANCEL any build or test commands - they may take 45+ minutes to complete.**

#### Environment Setup (45-60 minutes total)
```bash
# 1. System Setup (10-15 minutes) - NEVER CANCEL, set timeout to 20+ minutes
export DEBIAN_FRONTEND=noninteractive
echo '' | sudo add-apt-repository universe
time ./auto_setup.sh
```
**Known Issue**: `auto_setup.sh` may fail due to firewall restrictions accessing packages.ros.org. If this occurs, install ROS2 Galactic manually using the commands from the script.

#### Manual ROS2 Installation (if auto_setup.sh fails due to firewall)
```bash
# Add ROS2 repository manually
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update
sudo apt-get -y upgrade
sudo apt-get -y install ros-galactic-desktop

# Environment configuration
echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp" >> ~/.bashrc
source ~/.bashrc
```

#### Initialize Repository (2-3 minutes)
```bash
# Initialize submodules - takes ~1.3 seconds
git submodule update --init --recursive

# Source ROS2 environment
source /opt/ros/galactic/setup.bash
```

#### Build Process (30-45 minutes) - NEVER CANCEL
```bash
# Build all packages - NEVER CANCEL, set timeout to 60+ minutes
time colcon build --symlink-install
```
**Expected Time**: 30-45 minutes for full build. **NEVER CANCEL** - builds may appear to hang but are processing.

#### Test and Validation (15-20 minutes) - NEVER CANCEL  
```bash
# Run linting tests - NEVER CANCEL, set timeout to 30+ minutes
colcon test --packages-select auna_cacc auna_gazebo auna_nav2
colcon test-result --verbose
```

### Optional Component Setup

#### MATLAB and Simulink Integration (20-30 minutes)
```bash
# Install Python 3.9 for MATLAB integration
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt-get update
sudo apt install python3.9 python3.9-venv libpython3.9

# Create required symlinks
sudo ln -s /usr/lib/x86_64-linux-gnu/libpython3.9.so.1 /usr/lib/libpython3.9.so.1
sudo ln -s /usr/lib/x86_64-linux-gnu/libpython3.9.so.1.0 /usr/lib/libpython3.9.so.1.0

# Install numpy
sudo apt-get install python-numpy
```

#### OMNeT++ and Artery Setup (60+ minutes) - NEVER CANCEL
```bash
# OMNeT++ installation - NEVER CANCEL, set timeout to 90+ minutes
time ./omnet_auto_setup.sh

# Artery installation - NEVER CANCEL, set timeout to 60+ minutes  
time ./artery_auto_setup.sh
```

## Running the System

### Basic Simulation Startup
```bash
# ALWAYS source environment first
source /opt/ros/galactic/setup.bash
source install/setup.bash

# Launch basic platooning scenario (default 2 robots)
ros2 launch auna_scenarios scenario_platooning.launch.py

# Launch with multiple robots
ros2 launch auna_scenarios scenario_platooning.launch.py robot_number:=3

# Launch single robot navigation
ros2 launch auna_scenarios scenario_single_robot_arena_navigation.launch.py
```

### Available Scenarios
- `scenario_platooning.launch.py` - Multi-robot platooning with CACC
- `scenario_multi_robot_racetrack.launch.py` - Multiple robots on racetrack
- `scenario_single_robot_arena_navigation.launch.py` - Navigation in arena
- `scenario_mapping_robot_racetrack.launch.py` - SLAM on racetrack

### Available Worlds
- `arena` - Indoor navigation environment
- `empty_world` - Basic testing environment  
- `racetrack` - Simple racetrack
- `racetrack_decorated` - Enhanced racetrack with objects

## Validation Scenarios

**ALWAYS run these complete end-to-end validation scenarios after making changes:**

### 1. Basic Build Validation
```bash
# Verify clean build
colcon build --symlink-install --packages-select auna_gazebo auna_scenarios
echo "Build validation: PASS if no errors above"
```

### 2. Gazebo Simulation Validation  
```bash
# Test Gazebo launches and robots spawn correctly
ros2 launch auna_scenarios scenario_single_robot_racetrack.launch.py &
sleep 30
# Verify Gazebo window opens and robot appears in racetrack world
# Verify ROS2 topics are publishing: ros2 topic list | grep robot0
pkill -f gazebo
```

### 3. Multi-Robot Platooning Validation
```bash
# Test CACC platooning scenario
ros2 launch auna_scenarios scenario_platooning.launch.py robot_number:=2 &
sleep 45
# Verify 2 robots spawn and follow each other
# Check CACC controller topics: ros2 topic list | grep cacc
pkill -f gazebo
```

### 4. Navigation Validation
```bash
# Test autonomous navigation  
ros2 launch auna_scenarios scenario_single_robot_arena_navigation.launch.py &
sleep 30
# Verify Nav2 stack launches and robot navigates autonomously
# Check navigation topics: ros2 topic list | grep nav
pkill -f gazebo
```

**Critical**: If any validation scenario fails, investigate before proceeding. Common issues:
- Missing ROS2 environment sourcing
- Incomplete build due to missing dependencies
- Gazebo not finding world files

## Development Workflow

### Before Making Changes
```bash
# ALWAYS ensure environment is sourced
source /opt/ros/galactic/setup.bash
source install/setup.bash

# ALWAYS run clean build test
colcon build --symlink-install --packages-select [affected-packages]
```

### After Making Changes
```bash
# Build affected packages only for faster iteration
colcon build --symlink-install --packages-select [changed-packages]

# Run relevant tests  
colcon test --packages-select [changed-packages]

# ALWAYS test at least one complete scenario
ros2 launch auna_scenarios scenario_single_robot_racetrack.launch.py
```

### Linting and Code Quality
```bash
# ALWAYS run linting before committing
colcon test --packages-select [your-packages]
colcon test-result --verbose

# Check for common issues
grep -r "TODO\|FIXME\|HACK" src/
```

## Troubleshooting

### Common Build Issues
- **ament_cmake not found**: ROS2 not properly installed or sourced
- **Package not found**: Missing dependencies, run `rosdep install --from-paths src --ignore-src -r -y`
- **Gazebo models not found**: Run `export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$(pwd)/src/auna_gazebo/models`

### Performance Issues  
- **Slow simulation**: Reduce robot_number parameter
- **Build timeouts**: Increase timeout values, builds genuinely take 30-45 minutes
- **Memory issues**: Close other applications, simulation requires 8GB+ RAM

### Network Issues
- **ROS2 packages not downloading**: Firewall may block packages.ros.org - install manually
- **Submodule clone failures**: Check network connectivity to GitHub

## Common Tasks

### Package Structure Reference
```
src/
├── auna_cacc/          # CACC platooning controller
├── auna_gazebo/        # Simulation models and worlds  
├── auna_scenarios/     # Launch files for complete scenarios
├── auna_nav2/          # Navigation stack configuration
├── auna_omnet/         # OMNeT++ integration
├── auna_mqtt/          # MQTT communication
├── etsi_its_msgs/      # ETSI-ITS message definitions
└── physical/           # Physical robot integration
```

### Frequently Used Commands
```bash
# Check running nodes
ros2 node list

# Monitor robot topics  
ros2 topic list | grep robot0
ros2 topic echo /robot0/odom

# Debug CACC controller
ros2 topic echo /robot1/cacc_cmd_vel

# Launch RViz for visualization
ros2 launch auna_nav2 rviz.launch.py
```

### Key Configuration Files
- `src/auna_gazebo/config/model_params/` - Robot model parameters
- `src/auna_nav2/config/nav2_params/` - Navigation parameters  
- `src/auna_scenarios/launch/` - All scenario launch files
- `src/auna_gazebo/worlds/` - Gazebo world definitions

## Integration Notes

### MATLAB Integration
- **Required**: Add `pyenv('Version','/usr/bin/python3.9');` to every MATLAB script
- **CACC Controller**: Located in `src/auna_cacc/CACC/`
- **Testing**: Run MATLAB scripts to verify ROS2 connectivity

### OMNeT++ Integration  
- **Build Command**: `cmake --build build --target run_ros2_platooning`
- **Simulation Mode**: Select 'Fast' option when prompted
- **Prerequisites**: INET framework must be installed in OMNeT++ IDE

### Physical Robot Integration
- **Hardware**: Supports Logitech G29 steering wheel, VESC motor controllers
- **Setup**: Run `./udev.setup.sh` for device permissions
- **Configuration**: Modify `src/physical/ros-g29-force-feedback/config/g29.yaml`

## Critical Reminders

- **NEVER CANCEL** long-running builds or tests - they may take 45+ minutes
- **ALWAYS** source ROS2 environment before any commands  
- **ALWAYS** run at least one complete validation scenario after changes
- **Ubuntu 20.04 required** - setup fails on other versions
- **Set timeouts to 60+ minutes** for build commands, 30+ minutes for tests
- **Firewall issues expected** - document failures and provide alternatives