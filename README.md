# Autonomous Navigation System Simulator
___
This repository includes a complete ROS2 package for the simulation of autonomous robots. It features the simulation models, navigation algorithms and other components to run and evaluate cooperative driving scenarios. Each scenario can be extended to feature different robots, additional system components and more. The launch files are modularly built, so that each part can be configured without directly affecting the other components of the simulation.

Additionally, it integrates ROS2-Foxy with MATLAB/Simulink and OMNeT++/Artery, to enable the integration of control systems and communication standards. Currently, it includes a CACC-controller for platooning and an implementation of the ETSI-ITS-G5 communication architecture.

## Package Setup and Overview
___
### Installation

The following steps explain the required installation steps to run the framework on a machine running Ubuntu 20.04:

First, install the following packages and tools as described here:

    https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
    http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install
    https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html
    
Then, additionally install the following packages:
    
pip install ruamel.yaml
sudo apt install ros-foxy-xacro
sudo apt install ros-foxy-rmw-cyclonedds-cpp
sudo apt install ros-foxy-gazebo-ros-pkgs
sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup ros-foxy-turtlebot3 ros-foxy-turtlebot3-*

After that, build the package:

    colcon build --symlink-install
        
Run the following commands in the terminal before using ROS2:

    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export GAZEBO_MODEL_PATH=~/AuNa/src/car_simulator/models:$GAZEBO_MODEL_PATH
    
    export GAZEBO_MODEL_DATABASE_URI=http://models.gazebosim.org/
    export TURTLEBOT3_MODEL=waffle
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/foxy/share/turtlebot3_gazebo/models
    
    source /opt/ros/foxy/setup.bash
    source ~/AuNa/install/setup.bash
    
### MATLAB and Simulink

MATLAB does not currently support ROS2-Foxy by default. However, it is possible to connect MATLAB and ROS2 systems using the following steps:

First, install MATLAB and Simulink as described here:

    https://de.mathworks.com/help/install/
    https://de.mathworks.com/products/matlab.html
    https://de.mathworks.com/products/simulink.html

Install Python3.7:

    sudo add-apt-repository ppa:deadsnakes/ppa
    sudo apt-get update
    sudo apt install python3.7 python3.7-venv libpython3.7
    
Create symlinks to the Python3.7 installation:

    sudo ln -s /usr/lib/x86_64-linux-gnu/libpython3.7m.so.1 /usr/lib/libpython3.7m.so.1
    sudo ln -s /usr/lib/x86_64-linux-gnu/libpython3.7m.so.1.0 /usr/lib/libpython3.7m.so.1.0
    
Install numpy:

    sudo apt-get install python-numpy
    
Before launching MATLAB, you need to enter the following two commands in the terminal:

    source /usr/local/MATLAB/R2021b/sys/ros2/glnxa64/ros2/setup.bash
    export LD_PRELOAD="/usr/lib/x86_64-linux-gnu/libstdc++.so.6:/usr/lib/x86_64-linux-gnu/libcurl.so"
    
In every MATLAB script, you need to add the following line at the beginning:
    
    pyenv('Version','/usr/bin/python3.7');
    
After that, it should be possible for ROS2 and MATLAB/Simulink to communicate via DDS.

### OMNeT++ and Artery

OMNeT++ can be installed as described here:

    https://omnetpp.org/
    
After that, install the Artery framework. Clone the following GitHub repository:

    git clone --recurse-submodule https://github.com/HarunTeper/artery_ros2
    
Then, follow the installation guide here:

    http://artery.v2x-research.eu/install/
    
Build the artery_ros2 directory as follows:

    source /opt/ros/foxy/setup.bash
    mkdir build
    cd build
    cmake ..
    cmake --build .
	
### File stucture:
```
├── car_simulator
│   ├── car_simulator
│   │   └── yaml_launch.py  #Includes commands to read and configure .yaml files
│   ├── CMakeLists.txt
│   ├── config
│   │   ├── map_params #Parameter files for maps, such as spawn locations and others
│   │   ├── model_params # Model paramters of each robot model
│   │   ├── nav2_params # Navigation parameters for Nav2 nodes
│   │   └── scenario_params # Scenario parameters for robot nodes
│   ├── include # Include files for scripts in src
│   ├── launch # Launch files
│   │   ├── gazebo # Gazebo launch files for arbitrary world files
│   │   ├── navigation # Navigation launch files for single and multiple robots
│   │   ├── omnet # OMNeT++ launch files to launch bridge-nodes for communication with OMNeT++ and Artery
│   │   ├── scenarios # Currently implemented launch files for custom scenarios
│   │   └── spawn # Launch files to correctly spawn robots in Gazebo
│   ├── maps # Map files for Gazebo worlds
│   │   └── racetrack_decorated
│   ├── matlab
│   │   ├── CACC # Platooning controller implementation in MATLAB and Simulink
│   ├── models # Implemented robot models and world model files
│   │   ├── prius_custom
│   │   ├── race_car
│   │   └── racetrack
│   ├── package.xml
│   ├── rviz # RViz configuration files for scenarios
│   ├── scripts # Python scripts for scenarios and tools
│   │   ├── teleoperation # Scripts for keyboard controls
│   ├── src # C++ scripts
│   │   ├── omnet # OMNeT++ bridge-nodes
│   │   └── transformations # Transformation nodes for multi-robot setups
│   └── worlds # World files for Gazebo
├── etsi_its_msgs # ETSI-ITS-G5 messages for OMNeT++ and Artery
└── ros_its_msgs # CAM simple message format
```
	
## How to use?
___
## ROS2

After building the package, the currently implemented scenarios can be found in */src/car_simulator/launch/scenarios*. The multi-robot navigation scenario can be launched as follows:

    ros2 launch car_simulator scenario_multi_robot_racetrack.launch.py

Each launch file includes several parameters that can be adjusted. For example, the number of robots can be adjusted:

    ros2 launch car_simulator scenario_multi_robot_racetrack.launch.py robot_number:=3
    
## MATLAB and Simulink

In general, it is possible to integrate any MATLAB and Simulink script via the ROS2 publisher and subscriber functionalities.

An example is shown by the platooning controller, which can be found in *src/car_simulator/matlab/CACC*. It receives the current state of the direct leading vehicle and outputs the corresponding velocity and steering angle, so that a stable inter-vehicle distance is maintained.

## OMNeT++ and Artery

The wireless communication between robots is implemented via Artery, which implements the ETSI-ITS-G5 communication architecture. It is possible to add application-specific services to Artery in order to implement custom functionalities for communication.

For example, the platooning service is currently implemented in the ros2-platooning scenario in Artery. It detects whether or not a message originates from the direct leading vehicle and forwards these received messages to the platooning controller.

The scenario can be launched by running the following command

    cmake --build build --target run_ros2_platooning

In general, it is possible to add arbitrary services to Artery to evaluate other message formats or scenarios.

## Acknowledgements

We would like to thank all the authors who helped to extend the framework. In particular, we would like to thank Anggera Bayuwindra, Enio Prates Vasconcelos Filho, Raphael Riebl, and Ricardo Severino for providing their components and implementation details for the integration.























