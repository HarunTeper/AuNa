# Autonomous Navigation System Simulator
___
This repository includes a complete ROS2 package for the simulation of autonomous robots. It features the simulation models, navigation algorithms and other components to run and evaluate cooperative driving scenarios. Each scenario can be extended to feature different robots, additional system components and more. The launch files are modularly built, so that each part can be configured without directly affecting the other components of the simulation.

Additionally, it integrates ROS2-Foxy with MATLAB/Simulink and OMNeT++/Artery, to enable the integration of control systems and communication standards. Currently, it includes a CACC-controller for platooning and an implementation of the ETSI-ITS-G5 communication architecture.

![](https://github.com/HarunTeper/AuNa/blob/main/media/gazeboSimulation.gif)

## Package Setup and Overview
___
### Installation

The following steps explain the required installation steps to run the framework on a machine running Ubuntu 20.04:

### Using Devcontainer and Dockerfile

1. Install Docker: Follow the instructions to install Docker on your system from https://docs.docker.com/get-docker/
2. Install Visual Studio Code: Download and install Visual Studio Code from https://code.visualstudio.com/
3. Install Remote - Containers extension: In Visual Studio Code, go to the Extensions view by clicking the Extensions icon in the Activity Bar on the side of the window. Search for "Remote - Containers" and install it.
4. Clone the repository: Clone this repository to your local machine.
5. Open the repository in Visual Studio Code: Open Visual Studio Code and use the "Open Folder" option to open the cloned repository.
6. Reopen in Container: Once the repository is open, you should see a notification prompting you to reopen the folder in a container. Click "Reopen in Container". If you don't see the notification, you can manually reopen in container by pressing `F1` and selecting "Remote-Containers: Reopen in Container".

### MATLAB and Simulink

First, install MATLAB and Simulink as described here:

    https://de.mathworks.com/help/install/
    https://de.mathworks.com/products/matlab.html
    https://de.mathworks.com/products/simulink.html

Install Python3.9:

    sudo add-apt-repository ppa:deadsnakes/ppa
    sudo apt-get update
    sudo apt install python3.9 python3.9-venv libpython3.9.so

Create symlinks to the Python3.9 installation:

    sudo ln -s /usr/lib/x86_64-linux-gnu/libpython3.9.so.1 /usr/lib/libpython3.9.so.1
    sudo ln -s /usr/lib/x86_64-linux-gnu/libpython3.9.so.1.0 /usr/lib/libpython3.9.so.1.0

Install numpy:

    sudo apt-get install python-numpy

In every MATLAB script, you need to add the following line at the beginning:

    pyenv('Version','/usr/bin/python3.9');

After that, ROS2 and MATLAB/Simulink are connected.

### OMNeT++ and Artery

Run the following commands to set up OMNeT++ and Artery:

    cd /workspace
    wget https://github.com/omnetpp/omnetpp/releases/download/omnetpp-5.6.2/omnetpp-5.6.2-src-linux.tgz
    tar -xvzf omnetpp-5.6.2-src-linux.tgz -C ~/
    rm omnetpp-5.6.2-src-linux.tgz
    cd ~/omnetpp-5.6.2
    ./configure
    make

    cd /workspace
    git clone --recurse-submodule https://github.com/HarunTeper/artery-ros2 ~/artery-ros2
    cd ~/artery-ros2
    mkdir build
    cd build
    cmake ..
    cmake -j 24 --build .

### File structure:
```
├── auna_cacc
│   ├── CACC # Includes the MATLAB script and Simulink model
│   └── launch # Launch files for the CACC controller
├── auna_common # Commonly used scripts and functions
│   └── auna_common # Scripts
├── auna_gazebo
│   ├── config # Configuration files
│   │   ├── map_params # Map specific parameters
│   │   └── model_params # Robot model specific parameters
│   ├── include # Header files
│   ├── launch # ROS2 launch files
│   ├── models # Robot model URDF files
│   ├── src # Source files
│   └── worlds # Gazebo world files
├── auna_its_msgs # Simplified CAM message
│   └── msg # Message folder
├── auna_msgs # Commonly used custom message types
│   ├── msg # Messages
│   └── srv # Services
├── auna_nav2
│   ├── config # Configuration files
│   │   └── nav2_params # Nav2 parameters
│   ├── launch # ROS2 launch files
│   ├── maps # Map-specific occupancy grid maps
│   └── rviz # Rviz configurations
├── auna_omnet
│   ├── include # Header files
│   ├── launch # ROS2 launch files
│   └── src # Source files
├── auna_scenarios # Compiled ROS2 launch files for scenarios
│   └── launch # ROS2 launch files
├── auna_teleoperation # Teleoperation scripts
│   └── scripts # Scripts
└── etsi_its_msgs # CAM Message
    └── msg # Messages
```

## How to use?
___
## ROS2

Run the platooning scenario using the following command:

    ros2 launch auna_scenarios scenario_platooning.launch.py

Adjust the number of robots using parameters:

    ros2 launch auna_scenarios scenario_platooning.launch.py robot_number:=3

## MATLAB and Simulink

In general, it is possible to integrate any MATLAB and Simulink script via the ROS2 publisher and subscriber functionalities.

An example is shown by the platooning controller, which can be found in *src/car_simulator/matlab/CACC*. It receives the current state of the direct leading vehicle and outputs the corresponding velocity and steering angle, so that a stable inter-vehicle distance is maintained.

## OMNeT++ and Artery

The scenario can be launched by running the following command

    cmake --build build --target run_ros2_platooning

After building, select the *Fast* option to run the simulation.

![](https://github.com/HarunTeper/AuNa/blob/main/media/omnetSimulation.gif)

## Using the Devcontainer for Development

The devcontainer is configured to provide a consistent development environment with all necessary dependencies installed. To use the devcontainer for development:

1. Open the repository in Visual Studio Code.
2. Reopen the folder in a container by clicking "Reopen in Container" when prompted or by pressing `F1` and selecting "Remote-Containers: Reopen in Container".
3. Once the container is running, you can use the integrated terminal in Visual Studio Code to run commands and build the project.

## Acknowledgements

We would like to thank all the authors who helped to extend the framework. In particular, we would like to thank Anggera Bayuwindra, Enio Prates Vasconcelos Filho, Raphael Riebl, and Ricardo Severino for providing their components and implementation details for the integration.
