#!/bin/sh
# ROS2
sudo apt-get -y install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt-get -y install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update
sudo apt-get -y upgrade
sudo apt-get -y install ros-galactic-desktop

echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc

# Gazebo

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get -y install gazebo11

# AuNa

sudo apt-get -y install python3-pip python3-colcon-common-extensions ros-galactic-xacro ros-galactic-rmw-cyclonedds-cpp ros-galactic-gazebo-ros-pkgs ros-galactic-navigation2 ros-galactic-nav2-bringup ros-galactic-libg2o
sudo apt-get -y install libspnav-dev libasio-dev libbluetooth-dev libcwiid-dev libgeographic-dev

# AuNa physical

sudo usermod -a -G dialout $USER

sudo apt-get -y install ros-galactic-ackermann-msgs ros-galactic-diagnostic-* ros-galactic-geographic-* ros-galactic-joy* ros-galactic-laser* ros-galactic-realsense2-* ros-galactic-robot-localization* ros-galactic-teleop-twist-joy* ros-galactic-serial-driver ros-galactic-udp-* ros-galactic-asio-cmake-module ros-galactic-io-context ros-galactic-urg-node* 

# MQTT
git clone https://github.com/eclipse/paho.mqtt.c.git ~/paho.mqtt.c
cd ~/paho.mqtt.c
git checkout v1.3.8
cmake -Bbuild -H. -DPAHO_ENABLE_TESTING=OFF -DPAHO_BUILD_STATIC=ON \
    -DPAHO_WITH_SSL=ON -DPAHO_HIGH_PERFORMANCE=ON
sudo cmake --build build/ --target install
sudo ldconfig

git clone https://github.com/eclipse/paho.mqtt.cpp ~/paho.mqtt.cpp
cd ~/paho.mqtt.cpp
cmake -Bbuild -H. -DPAHO_BUILD_STATIC=ON \
    -DPAHO_BUILD_DOCUMENTATION=TRUE -DPAHO_BUILD_SAMPLES=TRUE
sudo cmake --build build/ --target install
sudo ldconfig

# JSON
sudo apt-get -y install nlohmann-json3-dev

# Build

cd ~/AuNa
source /opt/ros/galactic/setup.bash
colcon build --symlink-install
