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
sudo apt-get -y install libspnav-dev libasio-dev libbluetooth-dev libcwiid-dev
colcon build --symlink-install
