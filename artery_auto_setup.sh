#!/bin/sh

sudo apt-get -y install libgeographic-dev libcrypto++-dev

source ~/AuNa/install/setup.bash
git clone --recurse-submodule https://github.com/HarunTeper/artery-ros2 ~/artery-ros2
cd ~/artery-ros2
mkdir build
cd build
cmake ..
cmake -j 24 --build .
