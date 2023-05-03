#!/bin/sh

git clone https://github.com/IntelRealSense/librealsense.git ~/librealsense
sudo apt-get -y install git libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev
sudo apt-get -y install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

cd ~/librealsense
./scripts/setup_udev_rules.sh
./scripts/patch-realsense-ubuntu-lts.sh

mkdir build && cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release
sudo make uninstall && make clean && make -j 4 && sudo make install
