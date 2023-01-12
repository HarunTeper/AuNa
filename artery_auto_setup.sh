#!/bin/sh

git clone https://github.com/geographiclib/geographiclib ~/GeographicLib
mkdir ~/GeographicLib/build
cd ~/GeographicLib/build
cmake ..
make -j 24
make test
sudo make install
sudo apt-get install libgeographic-dev

wget https://www.cryptopp.com/cryptopp870.zip
mkdir ~/cryptopp
unzip cryptopp870 -d ~/cryptopp
rm cryptopp870
mkdir ~/cryptopp/build
cd ~/cryptopp/build
cmake ..
cd .. 
make -j 24
sudo make install

source ~/AuNa/install/setup.bash
git clone --recurse-submodule https://github.com/HarunTeper/artery-ros2 ~/artery-ros2
cd ~/artery-ros2
mkdir build
cd build
cmake ..
cmake -j 24 --build .
