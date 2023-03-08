#!/bin/sh

wget https://github.com/omnetpp/omnetpp/releases/download/omnetpp-5.6.2/omnetpp-5.6.2-src-linux.tgz
tar -xvzf omnetpp-5.6.2-src-linux.tgz -C ~/
rm omnetpp-5.6.2-src-linux.tgz

# Installing OMNeT++ dependencies (taken from https://doc.omnetpp.org/omnetpp/InstallGuide.pdf):

sudo apt-get update
sudo apt-get -y install build-essential clang lld gdb bison flex perl python3 python3-pip qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools libqt5opengl5-dev libxml2-dev zlib1g-dev doxygen graphviz libwebkit2gtk-4.0-37
python3 -m pip install --user --upgrade numpy pandas matplotlib scipy seaborn posix_ipc
sudo apt-get -y install openscenegraph-plugin-osgearth libosgearth-dev
sudo apt-get -y install mpi-default-dev
sudo apt-get -y install default-jre

echo "export OMNETPP_PATH=~/omnetpp-5.6.2" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${OMNETPP_PATH}/lib" >> ~/.bashrc
echo "export PATH=${PATH}:${OMNETPP_PATH}/bin" >> ~/.bashrc

# Install OMNeT++ using the commands in the INSTALL file in ~/omnetpp-5.6.2

# After launching OMNeT++, install the INET framework using the OMNeT++ IDE. It should either automatically open a prompt to install it, or go to Help->Install Simulation Models... to install it.
