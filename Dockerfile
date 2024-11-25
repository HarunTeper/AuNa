# Use ubuntu:22.04 as the base image
FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    software-properties-common \
    curl \
    wget \
    git \
    build-essential \
    cmake \
    python3-pip \
    python3-colcon-common-extensions \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libspnav-dev \
    libasio-dev \
    libbluetooth-dev \
    libcwiid-dev \
    libgeographic-dev \
    libosgearth-dev \
    mpi-default-dev \
    default-jre \
    qtbase5-dev \
    qtchooser \
    qt5-qmake \
    qtbase5-dev-tools \
    libqt5opengl5-dev \
    libxml2-dev \
    zlib1g-dev \
    doxygen \
    graphviz \
    libwebkit2gtk-4.0-37 \
    ros-galactic-desktop \
    ros-galactic-xacro \
    ros-galactic-rmw-cyclonedds-cpp \
    ros-galactic-gazebo-ros-pkgs \
    ros-galactic-navigation2 \
    ros-galactic-nav2-bringup \
    ros-galactic-libg2o \
    ros-galactic-ackermann-msgs \
    ros-galactic-diagnostic-* \
    ros-galactic-geographic-* \
    ros-galactic-joy* \
    ros-galactic-laser* \
    ros-galactic-realsense2-* \
    ros-galactic-robot-localization* \
    ros-galactic-teleop-twist-joy* \
    ros-galactic-serial-driver \
    ros-galactic-udp-* \
    ros-galactic-asio-cmake-module \
    ros-galactic-io-context \
    ros-galactic-urg-node* \
    nlohmann-json3-dev

# Install ROS2 and Gazebo
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    apt-get update && \
    apt-get install -y ros-galactic-desktop && \
    echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc && \
    source ~/.bashrc && \
    sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt-get update && \
    apt-get install -y gazebo11

# Install Python packages
RUN pip3 install scipy

# Clone and build required repositories
RUN git clone --recurse-submodule https://github.com/HarunTeper/artery-ros2 ~/artery-ros2 && \
    cd ~/artery-ros2 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    cmake -j 24 --build . && \
    git clone https://github.com/eclipse/paho.mqtt.c.git ~/paho.mqtt.c && \
    cd ~/paho.mqtt.c && \
    git checkout v1.3.8 && \
    cmake -Bbuild -H. -DPAHO_ENABLE_TESTING=OFF -DPAHO_BUILD_STATIC=ON -DPAHO_WITH_SSL=ON -DPAHO_HIGH_PERFORMANCE=ON && \
    cmake --build build/ --target install && \
    ldconfig && \
    git clone https://github.com/eclipse/paho.mqtt.cpp ~/paho.mqtt.cpp && \
    cd ~/paho.mqtt.cpp && \
    cmake -Bbuild -H. -DPAHO_BUILD_STATIC=ON -DPAHO_BUILD_DOCUMENTATION=TRUE -DPAHO_BUILD_SAMPLES=TRUE && \
    cmake --build build/ --target install && \
    ldconfig && \
    git clone https://github.com/omnetpp/omnetpp/releases/download/omnetpp-5.6.2/omnetpp-5.6.2-src-linux.tgz && \
    tar -xvzf omnetpp-5.6.2-src-linux.tgz -C ~/ && \
    rm omnetpp-5.6.2-src-linux.tgz && \
    cd ~/omnetpp-5.6.2 && \
    apt-get update && \
    apt-get install -y build-essential clang lld gdb bison flex perl python3 python3-pip qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools libqt5opengl5-dev libxml2-dev zlib1g-dev doxygen graphviz libwebkit2gtk-4.0-37 && \
    python3 -m pip install --user --upgrade numpy pandas matplotlib scipy seaborn posix_ipc && \
    apt-get install -y openscenegraph-plugin-osgearth libosgearth-dev mpi-default-dev default-jre && \
    echo "export OMNETPP_PATH=~/omnetpp-5.6.2" >> ~/.bashrc && \
    echo "export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${OMNETPP_PATH}/lib" >> ~/.bashrc && \
    echo "export PATH=${PATH}:${OMNETPP_PATH}/bin" >> ~/.bashrc && \
    git clone https://github.com/IntelRealSense/librealsense.git ~/librealsense && \
    cd ~/librealsense && \
    ./scripts/setup_udev_rules.sh && \
    ./scripts/patch-realsense-ubuntu-lts.sh && \
    mkdir build && \
    cd build && \
    cmake ../ -DCMAKE_BUILD_TYPE=Release && \
    make uninstall && make clean && make -j 4 && make install && \
    cd ~/AuNa && \
    git submodule update --init --recursive && \
    source /opt/ros/galactic/setup.bash && \
    colcon build --symlink-install
