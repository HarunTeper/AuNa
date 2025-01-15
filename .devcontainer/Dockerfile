ARG VARIANT
FROM mcr.microsoft.com/devcontainers/base:${VARIANT}

# --- Configure environment ---

# Configure DEBIAN_FRONTEND to avoid warnings
ENV DEBIAN_FRONTEND=noninteractive

# set up work directory
WORKDIR /home/vscode/workspace

# --- Install ROS2 ---
ARG INSTALL_ROS2=true
RUN if [ "$INSTALL_ROS2" = "true" ]; then \
    apt-get update && apt-get install -y locales && \
    locale-gen en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    apt-get update && \
    apt-get install -y software-properties-common curl git && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y ros-humble-desktop && \
    apt-get install -y libgeographic-dev && \
    apt-get install -y libunwind-dev && \
    apt-get -y install python3-pip python3-colcon-common-extensions ros-dev-tools; \
    fi

# --- Install additional ROS2 packages ---
RUN if [ "$INSTALL_ROS2" = "true" ]; then \
    apt-get install -y ros-humble-xacro ros-humble-udp-msgs ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-etsi-its-messages ros-humble-gazebo-msgs; \
    fi

# --- Install Gazebo ---
ARG INSTALL_GAZEBO=true
RUN if [ "$INSTALL_GAZEBO" = "true" ]; then \
    sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - && \
    apt-get update && \
    apt-get install -y gazebo && \
    apt-get install -y ros-humble-gazebo-ros-pkgs; \
    fi

# Expose the Gazebo port
EXPOSE 11345

# --- Install ROS2 tracing ---
ARG INSTALL_ROS2_TRACING=true
RUN if [ "$INSTALL_ROS2_TRACING" = "true" ]; then \
    apt-get install -y lttng-tools liblttng-ust-dev && \
    apt-get install -y python3-babeltrace python3-lttng && \
    apt-get install -y lttng-modules-dkms && \
    groupadd -f -r tracing && \
    usermod -a -G tracing vscode && \
    mkdir -p /home/vscode/ros2_tracing_ws/src/ros2_tracing && \
    git clone -b humble https://gitlab.com/ros-tracing/ros2_tracing.git /home/vscode/ros2_tracing_ws/src/ros2_tracing && \
    mkdir -p /home/vscode/ros2_tracing_ws/src/tracetools_analysis && \
    git clone -b humble https://github.com/ros-tracing/tracetools_analysis.git /home/vscode/ros2_tracing_ws/src/tracetools_analysis && \
    cd /home/vscode/ros2_tracing_ws &&\
    . /opt/ros/humble/setup.sh &&\
    colcon build --symlink-install --allow-overriding tracetools && \
    pip install bokeh jupyter; \
    fi

# --- Install ROS 2 Extensions ---

RUN if [ "$INSTALL_ROS2" = "true" ]; then \
    apt-get update && \
    apt-get install -y ros-humble-rmw-cyclonedds-cpp; \
    fi

# --- Install DevTools ---
RUN apt-get update && \
    apt-get -y install \
    clangd \
    clang \
    clang-format \
    clang-tidy \
    cppcheck \
    build-essential \
    cmake \
    libc++-dev \
    libstdc++-12-dev \
    libc++abi-dev && \
    pip install pep8 pylint

# --- Install ROS 2 packages for F1/10 ---
RUN if [ "$INSTALL_ROS2" = "true" ]; then \
    apt-get install -y ros-humble-ackermann-msgs ros-humble-urg-node ros-humble-serial-driver ros-humble-asio-cmake-module; \
    fi

# --- Set up user ---

# set user
RUN usermod -a -G dialout vscode
USER vscode

# source ROS2 setup file and workspace
ARG WORKSPACE
RUN if [ "$INSTALL_ROS2" = "true" ]; then \
    echo '. /opt/ros/humble/setup.sh' >> /home/vscode/.bashrc && \
    echo "if [ -f ~/workspace/packages/install/setup.bash ]; then source ~/workspace/packages/install/setup.bash; fi" >> /home/vscode/.bashrc && \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
    CYCLONEDDS_URI=/home/vscode/workspace/cyclone_dds_config.xml; \
    fi

# source ROS2 tracing setup file
RUN if [ "$INSTALL_ROS2_TRACING" = "true" ]; then \
    echo '. /home/vscode/ros2_tracing_ws/install/setup.sh' >> /home/vscode/.bashrc; \
    fi

# configure environment variables
ENV DISPLAY=${DISPLAY}
ENV ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}
ENV ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# --- Set up entrypoint ---

SHELL ["/bin/bash", "-c"]