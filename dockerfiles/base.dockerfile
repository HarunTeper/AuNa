FROM ros:humble-ros-base

# Set build arguments
ARG ROS_DISTRO=humble
ARG PIP_BREAK_SYSTEM_PACKAGES=1
ARG HOST_UID=1000
ARG HOST_GID=1000
ARG ROS_DOMAIN_ID=12

# Configure environment
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
ENV TZ=Etc/UTC
ENV ROS_DISTRO=${ROS_DISTRO}
ENV PIP_BREAK_SYSTEM_PACKAGES=${PIP_BREAK_SYSTEM_PACKAGES}

# Install essential packages
RUN apt-get update && apt-get upgrade -y --no-install-recommends \
    && apt-get install -y --no-install-recommends \
    python3-pip \
    git \
    bash-completion \
    curl \
    wget \
    sudo \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    lttng-tools \
    liblttng-ust-dev \
    python3-babeltrace \
    python3-lttng \
    lttng-modules-dkms \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Create non-root user for security
RUN groupadd -g ${HOST_GID} ubuntu \
    && useradd -m -u ${HOST_UID} -g ${HOST_GID} -s /bin/bash ubuntu \
    && echo 'ubuntu ALL=(root) NOPASSWD:ALL' > /etc/sudoers.d/ubuntu \
    && chmod 0440 /etc/sudoers.d/ubuntu \
    && usermod -aG sudo ubuntu \
    && touch /home/ubuntu/.sudo_as_admin_successful

# Switch to non-root user
USER ubuntu

# Set up work directory with proper ownership
WORKDIR /home/ubuntu/workspace
RUN sudo chown -R ubuntu:ubuntu /home/ubuntu/workspace

# Build ros2_tracing from source
RUN mkdir -p /home/ubuntu/tracing/src \
    && cd /home/ubuntu/tracing/src \
    && git clone https://gitlab.com/ros-tracing/ros2_tracing.git -b ${ROS_DISTRO} \
    && cd .. \
    && bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build" \
    && sudo usermod -aG tracing ubuntu

# Enhanced bash configuration with environment variable support
RUN echo 'PS1="\[\033[32m\]\u\[\033[0m\] ➜ \[\033[34m\]\w\[\033[31m\]\$(__git_ps1 \" (%s)\")\[\033[0m\] $ "' >> ~/.bashrc \
    && echo "source /opt/ros/\${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo 'source /home/ubuntu/tracing/install/setup.bash' >> ~/.bashrc \
    && echo 'export RMW_IMPLEMENTATION=rmw_zenoh_cpp' >> ~/.bashrc

SHELL ["/bin/bash", "-c"]
