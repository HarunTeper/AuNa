FROM ros:humble

# Configure environment
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=12
ENV TZ=Etc/UTC

# Update package lists once at the beginning
RUN apt-get update

# Upgrade system and install python3-pip
RUN apt-get upgrade -y --no-install-recommends \
    && apt-get install -y --no-install-recommends \
    python3-pip

# Install git and bash-completion for git prompt
RUN apt-get install -y git bash-completion

# Install zenohd
RUN apt-get install -y ros-humble-rmw-zenoh-cpp

# Install tracing tools
RUN apt-get install -y \
    lttng-tools \
    liblttng-ust-dev \
    python3-babeltrace \
    python3-lttng \
    lttng-modules-dkms

# Create ubuntu user and configure sudo access
RUN useradd -m -s /bin/bash ubuntu \
    && echo 'ubuntu ALL=(root) NOPASSWD:ALL' > /etc/sudoers.d/ubuntu \
    && chmod 0440 /etc/sudoers.d/ubuntu

ARG HOST_UID=1000
ARG HOST_GID=1000
# Set user and group IDs to match the host
RUN usermod -u ${HOST_UID} ubuntu \
    && groupmod -g ${HOST_GID} ubuntu \
    && usermod -aG sudo ubuntu

# Disable sudo hint message
RUN touch /home/ubuntu/.sudo_as_admin_successful

USER ubuntu

# Set up work directory
WORKDIR /home/ubuntu/workspace

# Fix ownership of workspace directory
RUN sudo chown -R ubuntu:ubuntu /home/ubuntu/workspace

# Clone ros2_tracing and build tracetools
RUN /bin/bash -c "cd /home/ubuntu \
    && mkdir -p /home/ubuntu/tracing/src \
    && cd /home/ubuntu/tracing/src \
    && git clone https://gitlab.com/ros-tracing/ros2_tracing.git \
    && cd .. \
    && source /opt/ros/humble/setup.bash \
    && colcon build --packages-up-to tracetools ros2trace"

RUN sudo usermod -aG tracing ubuntu

# Colored terminal
RUN echo 'PS1="\[\033[32m\]\u\[\033[0m\] ➜ \[\033[34m\]\w\[\033[31m\]\$(__git_ps1 \" (%s)\")\[\033[0m\] $ "' >> ~/.bashrc

# Source ROS2 in bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
RUN echo 'source /home/ubuntu/tracing/install/setup.bash' >> ~/.bashrc

# Set default shell to /bin/bash with -c flag
SHELL ["/bin/bash", "-c"]