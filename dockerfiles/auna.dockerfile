# syntax=docker/dockerfile:1-labs
#------------------------------------------------------------------------------
# ROS2 DOCKERFILE
#------------------------------------------------------------------------------

# Build arguments
ARG ROS_DISTRO=humble
ARG PIP_BREAK_SYSTEM_PACKAGES=1
ARG HOST_UID=1000
ARG HOST_GID=1000
ARG ROS_DOMAIN_ID=0
ARG RMW_IMPLEMENTATION=rmw_zenoh_cpp
ARG PACKAGE_NAMES=""
ARG INSTALL_PACKAGE_NAMES=""

#------------------------------------------------------------------------------
# BASE STAGE - Install system dependencies and create user
#------------------------------------------------------------------------------
FROM ros:${ROS_DISTRO}-ros-base AS base-stage

ARG ROS_DISTRO
ARG PIP_BREAK_SYSTEM_PACKAGES
ARG HOST_UID
ARG HOST_GID
ARG ROS_DOMAIN_ID
ARG RMW_IMPLEMENTATION

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
    babeltrace2 \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Create non-root user for security
RUN groupadd -g ${HOST_GID} ubuntu \
    && useradd -m -u ${HOST_UID} -g ${HOST_GID} -s /bin/bash ubuntu \
    && echo 'ubuntu ALL=(root) NOPASSWD:ALL' > /etc/sudoers.d/ubuntu \
    && chmod 0440 /etc/sudoers.d/ubuntu \
    && usermod -aG sudo ubuntu \
    && usermod -aG tracing ubuntu \
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
    && bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

# Enhanced bash configuration
RUN echo 'PS1="\\[\\033[32m\\]\\u\\[\\033[0m\\] ➜ \\[\\033[34m\\]\\w\\[\\033[31m\\]\\$(__git_ps1 \\" (%s)\\")\\[\\033[0m\\] $ "' >> ~/.bashrc \
    && echo "source /opt/ros/\${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo 'source /home/ubuntu/tracing/install/setup.bash' >> ~/.bashrc \
    && echo 'export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}' >> ~/.bashrc

#------------------------------------------------------------------------------
# DEPENDENCY STAGE - Handle package dependencies with smart nested package support
#------------------------------------------------------------------------------
FROM base-stage AS dependency-stage

ARG PACKAGE_NAMES
ARG INSTALL_PACKAGE_NAMES

USER root

# Copy dependency files using --parents to preserve nested structure
COPY --parents packages/src/**/package.xml /tmp/
COPY --parents packages/src/**/requirements.txt /tmp/
COPY --parents packages/src/**/pyproject.toml /tmp/
COPY --parents packages/src/**/setup.cfg /tmp/

# Fix ownership after COPY
RUN chown -R ubuntu:ubuntu /tmp/packages

USER ubuntu

# Update rosdep database
RUN sudo apt-get update && rosdep update

# Install ROS dependencies
RUN if [ -d "/tmp/packages" ]; then \
    echo "Installing ROS dependencies..."; \
    rosdep install --from-paths /tmp/packages --ignore-src -r -y \
    || echo "Warning: Some rosdep installations failed"; \
    fi

# Install Python dependencies
RUN if [ -d "/tmp/packages" ]; then \
    echo "Installing Python requirements..."; \
    find /tmp/packages -name "requirements.txt" -exec pip install --no-cache-dir -r {} \; \
    || echo "Warning: Some pip installations failed"; \
    fi

# Install Python packages (editable installs)
RUN if [ -d "/tmp/packages" ]; then \
    echo "Installing Python packages..."; \
    find /tmp/packages -name "pyproject.toml" -exec dirname {} \; | while read dir; do \
    pip install --no-cache-dir -e "$dir" || echo "Warning: pip install failed for $dir"; \
    done; \
    fi

# Install additional system packages if specified
RUN if [ -n "$INSTALL_PACKAGE_NAMES" ]; then \
    echo "Installing additional packages: $INSTALL_PACKAGE_NAMES"; \
    sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends $INSTALL_PACKAGE_NAMES \
    && sudo rm -rf /var/lib/apt/lists/*; \
    fi

# Cleanup
RUN sudo rm -rf /tmp/packages \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

#------------------------------------------------------------------------------
# BUILD STAGE - Build ROS packages with nested package support
#------------------------------------------------------------------------------
FROM dependency-stage AS build-stage

ARG PACKAGE_NAMES

# Copy source code for specified packages
COPY --parents packages/src /tmp/
RUN mkdir -p /home/ubuntu/workspace/packages/src && \
    if [ -n "${PACKAGE_NAMES}" ]; then \
    echo "Copying specified packages: ${PACKAGE_NAMES}"; \
    for pkg in ${PACKAGE_NAMES}; do \
    # Find all directories matching the package name (handles nested packages)
    find /tmp/packages/src -type d -name "$pkg" | while read pkg_dir; do \
    if [ -f "$pkg_dir/package.xml" ]; then \
    rel_path=$(echo "$pkg_dir" | sed 's|^/tmp/||'); \
    dest_dir="/home/ubuntu/workspace/$rel_path"; \
    mkdir -p "$(dirname "$dest_dir")"; \
    cp -r "$pkg_dir" "$(dirname "$dest_dir")/"; \
    echo "Copied package: $pkg from $rel_path"; \
    fi; \
    done; \
    done; \
    else \
    echo "No packages specified, skipping package copy"; \
    fi \
    && sudo rm -rf /tmp/packages

# Fix ownership and build
RUN sudo chown -R ubuntu:ubuntu /home/ubuntu/workspace \
    && if [ -n "${PACKAGE_NAMES}" ] && [ "$(find /home/ubuntu/workspace/packages/src -name "package.xml" -type f 2>/dev/null | wc -l)" -gt 0 ]; then \
    echo "Building packages..."; \
    cd /home/ubuntu/workspace/packages && \
    bash -c "source /opt/ros/\${ROS_DISTRO}/setup.bash && \
    source /home/ubuntu/tracing/install/setup.bash 2>/dev/null || true && \
    colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release"; \
    else \
    echo "No packages to build, skipping build step"; \
    fi

#------------------------------------------------------------------------------
# RUNTIME STAGE - Production ready image
#------------------------------------------------------------------------------
FROM build-stage AS runtime

USER ubuntu

# Source workspace in bashrc
RUN echo "source /home/ubuntu/workspace/packages/install/setup.bash" >> /home/ubuntu/.bashrc

# Copy and set up entrypoint
COPY dockerfiles/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN sudo chmod +x /usr/local/bin/entrypoint.sh

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/bin/bash"]

#------------------------------------------------------------------------------
# DEVELOPMENT STAGE - Development tools and full source access
#------------------------------------------------------------------------------
FROM build-stage AS development

# Additional development tools
RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
    clangd \
    clang-format \
    clang-tidy \
    gdb \
    valgrind \
    build-essential \
    cmake \
    vim \
    nano \
    && sudo rm -rf /var/lib/apt/lists/*

# Enhanced development aliases
RUN echo 'alias cb="colcon build --symlink-install"' >> /home/ubuntu/.bashrc \
    && echo 'alias cbp="colcon build --symlink-install --packages-select"' >> /home/ubuntu/.bashrc \
    && echo 'alias cbt="colcon test"' >> /home/ubuntu/.bashrc \
    && echo 'alias cbtr="colcon test-result --verbose"' >> /home/ubuntu/.bashrc \
    && echo 'function ll() { ls -alF "$@"; }' >> /home/ubuntu/.bashrc \
    && echo 'function la() { ls -A "$@"; }' >> /home/ubuntu/.bashrc

# Source workspace in bashrc
RUN echo "source /home/ubuntu/workspace/packages/install/setup.bash" >> /home/ubuntu/.bashrc

# Copy and set up entrypoint
COPY dockerfiles/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN sudo chmod +x /usr/local/bin/entrypoint.sh

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/bin/bash"]