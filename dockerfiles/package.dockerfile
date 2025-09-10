# syntax=docker/dockerfile:1-labs
#------------------------------------------------------------------------------
# STAGE 1: Dependency Resolution and Caching
#------------------------------------------------------------------------------
FROM base:latest AS dependency-stage

USER root

# Copy dependency files
COPY --parents packages/src/**/package.xml /tmp/package_xmls/
COPY --parents packages/**/requirements.txt /tmp/requirements/
COPY --parents packages/**/*.toml /tmp/toml/

# Fix ownership after COPY
RUN chown -R ubuntu:ubuntu /tmp/package_xmls /tmp/requirements /tmp/toml

USER ubuntu

# Update rosdep database
RUN sudo apt-get update && rosdep update

# Install system dependencies with error handling
ARG PACKAGE_NAMES
RUN echo "Resolving dependencies for: ${PACKAGE_NAMES}" && \
    if [ -n "${PACKAGE_NAMES}" ]; then \
    for pkg in ${PACKAGE_NAMES}; do \
    if [ -d "/tmp/package_xmls/packages/src/$pkg" ]; then \
    echo "Installing dependencies for $pkg"; \
    rosdep install --from-paths /tmp/package_xmls/packages/src/$pkg --ignore-src -r -y \
    || echo "Warning: rosdep failed for $pkg"; \
    else \
    echo "Warning: Package directory not found for $pkg"; \
    fi; \
    done; \
    fi

# Install Python dependencies with error handling
RUN if [ -n "${PACKAGE_NAMES}" ]; then \
    for pkg in ${PACKAGE_NAMES}; do \
    if [ -f "/tmp/requirements/packages/src/$pkg/requirements.txt" ]; then \
    echo "Installing Python requirements for $pkg"; \
    pip install --no-cache-dir -r /tmp/requirements/packages/src/$pkg/requirements.txt || echo "Warning: pip install failed for $pkg"; \
    fi; \
    if [ -f "/tmp/toml/packages/src/$pkg/pyproject.toml" ]; then \
    echo "Installing Python package $pkg"; \
    pip install --no-cache-dir -e /tmp/toml/packages/src/$pkg || echo "Warning: pip install failed for $pkg"; \
    fi; \
    done; \
    fi

RUN sudo rm -rf /tmp/package_xmls /tmp/requirements /tmp/toml \
    && sudo apt-get clean \
    && sudo rm -rf /var/lib/apt/lists/*

#------------------------------------------------------------------------------
# STAGE 2: Build Stage
#------------------------------------------------------------------------------
FROM dependency-stage AS build-stage

# Install additional packages if specified
ARG INSTALL_PACKAGE_NAMES
RUN if [ -n "$INSTALL_PACKAGE_NAMES" ]; then \
    echo "Installing additional packages: $INSTALL_PACKAGE_NAMES"; \
    sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends $INSTALL_PACKAGE_NAMES \
    && sudo rm -rf /var/lib/apt/lists/*; \
    fi

# Copy source code
COPY packages/src /tmp/src_temp/
RUN mkdir -p /home/ubuntu/workspace/packages/src && \
    if [ -n "${PACKAGE_NAMES}" ]; then \
    for pkg in ${PACKAGE_NAMES}; do \
    if [ -d "/tmp/src_temp/$pkg" ]; then \
    cp -r /tmp/src_temp/$pkg /home/ubuntu/workspace/packages/src/; \
    echo "Copied package: $pkg"; \
    else \
    echo "Warning: Package $pkg not found in source"; \
    fi; \
    done; \
    fi \
    && sudo rm -rf /tmp/src_temp

# Fix ownership and build
RUN sudo chown -R ubuntu:ubuntu /home/ubuntu/workspace \
    && cd /home/ubuntu/workspace/packages \
    && bash -c "source /opt/ros/\${ROS_DISTRO}/setup.bash && \
    source /home/ubuntu/tracing/install/setup.bash 2>/dev/null || true && \
    if [ -n '${PACKAGE_NAMES}' ]; then \
    colcon build --symlink-install --packages-select ${PACKAGE_NAMES} --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release; \
    else \
    colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release; \
    fi"

#------------------------------------------------------------------------------
# STAGE 3: Runtime Stage (Production)
#------------------------------------------------------------------------------
FROM base:latest AS runtime

# Copy built artifacts from build stage
COPY --from=build-stage --chown=ubuntu:ubuntu /home/ubuntu/workspace/packages/install /home/ubuntu/workspace/packages/install

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
# STAGE 4: Development Stage (with source code access)
#------------------------------------------------------------------------------
FROM build-stage AS development

# Additional development tools with cleanup
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

# Enhanced development aliases and configuration
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
