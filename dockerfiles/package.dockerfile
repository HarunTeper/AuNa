# syntax=docker/dockerfile:1-labs
# Build from the base dockerfile
FROM base:latest

USER root

# Copy the package.xml, requirements.txt, and pyproject.toml files for dependency resolution to maximize cache efficiency
COPY --parents packages/src/**/package.xml /tmp/package_xmls/
COPY --parents packages/**/requirements.txt /tmp/requirements/
COPY --parents packages/**/*.toml /tmp/toml/

# Fix ownership after COPY (COPY creates files owned by root)
RUN chown -R ubuntu:ubuntu /tmp/package_xmls
RUN chown -R ubuntu:ubuntu /tmp/requirements
RUN chown -R ubuntu:ubuntu /tmp/toml

# Copy and set up entrypoint script
COPY dockerfiles/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

RUN apt update

USER ubuntu

# Update rosdep database
RUN rosdep update

ARG PIP_BREAK_SYSTEM_PACKAGES
ENV PIP_BREAK_SYSTEM_PACKAGES=${PIP_BREAK_SYSTEM_PACKAGES}

ARG PACKAGE_NAMES
RUN echo "Building packages: $PACKAGE_NAMES"
# Install dependencies only for packages in PACKAGE_NAMES
RUN for pkg in $PACKAGE_NAMES; do \
    rosdep install --from-paths /tmp/package_xmls/packages/src/$pkg --ignore-src -r -y || echo "Warning: rosdep failed for $pkg"; \
    done && \
    rm -rf /tmp/package_xmls

# Install Python packages from requirements.txt only for PACKAGE_NAMES
RUN for pkg in $PACKAGE_NAMES; do \
    if [ -f "/tmp/requirements/packages/src/$pkg/requirements.txt" ]; then \
    pip install -r /tmp/requirements/packages/src/$pkg/requirements.txt; \
    fi; \
    done && \
    rm -rf /tmp/requirements

# Install Python packages from pyproject.toml only for PACKAGE_NAMES
RUN for pkg in $PACKAGE_NAMES; do \
    if [ -f "/tmp/toml/packages/src/$pkg/pyproject.toml" ]; then \
    pip install -e /tmp/toml/packages/src/$pkg; \
    fi; \
    done && \
    rm -rf /tmp/toml

ARG INSTALL_PACKAGE_NAMES
RUN echo "Installing additional packages: $INSTALL_PACKAGE_NAMES"
RUN if [ -n "$INSTALL_PACKAGE_NAMES" ]; then \
    sudo apt-get update && \
    sudo apt-get install -y --no-install-recommends $INSTALL_PACKAGE_NAMES; \
    fi

# Clean up package lists at the end
RUN sudo rm -rf /var/lib/apt/lists/*

# Install ROS packages and dependencies
RUN echo "Building packages: $PACKAGE_NAMES"
# Copy all specified packages
RUN mkdir -p /home/ubuntu/workspace/packages/src
COPY packages/src /tmp/src_temp/
RUN for pkg in $PACKAGE_NAMES; do \
    if [ -d "/tmp/src_temp/$pkg" ]; then \
    cp -r /tmp/src_temp/$pkg /home/ubuntu/workspace/packages/src/; \
    echo "Copied package: $pkg"; \
    else \
    echo "Warning: Package $pkg not found"; \
    fi; \
    done && \
    sudo rm -rf /tmp/src_temp

# Fix ownership after COPY (COPY creates files owned by root)
RUN sudo chown -R ubuntu:ubuntu /home/ubuntu/workspace

RUN cd /home/ubuntu/workspace/packages && \
    source /opt/ros/humble/setup.bash && \    
    colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace in bashrc
RUN echo "source /home/ubuntu/workspace/packages/install/setup.bash" >> /home/ubuntu/.bashrc

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/bin/bash"]