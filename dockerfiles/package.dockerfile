# Build from the base dockerfile
FROM base:latest

USER root

# Install ROS packages and dependencies
ARG PACKAGE_NAMES
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
    rm -rf /tmp/src_temp

# Fix ownership after COPY (COPY creates files owned by root)
RUN chown -R ubuntu:ubuntu /home/ubuntu/workspace

# Copy and set up entrypoint script
COPY dockerfiles/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

RUN apt update

USER ubuntu

# Update rosdep database
RUN rosdep update

ARG PIP_BREAK_SYSTEM_PACKAGES
ENV PIP_BREAK_SYSTEM_PACKAGES=${PIP_BREAK_SYSTEM_PACKAGES}

# Install dependencies for all packages
RUN cd /home/ubuntu/workspace/packages && \
    rosdep install --from-paths src --ignore-src -r -y

# Install Python packages from requirements files
RUN find /home/ubuntu/workspace/packages/src -name "requirements.txt" -exec pip3 install --break-system-packages -r {} \;

# Install Python packages from pyproject.toml files  
RUN find /home/ubuntu/workspace/packages/src -name "pyproject.toml" -exec sh -c 'pip3 install --break-system-packages -e "$(dirname "{}")"' \;

# Clean up package lists at the end
RUN sudo rm -rf /var/lib/apt/lists/*

RUN cd /home/ubuntu/workspace/packages && \
    source /opt/ros/humble/setup.bash && \    
    colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace in bashrc
RUN echo "source /home/ubuntu/workspace/packages/install/setup.bash" >> /home/ubuntu/.bashrc

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/bin/bash"]


# ENTRYPOINT ["/entrypoint.sh"]
# CMD ["bash", "-c", "ros2 launch cmd_multiplexer cmd_multiplexer.launch.py & ros2 launch physical_car physical_car.launch.py"]