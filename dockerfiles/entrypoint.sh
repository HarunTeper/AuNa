#!/bin/bash
set -e

# Function to print info messages
print_info() {
    echo "[INFO] $1"
}

# Source ROS environment
source /opt/ros/humble/setup.bash

# Source Gazebo setup if it exists
if [ -f "/usr/share/gazebo/setup.bash" ]; then
    source /usr/share/gazebo/setup.bash
    print_info "Sourced Gazebo environment"
fi

# Source workspace if it exists
if [ -f "/home/ubuntu/workspace/packages/install/setup.bash" ]; then
    source /home/ubuntu/workspace/packages/install/setup.bash
    print_info "Sourced workspace environment"
fi

# Execute the command
print_info "Launching: $*"
exec "$@"
