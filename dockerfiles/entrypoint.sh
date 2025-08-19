#!/bin/bash
set -e

# Function to print info messages
print_info() {
    echo "[INFO] $1"
}

# Function to print warning messages  
print_warning() {
    echo "[WARNING] $1" >&2
}

# Function to print error messages
print_error() {
    echo "[ERROR] $1" >&2
}

# Source ROS environment using environment variable
if [ -n "${ROS_DISTRO}" ]; then
    if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
        source /opt/ros/${ROS_DISTRO}/setup.bash
        print_info "Sourced ROS ${ROS_DISTRO} environment"
    else
        print_error "ROS ${ROS_DISTRO} setup.bash not found"
        exit 1
    fi
else
    # Fallback to humble if ROS_DISTRO not set
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        print_info "Sourced ROS Humble environment (default)"
    else
        print_error "No ROS environment found"
        exit 1
    fi
fi

# Set RMW implementation if specified
if [ -n "${RMW_IMPLEMENTATION}" ]; then
    export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
    print_info "Set RMW implementation to: ${RMW_IMPLEMENTATION}"
fi

# Set ROS_DOMAIN_ID if specified
if [ -n "${ROS_DOMAIN_ID}" ]; then
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
    print_info "Set ROS_DOMAIN_ID to: ${ROS_DOMAIN_ID}"
fi

# Source Gazebo setup if it exists
if [ -f "/usr/share/gazebo/setup.bash" ]; then
    source /usr/share/gazebo/setup.bash
    print_info "Sourced Gazebo environment"
fi

# Source tracing environment if it exists
if [ -f "/home/ubuntu/tracing/install/setup.bash" ]; then
    source /home/ubuntu/tracing/install/setup.bash
    print_info "Sourced tracing environment"
else
    print_warning "Tracing environment not found"
fi

# Source base workspace if it exists
if [ -f "/home/ubuntu/workspace/packages/install/setup.bash" ]; then
    source /home/ubuntu/workspace/packages/install/setup.bash
    print_info "Sourced workspace environment"
    
    # Set Gazebo model path if workspace contains gazebo models
    if [ -d "/home/ubuntu/workspace/packages/install" ]; then
        GAZEBO_MODELS=$(find /home/ubuntu/workspace/packages/install -name "models" -type d 2>/dev/null | head -1)
        if [ -n "${GAZEBO_MODELS}" ]; then
            export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${GAZEBO_MODELS}
            print_info "Updated GAZEBO_MODEL_PATH"
        fi
    fi
else
    print_warning "Workspace environment not found - this is normal for fresh containers"
fi

# Handle different execution contexts
if [ $# -eq 0 ]; then
    print_info "No command specified, starting interactive bash session"
    exec /bin/bash
else
    print_info "Executing command: $*"
    # Use exec to ensure proper signal handling[4]
    exec "$@"
fi
