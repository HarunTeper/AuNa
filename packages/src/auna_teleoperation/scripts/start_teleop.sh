#!/bin/bash

# Frage nach dem Namespace
read -p "Enter namespace (leave empty for none): " namespace
if [ ! -z "$namespace" ]; then
    ns_arg="--ros-args -r __ns:=/${namespace}"
else
    ns_arg=""
fi

# Frage nach dem CMD_VEL Topic
read -p "Enter cmd_vel topic name (leave empty for default cmd_vel): " cmd_vel
if [ ! -z "$cmd_vel" ]; then
    topic_arg="--ros-args -r /cmd_vel:=/${cmd_vel#/}"
else
    topic_arg=""
fi

# Starte die Teleop Node mit den gewählten Parametern
ros2 run teleop_twist_keyboard teleop_twist_keyboard $ns_arg $topic_arg
