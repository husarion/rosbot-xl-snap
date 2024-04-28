#!/usr/bin/bash

# Retrieve the namespace using snapctl
NAMESPACE="$(snapctl get namespace)"

# Check if NAMESPACE is not set or is empty
if [ -z "$NAMESPACE" ]; then
    # No namespace is set, run the launch command without the namespace argument
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
else
    # Namespace is set, include it in the launch command
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/$NAMESPACE
fi
