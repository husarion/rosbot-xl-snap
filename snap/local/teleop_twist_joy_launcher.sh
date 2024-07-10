#!/bin/bash -e

LAUNCH_OPTIONS=""

# Retrieve the namespace using snapctl
NAMESPACE="$(snapctl get ros.namespace)"

# Check if NAMESPACE is not set or is empty
if [ -n "$NAMESPACE" ]; then
    LAUNCH_OPTIONS+="namespace:=${NAMESPACE} "
fi

ros2 launch $SNAP/usr/bin/teleop_twist_joy_launch.py ${LAUNCH_OPTIONS}
