#!/bin/bash -e

NAMESPACE="$(snapctl get driver.namespace)"

# Check if the namespace is set and not empty
if [ -n "$NAMESPACE" ]; then
  ros2 launch $SNAP/usr/bin/bridge_launch.py namespace:=${NAMESPACE}
else
  ros2 launch $SNAP/usr/bin/bridge_launch.py
fi
