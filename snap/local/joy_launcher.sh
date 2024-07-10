#!/bin/bash -e

ros2 run joy joy_node --ros-args --params-file $SNAP_COMMON/joy_params.yaml

# # Retrieve the namespace using snapctl
# NAMESPACE="$(snapctl get ros.namespace)"

# # Check if NAMESPACE is not set or is empty
# if [ -z "$NAMESPACE" ]; then
#     # No namespace is set, run the launch command without the namespace argument
#     ros2 launch teleop_twist_joy teleop-launch.py \
#         config_filepath:=$SNAP_COMMON/joy_teleop.config.yaml
#     # ros2 launch teleop_twist_joy teleop-launch.py
# else
#     # Namespace is set, include it in the launch command
#     ros2 launch teleop_twist_joy teleop-launch.py \
#         config_filepath:=$SNAP_COMMON/joy_teleop.config.yaml --ros-args -r __ns:=/$NAMESPACE
#     # ros2 launch teleop_twist_joy teleop-launch.py --ros-args -r __ns:=/$NAMESPACE
# fi
