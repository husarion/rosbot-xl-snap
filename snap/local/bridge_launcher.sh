#!/bin/bash -e

# ros2 run foxglove_bridge foxglove_bridge --ros-args \
# -p port:=8766 \
# -p address:=127.0.0.1 \
# -p capabilities:=[clientPublish,connectionGraph,assets]

# ros2 run teleop_twist_joy teleop_node --ros-args --params-file ${SNAP_COMMON}/teleop_twist_joy_f710_params.yaml

# ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8766 address:=127.0.0.1 capabilities:=[clientPublish,connectionGraph,assets]
ros2 launch $SNAP/usr/bin/bridge_launch.py
