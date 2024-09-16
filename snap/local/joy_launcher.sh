#!/bin/bash -e

source $SNAP/usr/bin/utils.sh

# Retrieve the namespace using snapctl
NAMESPACE="$(snapctl get ros.namespace)"

# Initialize the launch arguments
LAUNCH_ARGS="config_filepath:=$SNAP_COMMON/joy_teleop.config.yaml"

# If NAMESPACE is set, append the namespace argument
if [ -n "$NAMESPACE" ]; then
    LAUNCH_ARGS="$LAUNCH_ARGS namespace:=/$NAMESPACE"
fi

# If ROS_DISTRO is jazzy, append the publish_stamped_twist argument
if [ "$ROS_DISTRO" == "jazzy" ]; then
    LAUNCH_ARGS="$LAUNCH_ARGS publish_stamped_twist:=true"
elif [ "$ROS_DISTRO" == "humble" ]; then
    LAUNCH_ARGS="$LAUNCH_ARGS publish_stamped_twist:=false"
fi

# export LD_LIBRARY_PATH="$SNAP/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/pulseaudio:$LD_LIBRARY_PATH"

# export LD_LIBRARY_PATH="$SNAP/usr/lib/aarch64-linux-gnu/pulseaudio/libpulsecommon-16.1.so:$LD_LIBRARY_PATH"

# export LD_LIBRARY_PATH=/snap/${SNAP_NAME}/x1/opt/ros/snap/lib:/snap/${SNAP_NAME}/x1/opt/ros/humble/lib/aarch64-linux-gnu:/snap/${SNAP_NAME}/x1/opt/ros/humble/lib:/snap/${SNAP_NAME}/x1/opt/ros/underlay_ws/usr/lib:/snap/${SNAP_NAME}/x1/opt/ros/underlay_ws/usr/lib/aarch64-linux-gnu:/snap/${SNAP_NAME}/x1/opt/ros/underlay_ws/opt/ros/humble/lib/aarch64-linux-gnu:/snap/${SNAP_NAME}/x1/opt/ros/underlay_ws/opt/ros/humble/lib:/snap/${SNAP_NAME}/x1/usr/lib/aarch64-linux-gnu/pulseaudio:/var/lib/snapd/lib/gl:/var/lib/snapd/lib/gl32:/var/lib/snapd/void:/snap/${SNAP_NAME}/x1/usr/lib:/snap/${SNAP_NAME}/x1/usr/lib/aarch64-linux-gnu
# echo $LD_LIBRARY_PATH

if [ "${LAUNCH_ARGS}" ]; then
  # watch the log with: "journalctl -t husarion-depthai"
  log_and_echo "Running with options: ${LAUNCH_ARGS}"
fi

# Run the launch command with the constructed arguments
# ros2 launch teleop_twist_joy teleop-launch.py $LAUNCH_ARGS
ros2 launch $SNAP/usr/bin/joy.launch.py $LAUNCH_ARGS
# ros2 run teleop_twist_joy teleop_node --ros-args -p publish_stamped_twist:=true
