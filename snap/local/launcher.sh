#!/bin/bash -e

source $SNAP/usr/bin/utils.sh

# Iterate over the snap parameters and retrieve their value.
# If a value is set, it is forwarded to the launch file.
OPTIONS="namespace mecanum include-camera-mount camera-model lidar-model"
LAUNCH_OPTIONS=""

for OPTION in ${OPTIONS}; do
  VALUE="$(snapctl get driver.${OPTION})"
  if [ -n "${VALUE}" ]; then
    OPTION_WITH_UNDERSCORE=$(echo ${OPTION} | tr - _)
    LAUNCH_OPTIONS+="${OPTION_WITH_UNDERSCORE}:=${VALUE} "
  fi
done

if [ "${LAUNCH_OPTIONS}" ]; then
  # watch the log with: "journalctl -t rosbot-xl"
  log "Running with options: ${LAUNCH_OPTIONS}"
fi

# ros2 launch rosbot_xl_bringup combined.launch.py ${LAUNCH_OPTIONS}
ros2 launch $SNAP/usr/bin/bringup/rosbot.launch.py ${LAUNCH_OPTIONS}