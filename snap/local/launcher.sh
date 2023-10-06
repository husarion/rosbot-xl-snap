#!/usr/bin/bash

# Iterate over the snap parameters and retrieve their value.
# If a value is set, it is forwarded to the launch file.
OPTIONS="mecanum include-camera-mount camera-model lidar-model"

for OPTION in ${OPTIONS}; do
  VALUE="$(snapctl get ${OPTION})"
  if [ -n "${VALUE}" ]; then
    LAUNCH_OPTIONS+="${OPTION}:=${VALUE} "
  fi
done

# Replace '-' with '_'
LAUNCH_OPTIONS=$(echo ${LAUNCH_OPTIONS} | tr - _)

if [ ${LAUNCH_OPTIONS} ]; then
  logger -t ${SNAP_NAME} "Running with options: ${LAUNCH_OPTIONS}"
fi

ros2 launch rosbot_xl_bringup bringup.launch.py ${LAUNCH_OPTIONS}
