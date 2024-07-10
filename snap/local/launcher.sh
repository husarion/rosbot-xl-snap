#!/bin/bash -e

source $SNAP/usr/bin/utils.sh

LAUNCH_OPTIONS=""

# Retrieve the namespace using snapctl
NAMESPACE="$(snapctl get ros.namespace)"

# Check if NAMESPACE is not set or is empty
if [ -n "$NAMESPACE" ]; then
    LAUNCH_OPTIONS+="namespace:=${NAMESPACE} "
fi

# Iterate over the snap parameters and retrieve their value.
# If a value is set, it is forwarded to the launch file.
OPTIONS="mecanum include-camera-mount camera-model lidar-model"

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

OPT="configuration"
VALUE="$(snapctl get ${OPT})"
if [ "${VALUE}" == "basic" ]; then
  ros2 launch rosbot_xl_bringup bringup.launch.py ${LAUNCH_OPTIONS}
elif [ "${VALUE}" == "manipulation" ]; then
  # Check if SERIAL_PORT is set to auto or specified
  SERIAL_PORT=$(find_ttyUSB driver.manipulator-serial-port "0403" "6014")
  if [ $? -ne 0 ]; then
    log_and_echo "Failed to find the serial port."
    exit 1
  else
    log_and_echo "Found serial port: $SERIAL_PORT"
  fi

  # Check if the specified serial port exists
  if [ ! -e "$SERIAL_PORT" ]; then
    log_and_echo "Specified serial port $SERIAL_PORT does not exist."
    exit 1
  else
    log_and_echo "Specified serial port exists: $SERIAL_PORT"
  fi

  ros2 launch rosbot_xl_manipulation_bringup bringup.launch.py \
    ${LAUNCH_OPTIONS} \
    manipulator_usb_port:=${SERIAL_PORT} \
    manipulator_baud_rate:=1000000 \
    joy_servo_params_file:=${SNAP_COMMON}/joy_servo.yaml \
    antenna_rotation_angle:=-1.57
fi
