#!/usr/bin/bash

# Iterate over the snap parameters and retrieve their value.
# If a value is set, it is forwarded to the launch file.

SERIAL_PORT="$(snapctl get serial-port)"

logger -t ${SNAP_NAME} "flashing with serial-port=${SERIAL_PORT}"

ros2 run rosbot_xl_utils flash_firmware --port /dev/ttyUSBDB
