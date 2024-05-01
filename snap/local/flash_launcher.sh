#!/usr/bin/bash

# Define a function to log and echo messages
log_and_echo() {
    local message="$1"
    # Log the message with logger
    logger -t "${SNAP_NAME}" "${SNAP_NAME}.flash: $message"
    # Echo the message to standard error
    echo >&2 "$message"
}

SERIAL_PORT="$(snapctl get serial-port)"

# watch the log with: "journalctl -t rosbot-xl"
log_and_echo "flashing with serial-port=${SERIAL_PORT}"

ros2 run rosbot_xl_utils flash_firmware --port $SERIAL_PORT

snapctl restart ${SNAP_NAME}.daemon

