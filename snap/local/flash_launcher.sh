#!/usr/bin/bash

# Define a function to log and echo messages
log_and_echo() {
    local message="$1"
    # Log the message with logger
    logger -t "${SNAP_NAME}" "${SNAP_NAME}.flash: $message"
    # Echo the message to standard error
    echo >&2 "$message"
}

# Check if the script is being run as root
if [ "$(id -u)" -ne 0 ]; then
    log_and_echo "Error: This script must be run as root."
    exit 1
fi

# Check if the daemon is running and stop it if it is
if snapctl services "${SNAP_NAME}.daemon" | grep -qw active; then
    log_and_echo "Stopping ${SNAP_NAME}.daemon..."
    snapctl stop "${SNAP_NAME}.daemon"
    DAEMON_WAS_RUNNING=true
else
    DAEMON_WAS_RUNNING=false
fi


SERIAL_PORT="$(snapctl get serial-port)"

# watch the log with: "journalctl -t rosbot-xl"
log_and_echo "flashing with serial-port=${SERIAL_PORT}"

ros2 run rosbot_xl_utils flash_firmware --port $SERIAL_PORT

# Restart the daemon if it was running before
if [ "$DAEMON_WAS_RUNNING" = true ]; then
    log_and_echo "Restarting ${SNAP_NAME}.daemon..."
    snapctl start "${SNAP_NAME}.daemon"
fi

