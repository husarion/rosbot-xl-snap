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
# # Define the names of the daemons using SNAP_NAME
# udp_daemon="${SNAP_NAME}.udp-daemon"
# shm_daemon="${SNAP_NAME}.shm-daemon"

# # Stop the UDP daemon if it is running
# if snapctl services $udp_daemon | grep -qw active; then
#     snapctl restart $udp_daemon
#     log_and_echo "Restarted $udp_daemon"
# fi

# # Check if the SHM daemon is already running
# if snapctl services $shm_daemon | grep -qw active; then
#     snapctl restart $shm_daemon
#     log_and_echo "Restarted $shm_daemon"
# fi
