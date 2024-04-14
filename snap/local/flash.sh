#!/usr/bin/bash

SERIAL_PORT="$(snapctl get serial-port)"

# watch the log with: "journalctl -t rosbot-xl"
logger -t ${SNAP_NAME} "flashing with serial-port=${SERIAL_PORT}"

ros2 run rosbot_xl_utils flash_firmware --port $SERIAL_PORT

# Define the names of the daemons using SNAP_NAME
udp_daemon="${SNAP_NAME}.udp-daemon"
shm_daemon="${SNAP_NAME}.shm-daemon"

# Stop the UDP daemon if it is running
if snapctl services $udp_daemon | grep -qw active; then
    snapctl restart $udp_daemon
    logger -t "${SNAP_NAME}" "Restarted $udp_daemon"
fi

# Check if the SHM daemon is already running
if snapctl services $shm_daemon | grep -qw active; then
    snapctl restart $shm_daemon
    logger -t "${SNAP_NAME}" "Restarted $shm_daemon"
fi
