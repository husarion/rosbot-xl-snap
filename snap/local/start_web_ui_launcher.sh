#!/bin/bash -e

# Define a function to log and echo messages
source $SNAP/usr/bin/utils.sh

# Check if the script is being run as root
if [ "$(id -u)" -ne 0 ]; then
    log_and_echo "Error: This script must be run as root."
    exit 1
fi

snapctl start --enable ${SNAP_NAME}.web-ui 2>&1 || true
snapctl start --enable ${SNAP_NAME}.web-ws 2>&1 || true