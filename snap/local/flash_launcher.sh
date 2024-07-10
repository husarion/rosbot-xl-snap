#!/bin/bash -e

# Define a function to log and echo messages
source $SNAP/usr/bin/utils.sh

# Check if the script is being run as root
if [ "$(id -u)" -ne 0 ]; then
    log_and_echo "Error: This script must be run as root."
    exit 1
fi

# Check if the daemon is running and stop it if it is
if snapctl services "${SNAP_NAME}.daemon" | grep -qw enabled; then
    log_and_echo "Stopping ${SNAP_NAME}.daemon..."
    snapctl stop "${SNAP_NAME}.daemon"
    DAEMON_WAS_RUNNING=true
else
    DAEMON_WAS_RUNNING=false
fi

# Check if SERIAL_PORT is set to auto or specified
SERIAL_PORT=$(find_ttyUSB driver.db-serial-port "0403" "6015")
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

ros2 run rosbot_xl_utils flash_firmware --port $SERIAL_PORT

# Restart the daemon if it was running before
if [ "$DAEMON_WAS_RUNNING" = true ]; then
    log_and_echo "Restarting ${SNAP_NAME}.daemon..."
    snapctl start "${SNAP_NAME}.daemon"
fi

