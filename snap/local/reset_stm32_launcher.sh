#!/bin/bash -e

source $SNAP/usr/bin/utils.sh

# Check if the script is being run as root
if [ "$(id -u)" -ne 0 ]; then
    log_and_echo "Error: This script must be run as root."
    exit 1
fi

export PYTHONPATH=$SNAP/usr/lib/python3/dist-packages

$SNAP/usr/bin/reset_stm32.py