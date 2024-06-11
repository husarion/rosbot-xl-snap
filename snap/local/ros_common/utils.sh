#!/bin/bash -e

# Define a function to log and echo messages
log_and_echo() {
    local message="$1"
    local script_name=$(basename "$0")
    # Log the message with logger
    logger -t "${SNAP_NAME}" "${script_name}: $message"
    # Echo the message to standard error
    echo >&2 "$message"
}

log() {
    local message="$1"
    local script_name=$(basename "$0")
    # Log the message with logger
    logger -t "${SNAP_NAME}" "${script_name}: $message"
}