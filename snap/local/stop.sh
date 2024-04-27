#!/bin/sh -e

log() {
    local message="$1"
    # Log the message with logger
    logger -t "${SNAP_NAME}" "stop: $message"
}

log "Stop ${SNAP_NAME} services"
snapctl stop --disable ${SNAP_NAME}.udp-daemon 2>&1 || true
snapctl stop --disable ${SNAP_NAME}.shm-daemon 2>&1 || true