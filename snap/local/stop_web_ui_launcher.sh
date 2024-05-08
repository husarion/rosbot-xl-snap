#!/bin/sh -e

log() {
    local message="$1"
    # Log the message with logger
    logger -t "${SNAP_NAME}" "stop: $message"
}

log "Stop Web UI"
snapctl stop --disable ${SNAP_NAME}.web-ui 2>&1 || true
snapctl stop --disable ${SNAP_NAME}.web-ws 2>&1 || true
