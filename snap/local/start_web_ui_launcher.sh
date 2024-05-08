#!/bin/sh -e

log() {
    local message="$1"
    # Log the message with logger
    logger -t "${SNAP_NAME}" "stop: $message"
}

log "Start Web UI"
snapctl start --enable ${SNAP_NAME}.web-ui 2>&1 || true
snapctl start --enable ${SNAP_NAME}.web-ws 2>&1 || true