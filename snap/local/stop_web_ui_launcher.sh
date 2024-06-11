#!/bin/bash -e

source $SNAP/usr/bin/utils.sh

log "Stop Web UI"
snapctl stop --disable ${SNAP_NAME}.web-ui 2>&1 || true
snapctl stop --disable ${SNAP_NAME}.web-ws 2>&1 || true
