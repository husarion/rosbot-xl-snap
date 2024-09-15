#!/bin/bash -e

source $SNAP/usr/bin/utils.sh

log "Stop ${SNAP_NAME}.daemon service"
snapctl stop --disable ${SNAP_NAME}.daemon 2>&1 || true

log "Stop ${SNAP_NAME}.joy service"
snapctl stop --disable ${SNAP_NAME}.joy 2>&1 || true