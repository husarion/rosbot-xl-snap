#!/bin/bash -e

source $SNAP/usr/bin/utils.sh

log "Restart ${SNAP_NAME}.daemon service"
snapctl restart --enable ${SNAP_NAME}.daemon 2>&1 || true

log "Restart ${SNAP_NAME}.joy service"
snapctl restart --enable ${SNAP_NAME}.joy 2>&1 || true