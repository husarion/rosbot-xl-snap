#!/bin/sh -e

logger -t ${SNAP_NAME} "Stop rosbot-xl services"
snapctl stop --disable ${SNAP_NAME}.udp-daemon 2>&1 || true
snapctl stop --disable ${SNAP_NAME}.shm-daemon 2>&1 || true
