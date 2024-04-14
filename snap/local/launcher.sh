#!/usr/bin/bash

# Iterate over the snap parameters and retrieve their value.
# If a value is set, it is forwarded to the launch file.
OPTIONS="mecanum include-camera-mount camera-model lidar-model"

for OPTION in ${OPTIONS}; do
  VALUE="$(snapctl get ${OPTION})"
  if [ -n "${VALUE}" ]; then
    LAUNCH_OPTIONS+="${OPTION}:=${VALUE} "
  fi
done

# Replace '-' with '_'
LAUNCH_OPTIONS=$(echo ${LAUNCH_OPTIONS} | tr - _)

if [ "${LAUNCH_OPTIONS}" ]; then
  # watch the log with: "journalctl -t rosbot-xl"
  logger -t ${SNAP_NAME} "Running with options: ${LAUNCH_OPTIONS}"
fi

FASTDDS_FILE=$(snapctl get fastdds-default-profiles-file)

if [ ! -f "${SNAP_COMMON}/${FASTDDS_FILE}" ]; then
  # eg. /var/snap/rosbot-xl/common/fastdds.xml
  logger -t ${SNAP_NAME} "${SNAP_COMMON}/${FASTDDS_FILE} does not exist."
fi

if [ -n "${FASTDDS_FILE}" ] && [ -f "${SNAP_COMMON}/${FASTDDS_FILE}" ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE=${SNAP_COMMON}/${FASTDDS_FILE}
  logger -t ${SNAP_NAME} "Using FASTRTPS profile: ${FASTRTPS_DEFAULT_PROFILES_FILE}"
  logger -t ${SNAP_NAME} "$(cat $FASTRTPS_DEFAULT_PROFILES_FILE)"
else
  TRANSPORT="$(snapctl get transport)"
  # watch the log with: "journalctl -t rosbot-xl"
  logger -t ${SNAP_NAME} "transport: ${TRANSPORT}"

  case "$TRANSPORT" in
  shm)
    export FASTRTPS_DEFAULT_PROFILES_FILE=$SNAP/usr/share/rosbot-xl/config/shm-only.xml
    logger -t ${SNAP_NAME} "$(cat $FASTRTPS_DEFAULT_PROFILES_FILE)"
    ;;
  udp)
    export FASTRTPS_DEFAULT_PROFILES_FILE=$SNAP/usr/share/rosbot-xl/config/udp-only.xml
    logger -t ${SNAP_NAME} "$(cat $FASTRTPS_DEFAULT_PROFILES_FILE)"
    ;;
  builtin)
    logger -t ${SNAP_NAME} "using builtin transport"
    ;;
  *)
    snapctl stop ${SNAP_NAME}.udp-daemon --disable
    snapctl stop ${SNAP_NAME}.shm-daemon --disable
    logger -t ${SNAP_NAME} "unknown transport: $TRANSPORT. Possible values: shm, udp, builtin"
    exit 1
    ;;
  esac
fi

export ROS_LOCALHOST_ONLY="$(snapctl get ros-localhost-only)"
logger -t ${SNAP_NAME} "ROS_LOCAHOST_ONLY=${ROS_LOCALHOST_ONLY}"

ros2 launch rosbot_xl_bringup combined.launch.py ${LAUNCH_OPTIONS}
