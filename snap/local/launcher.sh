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

if [ ${LAUNCH_OPTIONS} ]; then
  # watch the log with: "journalctl -t rosbot-xl"
  logger -t ${SNAP_NAME} "Running with options: ${LAUNCH_OPTIONS}"
fi

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
    *)
        # logger -t ${SNAP_NAME} "ROS_LOCAHOST_ONLY=${ROS_LOCALHOST_ONLY}"
        ;;
esac

export ROS_LOCALHOST_ONLY="$(snapctl get ros-localhost-only)"
logger -t ${SNAP_NAME} "ROS_LOCAHOST_ONLY=${ROS_LOCALHOST_ONLY}"
# logger -t ${SNAP_NAME} "$(printenv)"

ros2 launch rosbot_xl_bringup combined.launch.py ${LAUNCH_OPTIONS}
