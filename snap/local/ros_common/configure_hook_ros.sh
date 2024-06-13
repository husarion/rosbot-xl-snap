#!/bin/bash -e

# The configure hook is called every time one the following actions happen:
# - initial snap installation
# - snap refresh
# - whenever the user runs snap set|unset to change a configuration option

# Define a function to log and echo messages
source $SNAP/usr/bin/utils.sh

# Make sure ROS_LOCALHOST_ONLY is valid
OPT="ros-localhost-only"
ROS_LOCALHOST_ONLY="$(snapctl get ${OPT})"
if [ -n "${ROS_LOCALHOST_ONLY}" ]; then
  case "${ROS_LOCALHOST_ONLY}" in
  1) ;;
  0) ;;
  *)
    log_and_echo "'${ROS_LOCALHOST_ONLY}' is not a supported value for '${OPT}'. Possible values are 0 or 1."
    exit 1
    ;;
  esac
fi

# Make sure ROS_DOMAIN_ID is valid
OPT="ros-domain-id"
ROS_DOMAIN_ID="$(snapctl get ${OPT})"

if ! is_integer "${ROS_DOMAIN_ID}" || [ "${ROS_DOMAIN_ID}" -lt 0 ] || [ "${ROS_DOMAIN_ID}" -gt 232 ]; then
  log_and_echo "'${ROS_DOMAIN_ID}' is not a supported value for '${OPT}'. Possible values are integers between 0 and 232."
  exit 1
fi

# Get the transport setting using snapctl
OPT="transport"
TRANSPORT_SETTING="$(snapctl get ${OPT})"

# Only exit with status 1 if conditions are not met
if [ "$TRANSPORT_SETTING" != "builtin" ] && [ ! -f "${SNAP_COMMON}/${TRANSPORT_SETTING}.xml" ]; then
  log_and_echo "'${SNAP_COMMON}/${TRANSPORT_SETTING}.xml' does not exist."
  exit 1
fi

if [ "$TRANSPORT_SETTING" = "builtin" ] || [ "$TRANSPORT_SETTING" = "shm" ]; then
  if ! snapctl is-connected shm-plug; then
    log_and_echo "to use 'builtin' and 'shm' tranport shm-plug need to be connected, please run:"
    log_and_echo "sudo snap connect ${SNAP_NAME}:shm-plug ${SNAP_NAME}:shm-slot"
    exit 1
  fi
fi

# Make sure ros-humble-ros-base is connected
ROS_PLUG="ros-humble-ros-base"

if ! snapctl is-connected ${ROS_PLUG}; then
    log_and_echo "Plug '${ROS_PLUG}' isn't connected. Please run:"
    log_and_echo "snap connect ${SNAP_NAME}:${ROS_PLUG} ${ROS_PLUG}:${ROS_PLUG}"
    exit 1
fi

# Create the ${SNAP_COMMON}/ros.env file and export variables (for bash session running ROS2)
ROS_ENV_FILE="${SNAP_COMMON}/ros.env"

# Create the ${SNAP_COMMON}/ros.env file and export variables (for bash session running ROS2)
ROS_SNAP_ARGS="${SNAP_COMMON}/ros_snap_args"

echo "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" > "${ROS_ENV_FILE}"
echo "export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}" >> "${ROS_ENV_FILE}"

NAMESPACE=$(snapctl get driver.namespace)

# Check if the namespace is set and not empty
if [ -n "$NAMESPACE" ]; then
  echo "ros-domain-id=${ROS_DOMAIN_ID} ros-localhost-only=${ROS_LOCALHOST_ONLY} transport=${TRANSPORT_SETTING} driver.namespace=${NAMESPACE}" > "${ROS_SNAP_ARGS}"
  echo "export ROS_NAMESPACE=${NAMESPACE}" >> "${ROS_ENV_FILE}"
else
  echo "ros-domain-id=${ROS_DOMAIN_ID} ros-localhost-only=${ROS_LOCALHOST_ONLY} transport=${TRANSPORT_SETTING} driver.namespace!" > "${ROS_SNAP_ARGS}"
  echo "unset ROS_NAMESPACE" >> "${ROS_ENV_FILE}"
fi

if [ "$TRANSPORT_SETTING" != "builtin" ]; then
  echo "export FASTRTPS_DEFAULT_PROFILES_FILE=${SNAP_COMMON}/${TRANSPORT_SETTING}.xml" >> "${ROS_ENV_FILE}"
fi

# Define the path for the manage_ros_env.sh script
MANAGE_SCRIPT="${SNAP_COMMON}/manage_ros_env.sh"

# Create the manage_ros_env.sh script in ${SNAP_COMMON}
cat << EOF > "${MANAGE_SCRIPT}"
#!/bin/bash

ROS_ENV_FILE="${SNAP_COMMON}/ros.env"
SOURCE_LINE="source \${ROS_ENV_FILE}"

add_source_to_bashrc() {
  if ! grep -Fxq "\$SOURCE_LINE" ~/.bashrc; then
    echo "\$SOURCE_LINE" >> ~/.bashrc
    echo "Added '\$SOURCE_LINE' to ~/.bashrc"
  else
    echo "'\$SOURCE_LINE' is already in ~/.bashrc"
  fi
}

remove_source_from_bashrc() {
  sed -i "\|\$SOURCE_LINE|d" ~/.bashrc
  echo "Removed '\$SOURCE_LINE' from ~/.bashrc"
}

case "\$1" in
  remove)
    remove_source_from_bashrc
    ;;
  add|*)
    add_source_to_bashrc
    ;;
esac
EOF

# Make the manage_ros_env.sh script executable
chmod +x "${MANAGE_SCRIPT}"

