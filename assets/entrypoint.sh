#!/bin/bash
set -e

# check if ROS_MASTER_URI is set
ROS_MASTER_URI_IS_SET=0
if [ ! -z "${ROS_MASTER_URI}" ]; then
  ROS_MASTER_URI_IS_SET=1
fi

# constants
ROS_SETUP=(
  "/opt/ros/${ROS_DISTRO}/setup.bash"
  "${INSTALL_DIR}/setup.bash"
)

#TODO (andrea): replace `devel` with `build`
ROS_WS_SETUP="${SOURCE_DIR}/catkin_ws/devel/setup.bash"
CODE_SETUP="${SOURCE_DIR}/setup.sh"

# check the mandatory arguments
VEHICLE_NAME_IS_SET=1
if [ ${#VEHICLE_NAME} -le 0 ]; then
  VEHICLE_NAME_IS_SET=0
  VEHICLE_NAME=$(hostname)
  echo "The environment variable VEHICLE_NAME is not set. Using '${VEHICLE_NAME}'."
fi
export VEHICLE_NAME="${VEHICLE_NAME}"

# configure hosts
if [ "${VEHICLE_NAME_IS_SET}" -eq "0" ]; then
  echo "127.0.0.1 ${VEHICLE_NAME} ${VEHICLE_NAME}.local" >> /etc/hosts
fi

# setup ros environment
#TODO(andrea): check if necessary when we switch to ROS2
for ROS_SETUP_FILE in "${ROS_SETUP[@]}"; do
  if [ -f "${ROS_SETUP_FILE}" ]; then
    source "${ROS_SETUP_FILE}";
  fi
done

# setup ROS environment (if present)
#TODO(andrea): check if necessary when we switch to ROS2
if [ -f "${ROS_WS_SETUP}" ]; then
  source "${ROS_WS_SETUP}"
fi

# setup custom environment (if present)
if [ -f "${CODE_SETUP}" ]; then
  source "${CODE_SETUP}"
fi

# configure ROS IP
#TODO(andrea): remove when we switch to ROS2
CONTAINER_IP=$(hostname -I 2>/dev/null | cut -d " " -f 1)
export ROS_IP=${CONTAINER_IP}

# configure ROS MASTER URI
#TODO(andrea): remove when we switch to ROS2
if [ "${ROS_MASTER_URI_IS_SET}" -eq "0" ]; then
  export ROS_MASTER_URI="http://${VEHICLE_NAME}:11311/"
fi

# execute given commands (if any)
exec "$@"
