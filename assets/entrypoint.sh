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
  "${SOURCE_DIR}/catkin_ws/devel/setup.bash"
  "${SOURCE_DIR}/setup.sh"
)

# check the mandatory arguments
VEHICLE_NAME_IS_SET=1
if [ ${#VEHICLE_NAME} -le 0 ]; then
  VEHICLE_NAME_IS_SET=0
  VEHICLE_NAME=$(hostname)
  echo "The environment variable VEHICLE_NAME is not set. Using '${VEHICLE_NAME}'."
fi
export VEHICLE_NAME="${VEHICLE_NAME}"

# check optional arguments
VEHICLE_IP_IS_SET=0
if [ ${#VEHICLE_IP} -ne 0 ]; then
  VEHICLE_IP_IS_SET=1
  echo "The environment variable VEHICLE_IP is set to '${VEHICLE_IP}'. Adding to /etc/hosts."
  echo "${VEHICLE_IP} ${VEHICLE_NAME} ${VEHICLE_NAME}.local" >> /etc/hosts
fi

# configure hosts
if [ "${VEHICLE_NAME_IS_SET}" -eq "0" ]; then
  # vehicle name not set (assume vehicle is localhost)
  echo "127.0.0.1 ${VEHICLE_NAME} ${VEHICLE_NAME}.local" >> /etc/hosts
fi

# setup ros environment
#TODO(andrea): check if necessary when we switch to ROS2
for ROS_SETUP_FILE in "${ROS_SETUP[@]}"; do
  if [ -f "${ROS_SETUP_FILE}" ]; then
    source "${ROS_SETUP_FILE}";
  fi
done

# configure ROS IP
#TODO(andrea): remove when we switch to ROS2
CONTAINER_IP=$(hostname -I 2>/dev/null | cut -d " " -f 1)
export ROS_IP=${CONTAINER_IP}

# configure ROS MASTER URI
#TODO(andrea): remove when we switch to ROS2
if [ "${ROS_MASTER_URI_IS_SET}" -eq "0" ]; then
  export ROS_MASTER_URI="http://${VEHICLE_NAME}.local:11311/"
fi

# execute given commands (if any)
exec "$@"
