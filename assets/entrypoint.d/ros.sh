#!/usr/bin/env bash

# check if ROS_MASTER_URI is set
ROS_MASTER_URI_IS_SET=0
if [ -n "${ROS_MASTER_URI-}" ]; then
    ROS_MASTER_URI_IS_SET=1
    info "Forcing ROS_MASTER_URI=${ROS_MASTER_URI}"
fi

# check if ROS_HOSTNAME is set
ROS_HOSTNAME_IS_SET=0
if [ -n "${ROS_HOSTNAME-}" ]; then
    ROS_HOSTNAME_IS_SET=1
    info "Forcing ROS_HOSTNAME=${ROS_HOSTNAME}"
fi

# check if ROS_IP is set
ROS_IP_IS_SET=0
if [ -n "${ROS_IP-}" ]; then
    ROS_IP_IS_SET=1
    info "Forcing ROS_IP=${ROS_IP}"
fi

# constants
ROS_SETUP=(
    "/opt/ros/${ROS_DISTRO}/setup.bash"
    "${CATKIN_WS_DIR-}/devel/setup.bash"
    "${SOURCE_DIR}/setup.bash"
)

# setup ros environment
for ROS_SETUP_FILE in "${ROS_SETUP[@]}"; do
    if [ -f "${ROS_SETUP_FILE}" ]; then
        source "${ROS_SETUP_FILE}"
    fi
done

# configure ROS_HOSTNAME
if [ "${ROS_HOSTNAME_IS_SET}" -eq "0" ]; then
    if is_nethost; then
        # configure ROS_HOSTNAME
        MACHINE_HOSTNAME="$(hostname).local"
        debug "Detected '--net=host', setting ROS_HOSTNAME to '${MACHINE_HOSTNAME}'"
        export ROS_HOSTNAME=${MACHINE_HOSTNAME}
    fi
fi

# configure ROS_IP
if [ "${ROS_IP_IS_SET}" -eq "0" ]; then
    if ! is_nethost; then
        # configure ROS_IP
        CONTAINER_IP=$(hostname -I 2>/dev/null | cut -d " " -f 1)
        debug "Detected '--net=bridge', setting ROS_IP to '${CONTAINER_IP}'"
        export ROS_IP=${CONTAINER_IP}
    fi
fi

# configure ROS MASTER URI
if [ "${ROS_MASTER_URI_IS_SET}" -eq "0" ]; then
    export ROS_MASTER_URI="http://${VEHICLE_NAME}.local:11311/"
fi
