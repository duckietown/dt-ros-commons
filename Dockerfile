ARG ARCH=arm32v7
ARG ROS_DISTRO=kinetic

FROM duckietown/rpi-ros-${ROS_DISTRO}-base:master19-${ARCH}

# configure environment
ENV SOURCE_DIR /code
ENV CATKIN_WS_DIR "${SOURCE_DIR}/catkin_ws"
ENV INSTALL_DIR /usr/local
ENV ROS_LANG_DISABLE=gennodejs:geneus:genlisp
ENV READTHEDOCS True
WORKDIR "${CATKIN_WS_DIR}"

# turn on ARM emulation
RUN ["cross-build-start"]

ARG REPO_PATH="${CATKIN_WS_DIR}/src/dt-ros-commons"

# create repo directory
RUN mkdir -p "${REPO_PATH}"

# copy entire repo
COPY . "${REPO_PATH}/"

# build common packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
  catkin build \
    --workspace ${CATKIN_WS_DIR}/

# turn off ARM emulation
RUN ["cross-build-end"]

# configure entrypoint
COPY ./assets/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

LABEL maintainer="Andrea F. Daniele (afdaniele@ttic.edu)"
