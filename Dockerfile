# Note: this container will have the name duckietown/XXX-commons

ARG ARCH=arm32v7
ARG ROS_DISTRO=kinetic
ARG BRANCH=afdaniele-devel
# ARG BRANCH=master19

FROM duckietown/${ARCH}-ros-${ROS_DISTRO}-base:${BRANCH}

# configure environment
ENV SOURCE_DIR /code
ENV INSTALL_DIR /usr/local
ENV ROS_LANG_DISABLE=gennodejs:geneus:genlisp
WORKDIR "${SOURCE_DIR}"

# turn on ARM emulation
RUN ["cross-build-start"]

# create directories
RUN mkdir -p "${SOURCE_DIR}"

# copy source code
COPY ./catkin_ws "${SOURCE_DIR}/catkin_ws"

# build common packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
  catkin_make \
    -j \
    -C ${SOURCE_DIR}/catkin_ws/ \
    -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} \
    install && \
  rm -rf ${SOURCE_DIR}/catkin_ws

# turn off ARM emulation
RUN ["cross-build-end"]

# configure entrypoint
COPY ./assets/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

LABEL maintainer="Andrea F. Daniele (afdaniele@ttic.edu)"
