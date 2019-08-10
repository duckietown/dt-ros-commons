ARG ARCH=arm32v7
ARG ROS_DISTRO=kinetic
ARG BASE_TAG=devel20-${ARCH}

FROM duckietown/dt-ros-${ROS_DISTRO}-base:${BASE_TAG}

# configure environment
ENV SOURCE_DIR /code
ENV CATKIN_WS_DIR "${SOURCE_DIR}/catkin_ws"
ENV ROS_LANG_DISABLE=gennodejs:geneus:genlisp
ENV READTHEDOCS True
WORKDIR "${CATKIN_WS_DIR}"

ARG REPO_PATH="${CATKIN_WS_DIR}/src/dt-ros-commons"

# create repo directory
RUN mkdir -p "${REPO_PATH}"

# copy entire repo
COPY . "${REPO_PATH}/"

# build packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
  && catkin build \
    --workspace "${CATKIN_WS_DIR}/"

# configure entrypoint
COPY ./assets/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

LABEL maintainer="Andrea F. Daniele (afdaniele@ttic.edu)"
