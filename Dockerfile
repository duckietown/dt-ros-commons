# parameters
ARG REPO_NAME="dt-ros-commons"
ARG MAINTAINER="Andrea F. Daniele (afdaniele@ttic.edu)"

ARG ARCH=arm32v7
ARG DISTRO=ente
ARG BASE_TAG=${DISTRO}-${ARCH}
ARG BASE_IMAGE=dt-commons
ARG LAUNCHER=default

# define base image
FROM duckietown/${BASE_IMAGE}:${BASE_TAG}

# recall all arguments
ARG REPO_NAME
ARG ARCH
ARG DISTRO
ARG ROS_DISTRO
ARG BASE_TAG
ARG BASE_IMAGE
ARG MAINTAINER
ARG LAUNCHER

# configure environment
ENV CATKIN_WS_DIR "${SOURCE_DIR}/catkin_ws"
ENV ROS_LANG_DISABLE gennodejs:geneus:genlisp

# define and create repository path
ARG REPO_PATH="${CATKIN_WS_DIR}/src/${REPO_NAME}"
ARG LAUNCH_PATH="${LAUNCH_DIR}/${REPO_NAME}"
RUN mkdir -p "${REPO_PATH}"
RUN mkdir -p "${LAUNCH_PATH}"
WORKDIR "${REPO_PATH}"

# keep some arguments as environment variables
ENV DT_MODULE_TYPE "${REPO_NAME}"
ENV DT_MAINTAINER "${MAINTAINER}"
ENV DT_REPO_PATH "${REPO_PATH}"
ENV DT_LAUNCH_PATH "${LAUNCH_PATH}"
ENV DT_LAUNCHER "${LAUNCHER}"

# create repo directory
RUN mkdir -p "${REPO_PATH}"

# install apt dependencies
COPY ./dependencies-apt.txt "${REPO_PATH}/"
RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    $(awk -F: '/^[^#]/ { print $1 }' dependencies-apt.txt | uniq) \
  && rm -rf /var/lib/apt/lists/*

# install python dependencies
COPY ./dependencies-py3.txt "${REPO_PATH}/"
RUN pip3 install -r ${REPO_PATH}/dependencies-py3.txt

# copy the source code
COPY . "${REPO_PATH}/"

# build packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
  catkin build \
    --workspace ${CATKIN_WS_DIR}/

# install launcher scripts
COPY ./launchers/. "${LAUNCH_PATH}/"
RUN dt-install-launchers "${LAUNCH_PATH}"

# define default command
CMD ["bash", "-c", "dt-launcher-${DT_LAUNCHER}"]

# store module metadata
LABEL org.duckietown.label.architecture="${ARCH}" \
    org.duckietown.label.module.type="${REPO_NAME}" \
    org.duckietown.label.code.location="${REPO_PATH}" \
    org.duckietown.label.code.version.distro="${DISTRO}" \
    org.duckietown.label.base.image="${BASE_IMAGE}" \
    org.duckietown.label.base.tag="${BASE_TAG}" \
    org.duckietown.label.maintainer="${MAINTAINER}"

