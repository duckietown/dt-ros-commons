# parameters
ARG ARCH=arm64v8
ARG ROS_DISTRO=noetic
ARG OS_FAMILY=ubuntu
ARG OS_DISTRO=focal
ARG DISTRO=ente
ARG LAUNCHER=default
# ---
ARG REPO_NAME="dt-ros-commons"
ARG MAINTAINER="Andrea F. Daniele (afdaniele@duckietown.com)"
ARG DESCRIPTION="Base image containing common libraries and environment setup for ROS applications."
ARG ICON="square"

# duckietown environment image
ARG DISTRO=ente
ARG BASE_TAG=${DISTRO}-${ARCH}
ARG BASE_IMAGE=dt-commons
ARG DOCKER_REGISTRY=docker.io
FROM ${DOCKER_REGISTRY}/duckietown/${BASE_IMAGE}:${BASE_TAG} as duckietown

# base image
FROM ${ARCH}/${OS_FAMILY}:${OS_DISTRO} as base

# configure pip
ARG PIP_INDEX_URL="https://pypi.org/simple"
ENV PIP_INDEX_URL=${PIP_INDEX_URL}

#
# =====> Replicate the configuration from dt-base-environment ==============================

# recall all arguments
ARG OS_FAMILY
ARG OS_DISTRO
ARG ROS_DISTRO
ARG DISTRO
ARG LAUNCHER
ARG REPO_NAME
ARG DESCRIPTION
ARG MAINTAINER
ARG ICON
ARG DISTRO
ARG BASE_TAG
ARG BASE_IMAGE
ARG DOCKER_REGISTRY
# - buildkit
ARG TARGETPLATFORM
ARG TARGETOS
ARG TARGETARCH
ARG TARGETVARIANT

# setup environment
ENV INITSYSTEM="off" \
    TERM="xterm" \
    LANG="C.UTF-8" \
    LC_ALL="C.UTF-8" \
    READTHEDOCS="True" \
    CATKIN_VERSION="0.8.10" \
    PYTHONIOENCODING="UTF-8" \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED="1" \
    DEBIAN_FRONTEND="noninteractive" \
    DISABLE_CONTRACTS=1 \
    QEMU_EXECVE=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_ROOT_USER_ACTION=ignore \
    ROS_LANG_DISABLE="gennodejs:geneus:genlisp"

# nvidia runtime configuration
ENV NVIDIA_VISIBLE_DEVICES="all" \
    NVIDIA_DRIVER_CAPABILITIES="all"

# keep some arguments as environment variables
ENV OS_FAMILY="${OS_FAMILY}" \
    OS_DISTRO="${OS_DISTRO}" \
    ROS_DISTRO="${ROS_DISTRO}" \
    DT_MODULE_TYPE="${REPO_NAME}" \
    DT_MODULE_DESCRIPTION="${DESCRIPTION}" \
    DT_MODULE_ICON="${ICON}" \
    DT_MAINTAINER="${MAINTAINER}" \
    DT_LAUNCHER="${LAUNCHER}"

# code environment
ENV SOURCE_DIR="/code" \
    LAUNCH_DIR="/launch" \
    CATKIN_WS_DIR="/code/catkin_ws"
ENV USER_WS_DIR "${SOURCE_DIR}/user_ws"
WORKDIR "${SOURCE_DIR}"

# Install gnupg required for apt-key (not in base image since Focal)
RUN apt-get update \
  && apt-get install -y --no-install-recommends gnupg \
  && rm -rf /var/lib/apt/lists/*

# copy entire project
COPY --from=duckietown "${SOURCE_DIR}/dt-base-environment" "${SOURCE_DIR}/dt-base-environment"

# copy assets
RUN cp ${SOURCE_DIR}/dt-base-environment/assets/qemu/${TARGETPLATFORM}/* /usr/bin/ && \
    cp ${SOURCE_DIR}/dt-base-environment/assets/bin/* /usr/local/bin/

# install dependencies (APT)
RUN dt-apt-install "${SOURCE_DIR}/dt-base-environment/dependencies-apt.txt"

# upgrade PIP
RUN python3 -m pip install pip==22.2 && \
    ln -s $(which python3.8) /usr/bin/pip3.8

# install dependencies (PIP3)
RUN dt-pip3-install "${SOURCE_DIR}/dt-base-environment/dependencies-py3.*"

# install launcher scripts
COPY --from=duckietown "${LAUNCH_DIR}/dt-base-environment" "${LAUNCH_DIR}/dt-base-environment"
RUN dt-install-launchers "${LAUNCH_DIR}/dt-base-environment"

# configure catkin to work nicely with docker: https://docs.python.org/3/library/shutil.html#shutil.get_terminal_size
ENV COLUMNS 160

# <===== Replicate the configuration from dt-base-environment ==============================
#

#
# =====> Replicate the configuration from dt-commons =======================================

# copy entire project
COPY --from=duckietown "${SOURCE_DIR}/dt-commons" "${SOURCE_DIR}/dt-commons"

# duckie user
ENV DT_USER_NAME="duckie" \
    DT_USER_UID=2222 \
    DT_GROUP_NAME="duckie" \
    DT_GROUP_GID=2222 \
    DT_USER_HOME="/home/duckie"

# install dependencies (APT)
RUN dt-apt-install "${SOURCE_DIR}/dt-commons/dependencies-apt.txt"

# install dependencies (PIP3)
RUN dt-pip3-install "${SOURCE_DIR}/dt-commons/dependencies-py3.*"

# install LCM
RUN cd /tmp/ \
    && git clone -b v1.4.0 https://github.com/lcm-proj/lcm \
    && mkdir -p lcm/build \
    && cd lcm/build \
    && cmake .. \
    && make \
    && make install \
    && cd ~ \
    && rm -rf /tmp/lcm

# configure arch-specific environment
RUN ${SOURCE_DIR}/dt-commons/assets/setup/${TARGETPLATFORM}/setup.sh

# create `duckie` user
RUN addgroup --gid ${DT_GROUP_GID} "${DT_GROUP_NAME}" && \
    useradd \
        --create-home \
        --home-dir "${DT_USER_HOME}" \
        --comment "Duckietown User" \
        --shell "/bin/bash" \
        --password "aa26uhROPk6sA" \
        --uid ${DT_USER_UID} \
        --gid ${DT_GROUP_GID} \
        "${DT_USER_NAME}"

# copy binaries and scripts
RUN cp ${SOURCE_DIR}/dt-commons/assets/bin/* /usr/local/bin/ && \
    cp ${SOURCE_DIR}/dt-commons/assets/entrypoint.sh /entrypoint.sh && \
    cp ${SOURCE_DIR}/dt-commons/assets/environment.sh /environment.sh

# install launcher scripts
COPY --from=duckietown "${LAUNCH_DIR}/dt-commons" "${LAUNCH_DIR}/dt-commons"
RUN dt-install-launchers "${LAUNCH_DIR}/dt-commons"

# source environment on every bash session
RUN echo "source /environment.sh" >> ~/.bashrc

# configure entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# create health file
RUN echo ND > /health &&  \
    chmod 777 /health && \
    sudo -u ${DT_USER_NAME} touch /health

# define healthcheck
HEALTHCHECK \
    --interval=30s \
    CMD cat /health && grep -q '^healthy\|ND$' /health


# <===== Replicate the configuration from dt-commons =======================================
#

#
# =====> This DTProject ====================================================================

# define and create repository paths
ARG REPO_PATH="${CATKIN_WS_DIR}/src/${REPO_NAME}"
ARG LAUNCH_PATH="${LAUNCH_DIR}/${REPO_NAME}"
RUN mkdir -p "${CATKIN_WS_DIR}" "${REPO_PATH}" "${LAUNCH_PATH}" "${USER_WS_DIR}"
ENV DT_REPO_PATH="${REPO_PATH}" \
    DT_LAUNCH_PATH="${LAUNCH_PATH}"

# setup ROS sources
RUN apt-key adv \
    --keyserver hkp://keyserver.ubuntu.com:80 \
    --recv-keys F42ED6FBAB17C654 \
    && echo "deb http://packages.ros.org/ros/ubuntu ${OS_DISTRO} main" >> /etc/apt/sources.list.d/ros.list

# install dependencies (APT)
COPY ./dependencies-apt.txt "${REPO_PATH}/"
RUN dt-apt-install "${REPO_PATH}/dependencies-apt.txt"

# install dependencies (PIP3)
COPY ./dependencies-py3.* "${REPO_PATH}/"
RUN dt-pip3-install "${REPO_PATH}/dependencies-py3.*"

# copy the source code
COPY ./packages "${REPO_PATH}/packages"

# build packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
  catkin build \
    --workspace ${CATKIN_WS_DIR}/

# install launcher scripts
COPY ./launchers/. "${LAUNCH_PATH}/"
RUN dt-install-launchers "${LAUNCH_PATH}"

# install scripts
COPY ./assets/entrypoint.d "${REPO_PATH}/assets/entrypoint.d"

# define default command
CMD ["bash", "-c", "dt-launcher-${DT_LAUNCHER}"]

# store module metadata
LABEL org.duckietown.label.module.type="${REPO_NAME}" \
    org.duckietown.label.module.description="${DESCRIPTION}" \
    org.duckietown.label.module.icon="${ICON}" \
    org.duckietown.label.architecture="${ARCH}" \
    org.duckietown.label.code.location="${REPO_PATH}" \
    org.duckietown.label.code.version.distro="${DISTRO}" \
    org.duckietown.label.base.image="${BASE_IMAGE}" \
    org.duckietown.label.base.tag="${BASE_TAG}" \
    org.duckietown.label.maintainer="${MAINTAINER}"
