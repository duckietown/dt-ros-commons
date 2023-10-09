# parameters
ARG ARCH
ARG DISTRO
ARG DOCKER_REGISTRY
ARG BASE_REPOSITORY
ARG BASE_ORGANIZATION=duckietown
ARG BASE_TAG=${DISTRO}-${ARCH}
ARG LAUNCHER=default
ARG OS_FAMILY=ubuntu
ARG OS_DISTRO=focal
ARG ROS_DISTRO=noetic
# ---
ARG PROJECT_NAME
ARG PROJECT_MAINTAINER
ARG PROJECT_DESCRIPTION
ARG PROJECT_ICON="square"
ARG PROJECT_FORMAT_VERSION


# duckietown environment image
FROM ${DOCKER_REGISTRY}/${BASE_ORGANIZATION}/${BASE_REPOSITORY}:${BASE_TAG} as duckietown

# base image
FROM docker.io/${ARCH}/${OS_FAMILY}:${OS_DISTRO}

# configure pip
ARG PIP_INDEX_URL="https://pypi.org/simple"
ENV PIP_INDEX_URL=${PIP_INDEX_URL}

#
# =====> Replicate the configuration from dt-base-environment ==============================

# recall arguments
ARG OS_FAMILY
ARG OS_DISTRO
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
    PYTHONIOENCODING="UTF-8" \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED="1" \
    DEBIAN_FRONTEND="noninteractive" \
    DISABLE_CONTRACTS=1 \
    QEMU_EXECVE=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_ROOT_USER_ACTION=ignore

# nvidia runtime configuration
ENV NVIDIA_VISIBLE_DEVICES="all" \
    NVIDIA_DRIVER_CAPABILITIES="all"

# OS info
ENV OS_FAMILY="${OS_FAMILY}" \
    OS_DISTRO="${OS_DISTRO}"

# code environment
ENV SOURCE_DIR="/code/src" \
    LAUNCHERS_DIR="/launch" \
    USER_WS_DIR="/user_ws" \
    MINIMUM_DTPROJECT_FORMAT_VERSION="4"

# start inside the course code directory
WORKDIR "${SOURCE_DIR}"

# copy entire project
COPY --from=duckietown "${SOURCE_DIR}/dt-base-environment" "${SOURCE_DIR}/dt-base-environment"

# remove packages that are not supported by catkin
RUN rm -rf "${SOURCE_DIR}/dt-base-environment/packages"

# copy assets
RUN cp ${SOURCE_DIR}/dt-base-environment/assets/qemu/${TARGETPLATFORM}/* /usr/bin/ && \
    cp ${SOURCE_DIR}/dt-base-environment/assets/bin/* /usr/local/bin/

# Install gnupg required for apt-key (not in base image since Focal)
RUN apt-get update \
  && apt-get install -y --no-install-recommends \
        gnupg \
        python3-dev \
        python3-pip \
        make \
        cmake \
        gcc \
        sudo \
  && rm -rf /var/lib/apt/lists/*

# upgrade PIP
RUN python3 -m pip install pip==22.2 && \
    ln -s $(which python3.8) /usr/bin/pip3.8

# install dependencies (PIP3)
RUN dt-pip3-install "${SOURCE_DIR}/dt-base-environment/dependencies-py3.*"

# configure terminal size in docker: https://docs.python.org/3/library/shutil.html#shutil.get_terminal_size
ENV COLUMNS 160

# install launcher scripts
COPY --from=duckietown "${LAUNCHERS_DIR}/dt-base-environment" "${LAUNCHERS_DIR}/dt-base-environment"
RUN dt-install-launchers "${LAUNCHERS_DIR}/dt-base-environment"

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

# install dependencies (PIP3)
RUN dt-pip3-install "${SOURCE_DIR}/dt-commons/dependencies-py3.*"

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

# configure arch-specific environment
RUN ${SOURCE_DIR}/dt-commons/assets/setup/${TARGETPLATFORM}/setup.sh

# copy binaries and scripts
RUN cp ${SOURCE_DIR}/dt-commons/assets/bin/* /usr/local/bin/ && \
    cp ${SOURCE_DIR}/dt-commons/assets/entrypoint.sh /entrypoint.sh && \
    cp ${SOURCE_DIR}/dt-commons/assets/environment.sh /environment.sh

# source environment on every bash session
RUN echo "source /environment.sh" >> /etc/bash.bashrc

# configure entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# install launcher scripts
COPY --from=duckietown "${LAUNCHERS_DIR}/dt-commons" "${LAUNCHERS_DIR}/dt-commons"
RUN dt-install-launchers "${LAUNCHERS_DIR}/dt-commons"

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

# recall arguments
ARG ARCH
ARG DISTRO
ARG DOCKER_REGISTRY
ARG PROJECT_NAME
ARG PROJECT_DESCRIPTION
ARG PROJECT_MAINTAINER
ARG PROJECT_ICON
ARG PROJECT_FORMAT_VERSION
ARG BASE_TAG
ARG BASE_REPOSITORY
ARG BASE_ORGANIZATION
ARG LAUNCHER
ARG ROS_DISTRO

# ROS info
ENV ROS_DISTRO="${ROS_DISTRO}" \
    CATKIN_WS_DIR="/code"

# check build arguments
RUN dt-args-check \
    "PROJECT_NAME" "${PROJECT_NAME}" \
    "PROJECT_DESCRIPTION" "${PROJECT_DESCRIPTION}" \
    "PROJECT_MAINTAINER" "${PROJECT_MAINTAINER}" \
    "PROJECT_ICON" "${PROJECT_ICON}" \
    "PROJECT_FORMAT_VERSION" "${PROJECT_FORMAT_VERSION}" \
    "ARCH" "${ARCH}" \
    "DISTRO" "${DISTRO}" \
    "DOCKER_REGISTRY" "${DOCKER_REGISTRY}" \
    "BASE_REPOSITORY" "${BASE_REPOSITORY}"
RUN dt-check-project-format "${PROJECT_FORMAT_VERSION}"

# define/create repository path
ARG PROJECT_PATH="${SOURCE_DIR}/${PROJECT_NAME}"
ARG PROJECT_LAUNCHERS_PATH="${LAUNCHERS_DIR}/${PROJECT_NAME}"
RUN mkdir -p "${PROJECT_PATH}" "${PROJECT_LAUNCHERS_PATH}"
WORKDIR "${PROJECT_PATH}"

# keep some arguments as environment variables
ENV DT_PROJECT_NAME="${PROJECT_NAME}" \
    DT_PROJECT_DESCRIPTION="${PROJECT_DESCRIPTION}" \
    DT_PROJECT_MAINTAINER="${PROJECT_MAINTAINER}" \
    DT_PROJECT_ICON="${PROJECT_ICON}" \
    DT_PROJECT_PATH="${PROJECT_PATH}" \
    DT_PROJECT_LAUNCHERS_PATH="${PROJECT_LAUNCHERS_PATH}" \
    DT_LAUNCHER="${LAUNCHER}"

# setup ROS sources
RUN apt-key adv \
    --keyserver hkp://keyserver.ubuntu.com:80 \
    --recv-keys F42ED6FBAB17C654 \
    && echo "deb http://packages.ros.org/ros/ubuntu ${OS_DISTRO} main" >> /etc/apt/sources.list.d/ros.list

# install dependencies (APT)
COPY ./dependencies-apt.txt "${PROJECT_PATH}/"
RUN dt-apt-install "${PROJECT_PATH}/dependencies-apt.txt"

# install dependencies (PIP3)
COPY ./dependencies-py3.* "${PROJECT_PATH}/"
RUN dt-pip3-install "${PROJECT_PATH}/dependencies-py3.*"

# install LCM
ENV LCM_VERSION="1.5.0"
RUN cd /tmp/ \
    && git clone -b "v${LCM_VERSION}" https://github.com/lcm-proj/lcm \
    && mkdir -p lcm/build \
    && cd lcm/build \
    && cmake .. \
    && make \
    && make install \
    && cd ~ \
    && rm -rf /tmp/lcm

# copy the source code
COPY ./packages "${PROJECT_PATH}/packages"

# build packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
  catkin build \
    --workspace ${CATKIN_WS_DIR}/

# install launcher scripts
COPY ./launchers/. "${PROJECT_LAUNCHERS_PATH}/"
RUN dt-install-launchers "${PROJECT_LAUNCHERS_PATH}"

# install scripts
COPY ./assets/entrypoint.d "${PROJECT_PATH}/assets/entrypoint.d"
COPY ./assets/environment.d "${PROJECT_PATH}/assets/environment.d"

# define default command
CMD ["bash", "-c", "dt-launcher-${DT_LAUNCHER}"]

# store module metadata
LABEL \
    # module info
    org.duckietown.label.project.name="${PROJECT_NAME}" \
    org.duckietown.label.project.description="${PROJECT_DESCRIPTION}" \
    org.duckietown.label.project.maintainer="${PROJECT_MAINTAINER}" \
    org.duckietown.label.project.icon="${PROJECT_ICON}" \
    org.duckietown.label.project.path="${PROJECT_PATH}" \
    org.duckietown.label.project.launchers.path="${PROJECT_LAUNCHERS_PATH}" \
    # format
    org.duckietown.label.format.version="${PROJECT_FORMAT_VERSION}" \
    # platform info
    org.duckietown.label.platform.os="${TARGETOS}" \
    org.duckietown.label.platform.architecture="${TARGETARCH}" \
    org.duckietown.label.platform.variant="${TARGETVARIANT}" \
    # code info
    org.duckietown.label.code.distro="${DISTRO}" \
    org.duckietown.label.code.launcher="${LAUNCHER}" \
    org.duckietown.label.code.python.registry="${PIP_INDEX_URL}" \
    # base info
    org.duckietown.label.base.organization="${BASE_ORGANIZATION}" \
    org.duckietown.label.base.repository="${BASE_REPOSITORY}" \
    org.duckietown.label.base.tag="${BASE_TAG}"
