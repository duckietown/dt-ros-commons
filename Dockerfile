# parameters
ARG REPO_NAME="dt-ros-commons"

ARG ARCH=arm32v7
ARG MAJOR=ente
ARG BASE_TAG=${MAJOR}-${ARCH}
ARG BASE_IMAGE=dt-ros-base

# extend dt-commons
ARG SUPER_IMAGE=dt-commons
ARG SUPER_IMAGE_MAJOR=daffy
ARG SUPER_IMAGE_TAG=${SUPER_IMAGE_MAJOR}-${ARCH}
FROM duckietown/${SUPER_IMAGE}:${SUPER_IMAGE_TAG} as dt-commons

# define base image
FROM duckietown/${BASE_IMAGE}:${BASE_TAG}

# copy stuff from the super image
COPY --from=dt-commons /environment.sh /environment.sh

# configure environment
ENV SOURCE_DIR /code
ENV CATKIN_WS_DIR "${SOURCE_DIR}/catkin_ws"
ENV DUCKIEFLEET_ROOT "/data/config"
ENV ROS_LANG_DISABLE gennodejs:geneus:genlisp
ENV READTHEDOCS True
WORKDIR "${CATKIN_WS_DIR}"

# define repository path
ARG REPO_NAME
ARG REPO_PATH="${CATKIN_WS_DIR}/src/${REPO_NAME}"
WORKDIR "${REPO_PATH}"

# create repo directory
RUN mkdir -p "${REPO_PATH}"

# copy dependencies files only
COPY ./dependencies-apt.txt "${REPO_PATH}/"

# install apt dependencies
RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    $(awk -F: '/^[^#]/ { print $1 }' dependencies-apt.txt | uniq) \
  && rm -rf /var/lib/apt/lists/*

# install python3 dependencies
COPY ./dependencies-py3.txt "${REPO_PATH}/"
RUN pip3 install -r ${REPO_PATH}/dependencies-py3.txt

# create a workspace where we can build ROS
ARG ROS_PKGS_SRC_DIR="${ROS_SRC_DIR}/${REPO_NAME}/"
RUN mkdir -p ${ROS_PKGS_SRC_DIR}
WORKDIR ${ROS_PKGS_SRC_DIR}
RUN catkin \
  config \
    --init \
    -DCMAKE_BUILD_TYPE=Release \
    --install-space ${ROS_INSTALL_DIR} \
    --install

RUN wstool \
  init \
  -j $(nproc) \
  src

# analyze additional ROS packages
COPY packages-ros.txt /tmp/packages-ros.txt
RUN \
  set -e; \
  PACKAGES=$(sed -e '/#[ ]*BLACKLIST/,$d' /tmp/packages-ros.txt | sed "/^#/d" | uniq); \
  NUM_PACKAGES=$(echo $PACKAGES | sed '/^\s*#/d;/^\s*$/d' | wc -l); \
  if [ $NUM_PACKAGES -ge 1 ]; then \
    # merge ROS packages into the current workspace
    dt_analyze_packages /tmp/packages-ros.txt; \
    # install all python dependencies (replacing python -> python3)
    dt_install_dependencies --python-deps; \
    # install all non-python dependencies (exclude libboost, we build it from source for python3)
    dt_install_dependencies --no-python-deps; \
    # replace python -> python3 in all the shebangs of the packages
    dt_py2to3; \
    # blacklist ROS packages
    SKIP_BLACKLIST=$(grep -q "BLACKLIST" /tmp/packages-ros.txt && echo $?); \
    BLACKLIST=$(sed -e "1,/#[ ]*BLACKLIST/d" /tmp/packages-ros.txt | sed "/^#/d" | uniq); \
    BLACKLIST_LEN=$(echo $BLACKLIST | sed '/^\s*#/d;/^\s*$/d' | wc -l); \
    if [ $BLACKLIST_LEN -ge 1 ]; then \
      catkin config \
        --append-args \
        --blacklist $BLACKLIST; \
    fi; \
    # build ROS packages
    catkin build; \
    catkin clean -y --logs; \
  fi; \
  set +e

# RUN rm -rf ${ROS_SRC_DIR}/src/*
#
# # merge ROS packages into the current workspace
# COPY packages-ros.txt /tmp/packages-ros.txt
# RUN dt_analyze_packages /tmp/packages-ros.txt \
#   && ls -alh ${ROS_SRC_DIR}/src/ \
#   && dt_py2to3
#
# # install all python dependencies (replacing python -> python3)
# RUN dt_install_dependencies --python-deps
#
# # install all non-python dependencies (exclude libboost, we build it from source for python3)
# RUN dt_install_dependencies --no-python-deps
#
# # replace python -> python3 in all the shebangs of the packages
# # RUN dt_py2to3
#
# # blacklist ROS packages
# RUN \
#   if grep -Fxq "BLACKLIST" /tmp/packages-ros.txt; then \
#     catkin config \
#       --append-args \
#       --blacklist $(sed -e "1,/#[ ]*BLACKLIST/d" /tmp/packages-ros.txt | sed "/^#/d" | uniq); \
#   fi
#
# # build ROS
# RUN catkin build \
#   --workspace ${ROS_SRC_DIR}/

# return to project code
WORKDIR "${REPO_PATH}"

# copy the source code
COPY . "${REPO_PATH}/"

# build packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
  catkin build \
    --workspace ${CATKIN_WS_DIR}/

# configure entrypoint
COPY assets/entrypoint.sh /entrypoint.sh
RUN echo "source /entrypoint.sh" >> /etc/bash.bashrc
ENTRYPOINT ["/entrypoint.sh"]

# store module name
LABEL org.duckietown.label.module.type "${REPO_NAME}"
ENV DT_MODULE_TYPE "${REPO_NAME}"

# store module metadata
ARG ARCH
ARG MAJOR
ARG BASE_TAG
ARG BASE_IMAGE
LABEL org.duckietown.label.architecture "${ARCH}"
LABEL org.duckietown.label.code.location "${REPO_PATH}"
LABEL org.duckietown.label.code.version.major "${MAJOR}"
LABEL org.duckietown.label.base.image "${BASE_IMAGE}:${BASE_TAG}"

# define maintainer
LABEL maintainer="Andrea F. Daniele (afdaniele@ttic.edu)"
