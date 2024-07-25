# syntax=docker/dockerfile:1

# base image
FROM ubuntu:22.04 as base
SHELL ["/bin/bash", "-c"]

# username, uid, gid
ARG USER=user
ARG UID=1000
ARG GID=1000
ARG OBELISK_ROOT=/
ENV USER=$USER
ENV UID=$UID
ENV GID=$GID
ENV OBELISK_ROOT=$OBELISK_ROOT

# set timezone
ENV TZ=America/Los_Angeles
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# basic dependencies from docker_setup.sh (up until sudo and below)
RUN apt-get update && apt-get install -y \
    curl \
    build-essential \
    cmake \
    clang-tools-12 \
    nano \
    vim \
    git \
    libeigen3-dev \
    x11-apps \
    locales \
    sudo && \
    rm -rf /var/lib/apt/lists/* && \
    locale-gen en_US.UTF-8

# create non-root user with sudo privileges for certain commands
RUN groupadd --gid $GID $USER && \
    useradd --uid $UID --gid $GID -m $USER -d /home/${USER} --shell /usr/bin/bash && \
    echo "${USER}:password" | chpasswd && \
    usermod -aG sudo ${USER} && \
    echo "%${USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# switch to new user and workdir
USER ${UID}

# Install pinocchio
RUN sudo apt-get update && sudo apt-get install -qqy lsb-release curl && \
    mkdir -p /etc/apt/keyrings && \
    curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
    | sudo tee /etc/apt/keyrings/robotpkg.asc && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
    | sudo tee /etc/apt/sources.list.d/robotpkg.list && \
    sudo apt update && \
    sudo apt install -qqy robotpkg-pinocchio

# add local user binary folder to PATH variable
ENV PATH="${PATH}:/opt/openrobots/bin:/home/${USER}/.local/bin"
ENV XDG_RUNTIME_DIR=/run/user/${UID}
ENV PKG_CONFIG_PATH="${PKG_CONFIG_PATH}:/opt/openrobots/lib/pkgconfig"
ENV LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/opt/openrobots/lib"
ENV CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}:/opt/openrobots"

# Install OSQP and OSQPEigen
RUN mkdir -p ${HOME}/repos && cd ${HOME}/repos && \
    git clone --recursive https://github.com/osqp/osqp && \
    cd osqp && \
    mkdir build && cd build && \
    cmake ../ && \
    cmake --build . && \
    sudo cmake --build . --target install && \
    cd ${HOME}/repos && \
    git clone https://github.com/robotology/osqp-eigen.git && \
    cd osqp-eigen && \
    mkdir build && \
    cd build && \
    cmake ../ && \
    make && \
    sudo make install
# ENV OsqpEigen_DIR = "/home/${USER}"

# clone Obelisk into this docker image at the $OBELISK_ROOT path
RUN git clone https://github.com/Caltech-AMBER/obelisk.git /home/${USER}/obelisk
ENV OBELISK_ROOT=/home/${USER}/obelisk

# run setup script in Dockerfile
RUN source /home/${USER}/obelisk/setup.sh --pixi --cyclone-perf && \
    bash /home/${USER}/obelisk/scripts/install_sys_deps.sh -y --no-source-ros

WORKDIR /home/${USER}
