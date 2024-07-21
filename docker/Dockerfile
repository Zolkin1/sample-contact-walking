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
    python3-dev \
    python-is-python3 \
    python3-pip \
    python3-argcomplete \
    libeigen3-dev \
    mesa-utils \
    x11-apps \
    libyaml-dev \
    mesa-common-dev \
    libglfw3-dev \
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

# Install pinocchio
RUN apt-get update && apt-get install -qqy lsb-release curl && \
    mkdir -p /etc/apt/keyrings && \
    curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
    | sudo tee /etc/apt/keyrings/robotpkg.asc && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
    | sudo tee /etc/apt/sources.list.d/robotpkg.list && \
    apt update && \
    apt install -qqy robotpkg-pinocchio

# switch to new user and workdir
USER ${UID}

# run docker setup script in Dockerfile
COPY docker_setup.sh /tmp/docker_setup.sh
RUN source /tmp/docker_setup.sh --skip-docker && \
    sudo rm /tmp/docker_setup.sh

# RUN export PATH=/opt/openrobots/bin:$PATH && \
#     export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH && \
#     export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH && \
#     export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH

# add local user binary folder to PATH variable
ENV PATH="${PATH}:/opt/openrobots/bin:/home/${USER}/.local/bin"
ENV XDG_RUNTIME_DIR=/run/user/${UID}
ENV PKG_CONFIG_PATH = "${PKG_CONFIG_PATH}:/opt/openrobots/lib/pkgconfig"
ENV LD_LIBRARY_PATH = "${LD_LIBRARY_PATH}:/opt/openrobots/lib"
ENV CMAKE_PREFIX_PATH = "${CMAKE_PREFIX_PATH}:/opt/openrobots"
WORKDIR /home/${USER}

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