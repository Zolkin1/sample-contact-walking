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
    evtest \
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

RUN echo "Pulling Obelisk..."

# clone Obelisk into this docker image at the $OBELISK_ROOT path
RUN git clone -b v0.1.0-alpha https://github.com/Caltech-AMBER/obelisk.git /home/${USER}/obelisk
ENV OBELISK_ROOT=/home/${USER}/obelisk

# CppAD and CppAD Codegen
RUN cd /home/${USER} && git clone https://github.com/coin-or/CppAD.git /home/${USER}/cppad.git && \
    cd /home/${USER}/cppad.git && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -Dcppad_cxx_flags=-std=c++17 .. && \
    sudo make install -j8 && cd /home/${USER} && \
    git clone https://github.com/joaoleal/CppADCodeGen.git /home/${USER}/CppADCodeGen && \
    cd CppADCodeGen && mkdir build && cd build && \
    cmake .. && \
    sudo make install -j8

# HPIPM, BLASFEO, HPIPM_CPP
RUN cd /home/${USER} && git clone https://github.com/giaf/blasfeo && git clone https://github.com/giaf/hpipm && \
    cd /home/${USER}/blasfeo && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DBLASFEO_EXAMPLES=OFF && \
    make -j8 && sudo make install -j && cd /home/${USER}/hpipm && mkdir build && \
    cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DHPIPM_TESTING=OFF && \
    make -j8 && sudo make install -j && cd /home/${USER} && \
    git clone https://github.com/Zolkin1/hpipm-cpp.git && cd /home/${USER}/hpipm-cpp && mkdir build && \
    cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j8 && sudo make install -j 

# you must run the setup script from a directory where the user has permissions
# run docker setup script in Dockerfile
WORKDIR /home/${USER}
RUN source /home/${USER}/obelisk/setup.sh --install-sys-deps --cyclone-perf --obk-aliases --basic

WORKDIR /home/${USER}

# Python Packages
RUN pip install scipy && \
    pip install osqp
