FROM ubuntu:22.04

# **************************************************************************** #

ENV PROJECT_NAME multi-drone-core

# set work directory
ENV WORKSPACE /workspace/src
RUN mkdir -p $WORKSPACE/$PROJECT_NAME
WORKDIR $WORKSPACE

# **************************************************************************** #

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    mesa-utils \
    x11-apps \
    libgl1 \
    libegl1 \
    && rm -rf /var/lib/apt/lists/*

# **************************************************************************** #

# Обновляем пакеты и устанавливаем необходимые утилиты
RUN apt-get update && apt-get install -y \
    locales \
    software-properties-common \
    curl \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Устанавливаем язык и добавляем репозитории
RUN apt-get update && \
    add-apt-repository universe && \
    apt-get update

# Development tools
RUN apt-get update && apt-get install -y \
    cmake \
    build-essential \
    gnome-terminal \
    dbus-x11 \
    wget \
    vim \
    git \
    sudo \
    lsb-release \
    gnupg \
    && rm -rf /var/lib/apt/lists/*

# **************************************************************************** #

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt-get update && \
    apt-get install -y gz-harmonic

# **************************************************************************** #

RUN apt-get update && apt-get install -y \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# Install PX4 packages
RUN git clone https://github.com/PX4/PX4-Autopilot.git

RUN cd ./PX4-Autopilot && \
    git checkout v1.16.1 && \
    git submodule sync --recursive && \
    git submodule update --init --recursive

RUN sed -i 's/\bsudo\b//g' ./PX4-Autopilot/Tools/setup/ubuntu.sh && \
    ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools

RUN cd PX4-Autopilot && make px4_sitl

# **************************************************************************** #

RUN apt-get update && apt-get install -y \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    python3-gi \
    python3-gst-1.0 \
    libfuse2 \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libxcb-cursor-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt

RUN wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl-x86_64.AppImage && \
    chmod +x QGroundControl-x86_64.AppImage

# **************************************************************************** #

COPY . $WORKSPACE/$PROJECT_NAME

# **************************************************************************** #

# Install Python 3 and pip
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# install python dependencies
COPY requirements.txt /tmp/requirements.txt

RUN pip install --upgrade pip && \
    pip install -r /tmp/requirements.txt  && \
    rm -rf /root/.cache/pip

# **************************************************************************** #

WORKDIR /workspace

RUN echo "alias qgc='/opt/QGroundControl-x86_64.AppImage'" >> ~/.bashrc
RUN echo "export PATH=\$PATH:/workspace/src/PX4-Autopilot/build/px4_sitl_default/bin" >> ~/.bashrc