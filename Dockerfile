FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Sao_Paulo

ENV http_proxy $HTTP_PROXY
ENV https_proxy $HTTP_PROXY

ARG PYTHON_VERSION=3.6

# ADD . /Letmein_ws
# # WORKDIR  /Letmein_ws

# Needed for string substitution
SHELL ["/bin/bash", "-c"]

COPY requirements.txt /
# COPY install_ros.sh /
WORKDIR  /

# Install all dependencies for OpenCV
RUN apt-get update && \
    apt-get install -y --no-install-recommends software-properties-common && \
    add-apt-repository ppa:deadsnakes/ppa

# Install all dependencies for OpenCV
RUN apt-get -y update -qq --fix-missing && \
    apt-get -y install --no-install-recommends \
        wget \
        python3 \
        python3-dev \
        $( [ ${PYTHON_VERSION%%.*} -ge 3 ] && echo "python${PYTHON_VERSION%%.*}-distutils" ) \
        xcb \
        x11-apps \
        net-tools \
        iputils-ping \
        curl \
        tmux \
        vim \
        htop \
        nano \
        ffmpeg \
        libtbb2 \
        apt-utils \
        qt5-default \
        libopenblas-base \
        libpng-dev \
        libgstreamer1.0 \
        libgstreamer-plugins-base1.0-0 \
        libsm6 \
        libxext6 \
        libxrender1 \
        ca-certificates \
        debian-keyring \
        debian-archive-keyring \
        git \
        repo \
        build-essential \
        autoconf \
        libtool \
        libavahi-client-dev \
        libavcodec-dev \
        libavformat-dev \
        libswscale-dev \
        libncurses5-dev \
        mplayer \
        cpio 

# install python dependencies
RUN wget https://bootstrap.pypa.io/get-pip.py --progress=bar:force:noscroll --no-check-certificate && \
    python3 get-pip.py && \
    rm get-pip.py && \
    apt-get -y install python-dev python-pip \
        python3-dev python3-pip

# Install python2 dependencies
RUN pip2 install --upgrade pip && \
    pip2 install -r requirements.txt
    
# Install python3 dependencies
RUN pip3 install --upgrade pip && \
    pip3 install -r requirements.txt

# cleaning
RUN apt-get autoremove -y && \
    apt-get clean


ENV PYTHONIOENCODING=UTF-8


# update ros repository
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -

RUN apt-get -y update -qq --fix-missing && \
    apt-get -y install ros-melodic-desktop \
    ros-melodic-joy \
    ros-melodic-octomap-ros \
    ros-melodic-aruco \
    python3-wstool \
    python3-catkin-tools

RUN echo "source /opt/ros/melodic/setup.bash" | tee -a /root/.bashrc