FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Sao_Paulo

ENV http_proxy $HTTP_PROXY
ENV https_proxy $HTTP_PROXY
ENV FLASK_ENV=development

ARG INSTALL_DIR=/opt/intel/openvino_2021
ARG TEMP_DIR=/tmp/openvino_installer

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    repo \
    build-essential \
    autoconf \
    libtool  \
    libavahi-client-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libncurses5-dev \
    mplayer \
    wget \
    x11-apps \
    net-tools \
    iputils-ping \
    curl \
    tmux \
    vim \
    htop \
    nano \
    ca-certificates \
    cpio \
    sudo \
    lsb-release && \
    rm -rf /var/lib/apt/lists/*

# RUN mkdir -p $TEMP_DIR && cd $TEMP_DIR

# RUN cd $TEMP_DIR && \
#     curl -sc /tmp/cookie "https://drive.google.com/uc?export=download&id=1qsXBl66EankeF7aczjBAkC4DqLRXf9O5" > /dev/null && \
#     curl -Lb /tmp/cookie "https://drive.google.com/uc?export=download&confirm=$(awk '/_warning_/ {print $NF}' /tmp/cookie)&id=1qsXBl66EankeF7aczjBAkC4DqLRXf9O5" -o l_openvino_toolkit_p_2021.2.185.tgz

# RUN cd $TEMP_DIR && tar xf l_openvino_toolkit_p_2021.2.185.tgz && \
#     cd l_openvino_toolkit_p_2021.2.185 && \
#     sed -i 's/decline/accept/g' silent.cfg && \
#     ./install.sh -s silent.cfg && \
#     rm -rf $TEMP_DIR

#COPY 6379.conf /etc/redis/redis.conf
COPY requirements.txt /
WORKDIR  /

# Install python dependencies
RUN apt-get -y install python3-dev \
    python3-pip && \
    pip3 install --upgrade pip && \
    pip3 install -r requirements.txt


# RUN apt-get update && apt-get install -y --no-install-recommends \
#     git \
#     repo \
#     build-essential \
#     autoconf \
#     libtool  \
#     libavahi-client-dev \
#     libavcodec-dev \
#     libavformat-dev \
#     libswscale-dev \
#     libncurses5-dev \
#     mplayer \
#     wget \
#     x11-apps \
#     net-tools \
#     iputils-ping \
#     curl \
#     tmux \
#     vim \
#     htop \
#     nano \
#     ca-certificates \
#     cpio \
#     sudo \
#     lsb-release && \
#     rm -rf /var/lib/apt/lists/*


ENV PYTHONIOENCODING=UTF-8

# update ros repository
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
RUN apt-get update

# Desktop Install: ROS, rqt, rviz, and robot-generic libraries
RUN apt install -y ros-melodic-desktop

RUN echo "source /opt/ros/melodic/setup.bash" | tee -a /root/.bashrc
RUN echo "source ~/.bashrc"

# enables you to easily download many source trees for ROS packages with one command
RUN apt-get install -y python-rosinstall rosdep python-rosinstall-generator python-wstool

# Initialise rosdep
RUN rosdep init
RUN rosdep update


