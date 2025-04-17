FROM nvidia/opengl:1.2-glvnd-devel-ubuntu20.04

###### Set environment variable
ENV DISPLAY=:0
ENV ROS_DISTRO=noetic
ENV LIBGL_ALWAYS_INDIRECT=0
ARG CMAKE_VERSION=3.24.4

ENV DEBIAN_FRONTEND="noninteractive"
ENV TZ="UTC"

###### Install dependencies
RUN apt-get update && apt-get install -y \
    lsb-release \
    curl \
    gnupg2 \
    python-is-python3 \
    python3-pip \
    git \
    gedit \
    terminator \
    xorg-dev \
    libxcb-shm0 \
    libglu1-mesa-dev \
    python3-dev \
    libc++-dev \
    libc++abi-dev \
    libsdl2-dev \
    libxi-dev \
    libtbb-dev \
    libosmesa6-dev \
    libudev-dev \
    autoconf \
    libtool

###### Install Open3D
WORKDIR /tmp
RUN git clone https://github.com/isl-org/Open3D.git

###### Install CMAKE
RUN apt-get remove --purge --auto-remove -y cmake

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        wget \
        gnupg2 \
        lsb-release && \
    rm -rf /var/lib/apt/lists/*

RUN apt update
RUN apt install -y software-properties-common lsb-release
RUN apt clean all

WORKDIR /tmp
RUN apt-get update && apt-get install -y --no-install-recommends \
        wget tar gzip && \
    wget https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-linux-x86_64.tar.gz && \
    tar -xzf cmake-${CMAKE_VERSION}-linux-x86_64.tar.gz && \
    mv cmake-${CMAKE_VERSION}-linux-x86_64 /opt/cmake && \
    ln -s /opt/cmake/bin/cmake /usr/local/bin/cmake && \
    rm cmake-${CMAKE_VERSION}-linux-x86_64.tar.gz

WORKDIR /tmp/Open3D
RUN mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

###### Install ROS
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop-full

RUN apt-get install -yy \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    python3-catkin-tools

RUN rosdep init
RUN rosdep update

###### Install necessary ROS packages
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-xacro

###### Source the ROS setup.bash
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

####### Install ROS Packages
RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws/src
RUN git clone https://github.com/moveit/moveit_msgs.git
RUN git clone https://github.com/wg-perception/object_recognition_msgs.git
RUN git clone https://github.com/OctoMap/octomap_msgs.git
RUN git clone -b master https://github.com/ros/eigen_stl_containers.git

####### Install ROS Packages
RUN pip install open3d
RUN pip install --upgrade numpy

###### Create a workspace
WORKDIR /ros_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

###### Set ROS environment variables: Tiago wired
RUN echo "#export ROS_MASTER_URI=http://10.68.0.1:11311" >> /root/.bashrc
RUN echo "#export ROS_IP=10.68.0.128" >> /root/.bashrc
