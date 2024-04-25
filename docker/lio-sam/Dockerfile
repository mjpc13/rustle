FROM ros:kinetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS /rustle/catkin_ws

LABEL author="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL maintainer="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL description="LIO-SAM Environment for Rustle"

#Install dependencies
RUN apt-get update \
  && apt-get install -y curl git \
  && apt-get update \
  && apt-get install -y ros-$ROS_DISTRO-navigation \
  ros-$ROS_DISTRO-robot-localization \
  ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-robot-state-publisher \
  ros-$ROS_DISTRO-xacro \
  python-catkin-tools \
  && rm -rf /var/lib/apt/lists/* \
  && mkdir -p /rustle/catkin_ws/src

# Install GTSAM (a non ROS dependency)
RUN apt-get update \
  && apt install -y software-properties-common \
  && add-apt-repository -y ppa:borglab/gtsam-release-4.0 \
  && apt-get update \
  && apt install -y libgtsam-dev libgtsam-unstable-dev \
  && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /rustle/dataset/ && mkdir -p /rustle/config/
SHELL ["/bin/bash", "-l", "-c"]
WORKDIR ${ROS_WS}

#Clone only latest commit, initialize and update nested submodules 
RUN git clone --depth 1 --recurse-submodules https://github.com/TixiaoShan/LIO-SAM.git src/LIO-SAM

# Build ros package source
RUN catkin config --extend /opt/ros/$ROS_DISTRO && catkin create pkg -p src/ rustle

#Copy custom launch file into our package
COPY ./rustle.launch /rustle/catkin_ws/src/rustle/launch/rustle.launch

#Compile packages
RUN catkin build -s

# source catkin workspace
RUN echo "source /rustle/catkin_ws/devel/setup.bash" >> /root/.profile