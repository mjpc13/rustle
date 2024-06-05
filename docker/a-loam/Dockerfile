FROM ros:kinetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS /rustle/catkin_ws

LABEL author="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL maintainer="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL description="A-LOAM Environment for Rustle"

#Install dependencies
RUN apt-get update \
  && apt-get install -y curl git \
  && apt-get update \
  && apt-get install -y ros-$ROS_DISTRO-navigation \
  libatlas-base-dev \
  libeigen3-dev \
  libgoogle-glog-dev \
  libsuitesparse-dev \
  ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-image-transport \
  ros-${ROS_DISTRO}-message-filters \
  ros-${ROS_DISTRO}-tf \
  python-catkin-tools \
  && rm -rf /var/lib/apt/lists/* \
  && mkdir -p /rustle/catkin_ws/src

# Install GTSAM (a non ROS dependency)
RUN git clone https://ceres-solver.googlesource.com/ceres-solver && cd ceres-solver \
    && git checkout 1.12.0 \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j4 \
    && make install

RUN mkdir -p /rustle/dataset/ && mkdir -p /rustle/config/
SHELL ["/bin/bash", "-l", "-c"]
WORKDIR ${ROS_WS}

#Clone only latest commit, initialize and update nested submodules 
RUN git clone --depth 1 --recurse-submodules https://github.com/HKUST-Aerial-Robotics/A-LOAM.git src/A-LOAM

# Build ros package source
RUN catkin config --extend /opt/ros/$ROS_DISTRO && catkin create pkg -p src/ rustle

#Copy custom launch file into our package
COPY ./rustle.launch /rustle/catkin_ws/src/rustle/launch/rustle.launch

#Compile packages
RUN catkin build -s

# source catkin workspace
RUN echo "source /rustle/catkin_ws/devel/setup.bash" >> /root/.profile