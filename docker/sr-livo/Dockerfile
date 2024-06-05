# Choose the initial Docker image (noetic base is enough for iG-LIO)
FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_VERSION=noetic
ENV ROS_WS /rustle/catkin_ws

### METADATA ###
LABEL author="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL maintainer="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL description="SR-LIVO Environment for Rustle"
### END METADATA ###


### INSTALL DEPENDENCIES ###
RUN apt-get update \
  && apt-get install -y curl git \
  && apt-get update \
  && apt-get install -y \
  libeigen3-dev \
  libopencv-dev \
  python3-opencv \
  libpcl-dev \
  libgoogle-glog-dev \
  libgflags-dev \
  libatlas-base-dev \
  libsuitesparse-dev \
  ros-$ROS_DISTRO-pcl-ros \
  ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-eigen-conversions \
  python3-catkin-tools \
  && rm -rf /var/lib/apt/lists/* \
  && mkdir -p /rustle/catkin_ws/src

RUN git clone https://github.com/Livox-SDK/Livox-SDK && cd Livox-SDK \
    && cd build \
    && cmake .. && make -j4 \
    && sudo make install

RUN git clone https://ceres-solver.googlesource.com/ceres-solver && cd ceres-solver \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j4 \
    && make install




### END INSTALL DEPENDENCIES ###

RUN mkdir -p /rustle/dataset/ && mkdir -p /rustle/config/
SHELL ["/bin/bash", "-l", "-c"]
WORKDIR ${ROS_WS}


### CLONE SLAM ALGORITHM
#Clone only latest commit, initialize and update nested submodules (if any)
RUN git clone --depth 1 --recurse-submodules https://github.com/ZikangYuan/sr_livo.git src/sr-livo \
    && git clone --depth 1 --recurse-submodules https://github.com/Livox-SDK/livox_ros_driver src/livox_ros_driver
### CLONE SLAM ALGORITHM


# Build ros package source (compiles every package in the workspace, don't change this!)
RUN catkin config --extend /opt/ros/$ROS_DISTRO --buildlist livox_ros_driver && catkin create pkg -p src/ rustle

#Copy custom launch file into our package
COPY ./rustle.launch /rustle/catkin_ws/src/rustle/launch/rustle.launch

#Compile packages
RUN catkin build -s
RUN catkin config --extend /opt/ros/$ROS_DISTRO --no-buildlist && catkin build -s


# source catkin workspace
RUN echo "source /rustle/catkin_ws/devel/setup.bash" >> /root/.profile