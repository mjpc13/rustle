# Choose the initial Docker image (noetic base is enough for iG-LIO)
FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_VERSION=noetic
ENV ROS_WS /rustle/catkin_ws

### METADATA ###
LABEL author="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL maintainer="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL description="DLO Environment for Rustle"
### END METADATA ###


### INSTALL DEPENDENCIES ###
RUN apt-get update \
  && apt-get install -y curl git \
  && apt-get update \
  && apt-get install -y \
  libeigen3-dev \
  libomp-dev \
  libpcl-dev \
  ros-$ROS_DISTRO-pcl-ros \
  ros-$ROS_DISTRO-pcl-conversions \
  ros-$ROS_DISTRO-eigen-conversions \
  python3-catkin-tools \
  && rm -rf /var/lib/apt/lists/* \
  && mkdir -p /rustle/catkin_ws/src

### END INSTALL DEPENDENCIES ###
RUN mkdir -p /rustle/dataset/ && mkdir -p /rustle/config/
SHELL ["/bin/bash", "-l", "-c"]
WORKDIR ${ROS_WS}


### CLONE SLAM ALGORITHM
#Clone only latest commit, initialize and update nested submodules (if any)
RUN git clone --depth 1 --recurse-submodules https://github.com/vectr-ucla/direct_lidar_odometry.git src/direct_lidar_odometry
### CLONE SLAM ALGORITHM


# Build ros package source (compiles every package in the workspace, don't change this!)
RUN catkin config --extend /opt/ros/$ROS_DISTRO && catkin create pkg -p src/ rustle

#Copy custom launch file into our package
COPY ./rustle.launch /rustle/catkin_ws/src/rustle/launch/rustle.launch

#Compile packages
RUN catkin build -s

# source catkin workspace
RUN echo "source /rustle/catkin_ws/devel/setup.bash" >> /root/.profile