# Choose the initial Docker image (noetic base is enough for iG-LIO)
FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_VERSION=noetic
ENV ROS_WS /rustle/catkin_ws

### METADATA ###
LABEL author="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL maintainer="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL description="DLIO Environment for Rustle"
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
RUN git clone --depth 1 --recurse-submodules https://github.com/vectr-ucla/direct_lidar_inertial_odometry.git src/direct_lidar_inertial_odometry
### CLONE SLAM ALGORITHM

### CLONE ANOTHER VERSION TO REDUCE THE QUEUE SIZE TO 1 FOR THE SPEED TEST
#RUN git clone --depth 1 --recurse-submodules https://github.com/vectr-ucla/direct_lidar_inertial_odometry.git src/direct_lidar_inertial_odometry_speed

#Replaces their subscriber queue to 1
#RUN sed -i '31s/100/1/g' src/direct_lidar_inertial_odometry_speed/src/dlio/odom.cc
#RUN sed -i '79s|<direct_lidar_inertial_odometry/save_pcd.h>|<direct_lidar_inertial_odometry_speed/save_pcd.h>|' src/direct_lidar_inertial_odometry_speed/include/dlio/dlio.h
#RUN sed -i 's/^project(direct_lidar_inertial_odometry)$/project(direct_lidar_inertial_odometry_speed)/' src/direct_lidar_inertial_odometry_speed/CMakeLists.txt
#RUN sed -i 's/<name>direct_lidar_inertial_odometry<\/name>/<name>direct_lidar_inertial_odometry_speed<\/name>/g' src/direct_lidar_inertial_odometry_speed/package.xml


# Build ros package source (compiles every package in the workspace, don't change this!)
RUN catkin config --extend /opt/ros/$ROS_DISTRO && catkin create pkg -p src/ rustle

RUN git clone --depth 1 --recurse-submodules https://github.com/mjpc13/rustle-ros.git  src/rustle-ros

#Copy custom launch file into our package
COPY ./rustle.launch /rustle/catkin_ws/src/rustle-ros/launch/rustle.launch

#Compile packages
RUN catkin build -s -j 2

# source catkin workspace
RUN echo "source /rustle/catkin_ws/devel/setup.bash" >> /root/.profile