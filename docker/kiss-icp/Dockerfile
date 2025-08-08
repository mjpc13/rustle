FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/rustle/catkin_ws

### METADATA ###
LABEL author="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL maintainer="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL description="KISS-ICP Environment for Rustle"
### END METADATA ###


### INSTALL NON CATKIN DEPENDENCIES ###
RUN apt-get update \
  && apt-get install -y git \
    ros-$ROS_DISTRO-tf2 \
    ros-$ROS_DISTRO-tf2-ros \
    python3-catkin-tools \
    python3-pip \
  && rm -rf /var/lib/apt/lists/* \
  && mkdir -p /rustle/catkin_ws/src


RUN pip install kiss-icp


### END INSTALL DEPENDENCIES ###

RUN mkdir -p /rustle/dataset/ && mkdir -p /rustle/config/
SHELL ["/bin/bash", "-l", "-c"]
WORKDIR ${ROS_WS}


### CLONE SLAM ALGORITHM
#Clone only latest commit, initialize and update nested submodules (if any)
RUN git clone --recurse-submodules https://github.com/PRBonn/kiss-icp src/kiss-icp && cd src/kiss-icp && git checkout v0.3.0
### CLONE SLAM ALGORITHM

### CLONE ANOTHER VERSION TO REDUCE THE QUEUE SIZE TO 1 FOR THE SPEED TEST
RUN git clone --recurse-submodules https://github.com/PRBonn/kiss-icp src/kiss-icp_speed && cd src/kiss-icp_speed && git checkout v0.3.0

#Replaces their subscriber queue to 1
RUN sed -i '75s/queue_size_/1/g;' src/kiss-icp_speed/ros/ros1/OdometryServer.cpp
#Change project name
RUN sed -i 's/^project(kiss_icp VERSION 0.3.0 LANGUAGES CXX)$/project(kiss_icp_speed VERSION 0.3.0 LANGUAGES CXX)/' src/kiss-icp_speed/ros/CMakeLists.txt
RUN sed -i 's/<name>kiss_icp<\/name>/<name>kiss_icp_speed<\/name>/g' src/kiss-icp_speed/ros/package.xml

# Patch KISS-ICP for RUSTLE (don't hardcode topic name)
RUN sed -i '75s/("pointcloud_topic",/(pointcloud_topic_,/g;' src/kiss-icp/ros/ros1/OdometryServer.cpp
RUN sed -i '65a pnh_.param<std::string>("pointcloud_topic", pointcloud_topic_, "lidar_topic");' src/kiss-icp/ros/ros1/OdometryServer.cpp
RUN sed -i '86a std::string pointcloud_topic_;' src/kiss-icp/ros/ros1/OdometryServer.hpp

RUN sed -i '75s/("pointcloud_topic",/(pointcloud_topic_,/g;' src/kiss-icp_speed/ros/ros1/OdometryServer.cpp
RUN sed -i '65a pnh_.param<std::string>("pointcloud_topic", pointcloud_topic_, "lidar_topic");' src/kiss-icp_speed/ros/ros1/OdometryServer.cpp
RUN sed -i '86a std::string pointcloud_topic_;' src/kiss-icp_speed/ros/ros1/OdometryServer.hpp


# # Build ros package source (compiles every package in the workspace, don't change this!)
RUN catkin config --extend /opt/ros/$ROS_DISTRO

RUN git clone --depth 1 --recurse-submodules https://github.com/mjpc13/rustle-ros.git  src/rustle-ros

#Copy custom launch file into our package
COPY ./rustle.launch /rustle/catkin_ws/src/rustle-ros/launch/rustle.launch

#Compile packages
RUN catkin build -s

# source catkin workspace
RUN echo "source /rustle/catkin_ws/devel/setup.bash" >> /root/.profile