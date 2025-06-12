# Choose the initial Docker image (noetic base is enough for iG-LIO)
FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_VERSION=noetic
ENV ROS_WS /rustle/catkin_ws

### METADATA ###
LABEL author="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL maintainer="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL description="Point-LIO Environment for Rustle"
### END METADATA ###


### INSTALL DEPENDENCIES ###
RUN apt-get update \
  && apt-get install -y curl git \
  && apt-get update \
  && apt-get install -y \
  libeigen3-dev \
  libgoogle-glog-dev \
  ros-$ROS_DISTRO-pcl-ros \
  ros-$ROS_DISTRO-pcl-conversions \
  ros-$ROS_DISTRO-eigen-conversions \
  python3-catkin-tools \
  && rm -rf /var/lib/apt/lists/* \
  && mkdir -p /rustle/catkin_ws/src

RUN git clone https://github.com/Livox-SDK/Livox-SDK && cd Livox-SDK \
    && cd build \
    && cmake .. && make -j4 \
    && sudo make install

### END INSTALL DEPENDENCIES ###
RUN mkdir -p /rustle/dataset/ && mkdir -p /rustle/config/
SHELL ["/bin/bash", "-l", "-c"]
WORKDIR ${ROS_WS}


### CLONE SLAM ALGORITHM
#Clone only latest commit, initialize and update nested submodules (if any)
RUN git clone --depth 1 --recurse-submodules https://github.com/hku-mars/Point-LIO.git src/Point-LIO \
    && git clone --depth 1 --recurse-submodules https://github.com/Livox-SDK/livox_ros_driver src/livox_ros_driver
### CLONE SLAM ALGORITHM

### CLONE ANOTHER VERSION TO REDUCE THE QUEUE SIZE TO 1 FOR THE SPEED TEST
RUN git clone --depth 1 --recurse-submodules https://github.com/hku-mars/Point-LIO.git src/Point-LIO_speed

#Replaces their subscriber queue to 1
RUN sed -i '375s/200000/1/g;376s/200000/1/g;377s/200000/1/g' src/Point-LIO_speed/src/laserMapping.cpp
RUN sed -i 's/^project(point_lio)$/project(point_lio_speed)/' src/Point-LIO_speed/CMakeLists.txt
RUN sed -i 's/<name>point_lio<\/name>/<name>point_lio_speed<\/name>/g' src/Point-LIO_speed/package.xml


# Build ros package source (compiles every package in the workspace, don't change this!)
RUN catkin config --extend /opt/ros/$ROS_DISTRO && catkin create pkg -p src/ rustle

RUN git clone --depth 1 --recurse-submodules https://github.com/mjpc13/rustle-ros.git  src/rustle-ros

#Copy custom launch file into our package
COPY ./rustle.launch /rustle/catkin_ws/src/rustle/launch/rustle.launch

#Compile packages
RUN catkin build -s

# source catkin workspace
RUN echo "source /rustle/catkin_ws/devel/setup.bash" >> /root/.profile