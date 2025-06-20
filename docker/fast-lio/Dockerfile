# Choose the initial Docker image (noetic base is enough for iG-LIO)
FROM ros:melodic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS /rustle/catkin_ws

### METADATA ###
LABEL author="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL maintainer="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL description="iG-LIO Environment for Rustle"
### END METADATA ###


### INSTALL DEPENDENCIES ###
RUN apt-get update \
  && apt-get install -y curl git \
  && apt-get update \
  && apt-get install -y \
  libeigen3-dev \
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
RUN git clone --depth 1 --recurse-submodules https://github.com/hku-mars/FAST_LIO.git src/FAST_LIO \
    && git clone --depth 1 --recurse-submodules https://github.com/Livox-SDK/livox_ros_driver src/livox_ros_driver
### CLONE SLAM ALGORITHM

### CLONE ANOTHER VERSION TO REDUCE THE QUEUE SIZE TO 1 FOR THE SPEED TEST
RUN git clone --depth 1 --recurse-submodules https://github.com/hku-mars/FAST_LIO.git src/FAST_LIO_speed

#Replaces their subscriber queue to 1
RUN sed -i '846s/200000/1/g;847s/200000/1/g;848s/200000/1/g' src/FAST_LIO_speed/src/laserMapping.cpp
RUN sed -i 's/^project(fast_lio)$/project(fast_lio_speed)/' src/FAST_LIO_speed/CMakeLists.txt
RUN sed -i 's/fast_lio/fast_lio_speed/g' src/FAST_LIO_speed/include/common_lib.h

RUN sed -i 's/<name>fast_lio<\/name>/<name>fast_lio_speed<\/name>/g' src/FAST_LIO_speed/package.xml


# Build ros package source (compiles every package in the workspace, don't change this!)
RUN catkin config --extend /opt/ros/$ROS_DISTRO && catkin create pkg -p src/ rustle

#Copy custom launch file into our package
COPY ./rustle.launch /rustle/catkin_ws/src/rustle-ros/launch/rustle.launch

#Compile packages
RUN catkin build -s

# source catkin workspace
RUN echo "source /rustle/catkin_ws/devel/setup.bash" >> /root/.profile