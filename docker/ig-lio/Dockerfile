# Choose the initial Docker image (noetic base is enough for iG-LIO)
FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_VERSION=noetic
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
  libgoogle-glog-dev \
  inxi \ 
  libtbb-dev \
  ros-$ROS_DISTRO-pcl-ros \
  ros-$ROS_DISTRO-topic-tools \
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
RUN git clone --depth 1 --recurse-submodules https://github.com/zijiechenrobotics/ig_lio.git src/ig_lio \
    && git clone --depth 1 --recurse-submodules https://github.com/Livox-SDK/livox_ros_driver src/livox_ros_driver
### CLONE SLAM ALGORITHM


### CLONE ANOTHER VERSION TO REDUCE THE QUEUE SIZE TO 1 FOR THE SPEED TEST
RUN git clone --depth 1 --recurse-submodules https://github.com/zijiechenrobotics/ig_lio.git src/ig_lio_speed

#Replaces their subscriber queue to 1
RUN sed -i '549s/10000/1/g;566s/10000/1/g;568s/10000/1/g' src/ig_lio_speed/src/ig_lio_node.cpp
RUN sed -i 's/^project(ig_lio)$/project(ig_lio_speed)/' src/ig_lio_speed/CMakeLists.txt
RUN sed -i 's/<name>ig_lio<\/name>/<name>ig_lio_speed<\/name>/g' src/ig_lio_speed/package.xml


# Build ros package source (compiles every package in the workspace, don't change this!)
RUN catkin config --extend /opt/ros/$ROS_DISTRO && catkin create pkg -p src/ rustle

RUN git clone --depth 1 --recurse-submodules https://github.com/mjpc13/rustle-ros.git  src/rustle-ros


#Copy custom launch file into our package
COPY ./rustle.launch /rustle/catkin_ws/src/rustle-ros/launch/rustle.launch

#Compile packages
RUN catkin build -s

# source catkin workspace
RUN echo "source /rustle/catkin_ws/devel/setup.bash" >> /root/.profile