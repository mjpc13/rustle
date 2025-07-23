# Choose the initial Docker image (noetic base is enough for iG-LIO)
FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS /rustle/catkin_ws

### METADATA ###
LABEL author="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL maintainer="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL description="Faster-LIO Environment for Rustle"
### END METADATA ###


### INSTALL DEPENDENCIES ###
RUN apt-get update \
  && apt-get install -y curl git \
  && apt-get update \
  && apt-get install -y \
  libeigen3-dev \
  libgoogle-glog-dev \
  libtbb-dev \
  libyaml-cpp-dev \
  libpcl-dev \
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
RUN git clone --depth 1 --recurse-submodules https://github.com/gaoxiang12/faster-lio.git src/faster-lio \
    && git clone --depth 1 --recurse-submodules https://github.com/Livox-SDK/livox_ros_driver src/livox_ros_driver
### CLONE SLAM ALGORITHM

### CLONE ANOTHER VERSION TO REDUCE THE QUEUE SIZE TO 1 FOR THE SPEED TEST
RUN git clone --depth 1 --recurse-submodules https://github.com/gaoxiang12/faster-lio.git src/faster-lio_speed

#Replaces their subscriber queue to 1
RUN sed -i '243s/200000/1/g;246s/200000/1/g;249s/200000/1/g' src/faster-lio_speed/src/laser_mapping.cc
#Change project name
RUN sed -i 's/^project(faster_lio)$/project(faster_lio_speed)/' src/faster-lio_speed/CMakeLists.txt
RUN sed -i 's/<name>faster_lio<\/name>/<name>faster_lio_speed<\/name>/g' src/faster-lio_speed/package.xml


#Change namespace in cpp files
RUN sed -i 's/faster_lio/faster_lio_speed/g' src/faster-lio_speed/src/laser_mapping.cc
RUN sed -i 's/faster_lio/faster_lio_speed/g' src/faster-lio_speed/src/options.cc
RUN sed -i 's/faster_lio/faster_lio_speed/g' src/faster-lio_speed/src/pointcloud_preprocess.cc
RUN sed -i 's/faster_lio/faster_lio_speed/g' src/faster-lio_speed/src/utils.cc
RUN sed -i 's/faster_lio/faster_lio_speed/g' src/faster-lio_speed/app/run_mapping_online.cc
RUN sed -i 's/faster_lio/faster_lio_speed/g' src/faster-lio_speed/app/run_mapping_offline.cc

#Change namespace in cpp headers
RUN sed -i 's/faster_lio/faster_lio_speed/g' src/faster-lio_speed/include/common_lib.h
RUN sed -i 's/faster_lio/faster_lio_speed/g' src/faster-lio_speed/include/laser_mapping.h
RUN sed -i 's/faster_lio/faster_lio_speed/g' src/faster-lio_speed/include/imu_processing.hpp
RUN sed -i 's/faster_lio/faster_lio_speed/g' src/faster-lio_speed/include/options.h
RUN sed -i 's/faster_lio/faster_lio_speed/g' src/faster-lio_speed/include/so3_math.h
RUN sed -i 's/faster_lio/faster_lio_speed/g' src/faster-lio_speed/include/use-ikfom.hpp
RUN sed -i 's/faster_lio/faster_lio_speed/g' src/faster-lio_speed/include/pointcloud_preprocess.h
RUN sed -i 's/faster_lio/faster_lio_speed/g' src/faster-lio_speed/include/utils.h
RUN sed -i 's/faster_lio/faster_lio_speed/g' src/faster-lio_speed/include/ivox3d/eigen_types.h
RUN sed -i 's/faster_lio/faster_lio_speed/g' src/faster-lio_speed/include/ivox3d/ivox3d.h
RUN sed -i 's/faster_lio/faster_lio_speed/g' src/faster-lio_speed/include/ivox3d/ivox3d_node.hpp

# Build ros package source (compiles every package in the workspace, don't change this!)
RUN catkin config --extend /opt/ros/$ROS_DISTRO

RUN git clone --depth 1 --recurse-submodules https://github.com/mjpc13/rustle-ros.git  src/rustle-ros && echo

#Copy custom launch file into our package
COPY ./rustle.launch /rustle/catkin_ws/src/rustle-ros/launch/rustle.launch

#Compile packages
RUN catkin build -s -j2

# source catkin workspace
RUN echo "source /rustle/catkin_ws/devel/setup.bash" >> /root/.profile