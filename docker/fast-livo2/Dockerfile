FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/rustle/catkin_ws

### METADATA ###
LABEL author="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL maintainer="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL description="FAST-LIVO2 Environment for Rustle"
### END METADATA ###


### INSTALL NON CATKIN DEPENDENCIES ###
RUN apt-get update \
  && apt-get install -y git \
    libeigen3-dev \
    libpcl-dev \
    libopencv-dev \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-pcl-conversions \
    ros-$ROS_DISTRO-eigen-conversions \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-image-transport \
    ros-${ROS_DISTRO}-rosbridge-server \
    python3-catkin-tools \
  && rm -rf /var/lib/apt/lists/* \
  && mkdir -p /rustle/catkin_ws/src


# Clone and build Sophus
RUN git clone https://github.com/strasdat/Sophus.git && cd Sophus \
  && git checkout a621ff2e \
  && sed -i -E \
    -e 's/unit_complex_\.real\(\) = ([0-9.]+);/unit_complex_.real(\1);/' \
    -e 's/unit_complex_\.imag\(\) = ([0-9.]+);/unit_complex_.imag(\1);/' \
    /Sophus/sophus/so2.cpp \
  && mkdir build && cd build && cmake .. \
  && make install


### END INSTALL DEPENDENCIES ###

RUN mkdir -p /rustle/dataset/ && mkdir -p /rustle/config/
SHELL ["/bin/bash", "-l", "-c"]
WORKDIR ${ROS_WS}


### CLONE SLAM ALGORITHM
#Clone only latest commit, initialize and update nested submodules (if any)
RUN git clone --depth 1 --recurse-submodules https://github.com/hku-mars/FAST-LIVO2 src/FAST-LIVO2 \
    && git clone --depth 1 --recurse-submodules https://github.com/Livox-SDK/livox_ros_driver src/livox_ros_driver \
    &&  git clone https://github.com/xuankuzcr/rpg_vikit.git src/rpg_vikit
### CLONE SLAM ALGORITHM


### CLONE ANOTHER VERSION TO REDUCE THE QUEUE SIZE TO 1 FOR THE SPEED TEST
RUN git clone --depth 1 --recurse-submodules https://github.com/hku-mars/FAST-LIVO2 src/FAST-LIVO2_speed

#Replaces their subscriber queue to 1
RUN sed -i '190s/200000/1/g;191s/200000/1/g;192s/200000/1/g;193s/200000/1/g;' src/FAST-LIVO2_speed/src/LIVMapper.cpp
#Change project name
RUN sed -i 's/^project(fast_livo)$/project(fast_livo_speed)/' src/FAST-LIVO2_speed/CMakeLists.txt
RUN sed -i 's/<name>fast_livo<\/name>/<name>fast_livo_speed<\/name>/g' src/FAST-LIVO2_speed/package.xml


# # Build ros package source (compiles every package in the workspace, don't change this!)
RUN catkin config --extend /opt/ros/$ROS_DISTRO

RUN git clone --depth 1 --recurse-submodules https://github.com/mjpc13/rustle-ros.git  src/rustle-ros

#Copy custom launch file into our package
COPY ./rustle.launch /rustle/catkin_ws/src/rustle-ros/launch/rustle.launch

#Compile packages
RUN catkin build -s -j2

# source catkin workspace
RUN echo "source /rustle/catkin_ws/devel/setup.bash" >> /root/.profile