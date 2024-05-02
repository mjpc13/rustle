# Choose the initial Docker image (noetic base is enough for iG-LIO)
FROM ros:melodic-ros-base


ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS /rustle/catkin_ws

### METADATA ###
LABEL author="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL maintainer="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL description="LVI-SAM Environment for Rustle"
### END METADATA ###


### INSTALL DEPENDENCIES ###
RUN apt-get update \
  && apt-get install -y curl git wget \
  libgoogle-glog-dev \
  libatlas-base-dev \
  libeigen3-dev \
  libpcl-dev \
  libsuitesparse-dev \
  libtbb-dev \
  libglew-dev \
  python-catkin-tools \
  ros-$ROS_DISTRO-pcl-ros \
  ros-$ROS_DISTRO-pcl-conversions \
  ros-$ROS_DISTRO-tf \
  ros-$ROS_DISTRO-cv-bridge \
  ros-$ROS_DISTRO-image-transport \
  ros-${ROS_DISTRO}-image-transport-plugins \
  && rm -rf /var/lib/apt/lists/* \
  && mkdir -p /rustle/catkin_ws/src

# Install ceres-solver
WORKDIR /home/thirdParty
RUN wget https://github.com/ceres-solver/ceres-solver/archive/refs/tags/1.14.0.tar.gz
RUN tar zxf 1.14.0.tar.gz
WORKDIR /home/thirdParty/ceres-solver-1.14.0

RUN mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release ../../ceres-solver-1.14.0 && make -j8 && make install

# Install GTSAM (a non ROS dependency)
RUN apt-get update \
  && apt install -y software-properties-common \
  && add-apt-repository -y ppa:borglab/gtsam-release-4.2 \
  && apt-get update \
  && apt install -y libgtsam-dev libgtsam-unstable-dev \
  && rm -rf /var/lib/apt/lists/*


### END INSTALL DEPENDENCIES ###

RUN mkdir -p /rustle/dataset/ && mkdir -p /rustle/config/
SHELL ["/bin/bash", "-l", "-c"]
WORKDIR ${ROS_WS}


### CLONE SLAM ALGORITHM
#Clone only latest commit, initialize and update nested submodules (if any)
RUN git clone --depth 1 --recurse-submodules https://github.com/TixiaoShan/LVI-SAM.git src/LVI-SAM
### CLONE SLAM ALGORITHM


# Build ros package source (compiles every package in the workspace, don't change this!)
RUN catkin config --extend /opt/ros/$ROS_DISTRO && catkin create pkg -p src/ rustle

#Copy custom launch file into our package
COPY ./rustle.launch /rustle/catkin_ws/src/rustle/launch/rustle.launch

#Compile packages
RUN catkin build -s

# source catkin workspace
RUN echo "source /rustle/catkin_ws/devel/setup.bash" >> /root/.profile