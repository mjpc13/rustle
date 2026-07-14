FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV ROS_WS=/rustle/colcon_ws

# FIX 1: Set shell early so 'source' works for all subsequent RUN commands
SHELL ["/bin/bash", "-c"]

### METADATA ###
LABEL author="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL maintainer="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL description="KISS-ICP Environment for Rustle (ROS 2 Jazzy)"
### END METADATA ###

### INSTALL DEPENDENCIES ###
RUN apt-get update \
  && apt-get install -y curl git build-essential inxi python3-colcon-common-extensions python3-rosdep \
  && apt-get install -y ros-${ROS_DISTRO}-rosbridge-server \
  && rm -rf /var/lib/apt/lists/* \
  && mkdir -p ${ROS_WS}/src

# Initialize rosdep database if it hasn't been initialized yet
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi

RUN mkdir -p /rustle/dataset/ && mkdir -p /rustle/config/
WORKDIR ${ROS_WS}

### CLONE SLAM ALGORITHM ###
RUN git clone --depth 1 https://github.com/PRBonn/kiss-icp.git src/kiss-icp

### CREATE RUSTLE_ROS WRAPPER LOCALLY ###
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && ros2 pkg create --build-type ament_cmake --destination-directory src rustle_ros

# Create the launch directory inside our new package and copy our file into it
RUN mkdir -p src/rustle_ros/launch
# NOTE: Ensure your host workspace file matches this exact filename (rustle.launch.py)
COPY ./rustle.launch.py src/rustle_ros/launch/rustle.launch.py

# Append the install rule to the auto-generated CMakeLists.txt so colcon knows to track the launch file
RUN echo -e "\ninstall(DIRECTORY launch DESTINATION share/\${PROJECT_NAME})" >> src/rustle_ros/CMakeLists.txt

# FIX 2: Resolve and install any hidden dependencies for KISS-ICP before building
RUN apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*

# Build the ROS 2 workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Setup entrypoint to source the new colcon workspace automatically
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.profile \
    && echo "source ${ROS_WS}/install/setup.bash" >> /root/.profile
