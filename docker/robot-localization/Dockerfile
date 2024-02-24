FROM ros:noetic-ros-core
ENV ROS_VERSION=noetic
ENV ROS_WS /rustle/catkin_ws

LABEL author="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL maintainer="Mario Cristovao <mario.cristovao@isr.uc.pt>"
LABEL description="Robot Localization Environment for Rustle"

ENV DEBIAN_FRONTEND=noninteractive

#Install dependencies
RUN apt-get update \
  && apt-get install -y git build-essential\
  ros-$ROS_VERSION-robot-localization \
  ros-$ROS_VERSION-robot-state-publisher \
  ros-$ROS_VERSION-xacro \
  ros-$ROS_VERSION-dynamic-reconfigure \
  ros-$ROS_VERSION-tf2-ros \
  python3-catkin-tools \
  && rm -rf /var/lib/apt/lists/* \
  && mkdir -p /rustle/catkin_ws/src

SHELL ["/bin/bash", "-l", "-c"]
WORKDIR ${ROS_WS}

# Build ros package source
RUN catkin config --extend /opt/ros/$ROS_DISTRO && catkin create pkg -p src/ rustle

#Copy custom launch file into our package
COPY ./rustle.launch /rustle/catkin_ws/src/rustle/launch/rustle.launch
COPY ./entrypoint.sh /entrypoint.sh
#Compile packages
RUN catkin build -s

# source catkin workspace
RUN echo "source /rustle/catkin_ws/devel/setup.bash" >> /root/.profile && chmod +x /entrypoint.sh

# ENTRYPOINT ["/entrypoint.sh"]
CMD [ "roscore" ]
