FROM ros:noetic-ros-core as stage1

LABEL author="Christian Brommer <christian.brommer@aau.at>"
LABEL author="Martin Scheiber <martin.scheiber@aau.at>"
LABEL maintainer="Christian Brommer <christian.brommer@aau.at>"
LABEL description="Mars Ros Wrapper Test Environment"

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install --no-install-recommends -y \
  vim \
  git \
  wget \
  doxygen \
  graphviz \
  build-essential \
  libeigen3-dev \
  python3-catkin-tools \
  ros-noetic-dynamic-reconfigure \
  ros-noetic-tf2-ros \
  && rm -rf /var/lib/apt/lists/* \
  && mkdir -p /rustle/catkin_ws/src

WORKDIR /rustle/catkin_ws

RUN /bin/bash -c "source /opt/ros/noetic/setup.sh; rosdep init && rosdep update; \
  source /opt/ros/noetic/setup.sh; catkin init; catkin config --extend /opt/ros/noetic; \
  source /opt/ros/noetic/setup.sh;"

RUN git clone https://github.com/aau-cns/mars_ros.git src/mars_ros && catkin build;

# flatten image
FROM scratch as final
COPY --from=stage1 / /

ENTRYPOINT ["/bin/bash"]
CMD /docker_application_test.sh
