FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

# Install essential compilation and git dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Set up a clean colcon workspace overlay
WORKDIR /ros2_ws/src

# Clone the PRBonn repository (colcon will automatically find the ros/ subdirectory package)
RUN git clone https://github.com/PRBonn/kiss-icp

# Compile the ROS2 C++ wrapper package
WORKDIR /ros2_ws

RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Update the core entrypoint script so the workspace is sourced automatically upon container execution
RUN printf '#!/bin/bash\nset -e\nsource "/opt/ros/jazzy/setup.bash"\nsource "/ros2_ws/install/setup.bash"\nexec "$@"\n' > /ros_entrypoint.sh \
    && chmod +x /ros_entrypoint.sh

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
