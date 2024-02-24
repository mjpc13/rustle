#!/bin/bash
set -e

# Source ROS environment
source "/rustle/catkin_ws/devel/setup.bash" --
exec "$@"
