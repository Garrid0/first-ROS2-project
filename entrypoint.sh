#!/bin/bash
set -e

# Source the ROS2 environment
source /opt/ros/humble/setup.bash

# Source the workspace
source /ros2_ws/install/setup.bash

# Execute the principle command
exec "$@"
