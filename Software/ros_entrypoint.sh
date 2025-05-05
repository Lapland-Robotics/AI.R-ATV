#!/bin/bash
set -e

source /opt/ros/$ROS_DISTRO/setup.bash
source /app/ros2_ws/install/setup.bash

echo "Welcome to AI.R Autonomous Robot Docker container"

exec "$@"
