#!/bin/bash
set -e

source /opt/ros/$ROS_DISTRO/setup.bash
source /app/ros2_ws/install/setup.bash
source /uros_ws/install/local_setup.sh

# ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 > /app/micro-ros-agent.log &
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 > /app/micro-ros-agent.log &

echo "Welcome to AI.R ATV Docker container"

exec "$@"
