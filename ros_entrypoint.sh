#!/bin/bash
set -e

source /opt/ros/$ROS_DISTRO/setup.bash
source /app/ros2_ws/install/setup.bash
source /uros_ws/install/local_setup.sh

ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 > /app/micro-ros-agent.log &

exec "$@"
