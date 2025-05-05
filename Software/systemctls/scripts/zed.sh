#!/bin/bash

set -e # exit immediately on any command failure
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source /opt/ros/humble/setup.bash
source /home/robotics/zed2_ros2_ws/install/setup.bash

ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2 publish_tf:=false
