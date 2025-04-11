#!/bin/bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source /opt/ros/humble/setup.bash
source /home/robotics/ouster_lider_ros2_ws/install/setup.bash

ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=os1-122030000368.local ouster_ns:=ouster viz:=false timestamp_mode:="TIME_FROM_ROS_TIME"
