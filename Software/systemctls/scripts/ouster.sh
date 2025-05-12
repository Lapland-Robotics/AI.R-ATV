#!/bin/bash

set -e # exit immediately on any command failure

source ros2_config.bash
source /home/robotics/ouster_lider_ros2_ws/install/setup.bash

ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=os1-122030000368.local ouster_ns:=ouster viz:=false timestamp_mode:="TIME_FROM_ROS_TIME"

# source /home/robotics/laser_odom_ws/install/setup.bash
# ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py
