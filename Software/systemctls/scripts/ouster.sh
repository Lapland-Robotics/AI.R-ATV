#!/bin/bash

set -e # exit immediately on any command failure

source /home/robotics/repos/AI.R-Autonomous_Robot/Software/systemctls/scripts/ros2_config.bash
source /home/robotics/repos/AI.R-Autonomous_Robot/Software/drivers/install/setup.bash

ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=os1-122030000368.local ouster_ns:=ouster viz:=false timestamp_mode:="TIME_FROM_ROS_TIME"
