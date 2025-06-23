#!/bin/bash

set -e # exit immediately on any command failure

source $AIR_AR_PATH/Software/systemctls/scripts/ros2_config.bash
source $AIR_AR_PATH/Software/drivers/src/install/setup.bash

ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=os1-122030000368.local ouster_ns:=ouster viz:=false timestamp_mode:="TIME_FROM_ROS_TIME"
