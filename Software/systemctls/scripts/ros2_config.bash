#!/bin/bash

export AIR_AR_PATH=/home/sohan-lapinamk/repos/Autonomous_Robot

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CONFIG_URI=$AIR_AR_PATH/Software/zenoh-router-config.json5
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export CYCLONEDDS_URI=file://$HOME/repo/AI.R-ATV/Software/cyclonedds-config.xml

source /opt/ros/humble/setup.bash
source $HOME/ros2_zenoh/install/setup.bash
