#!/bin/bash

# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export CYCLONEDDS_URI=file://$HOME/repo/AI.R-ATV/Software/cyclonedds-config.xml

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CONFIG_URI=$HOME/repo/AI.R-ATV/Software/zenoh-router-config.json5

source /opt/ros/humble/setup.bash
source $HOME/ros2_zenoh/install/setup.bash
