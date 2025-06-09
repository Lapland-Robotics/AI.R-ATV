#!/bin/bash

# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export CYCLONEDDS_URI=file:///home/sohan-lapinamk/repos/Autonomous_Robot/Software/cyclonedds-config.xml

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_ROUTER_CONFIG_URI=/home/sohan-lapinamk/repos/Autonomous_Robot/Software/zenoh-router-config.json5

source /opt/ros/humble/setup.bash
source /home/sohan-lapinamk/ros2_zenoh/install/setup.bash