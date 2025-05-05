#!/bin/bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source /opt/ros/humble/setup.bash
source /home/robotics/repo/AI.R-ATV/Software/install/setup.bash

ros2 launch data_collector data_collector.launch.py
