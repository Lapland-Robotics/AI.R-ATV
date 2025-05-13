#!/bin/bash

set -e # exit immediately on any command failure

source /home/robotics/repo/AI.R-ATV/Software/systemctls/scripts/ros2_config.bash
source /home/robotics/repo/AI.R-ATV/Software/install/setup.bash

ros2 launch data_collector data_collector.launch.py
