#!/bin/bash

set -e # exit immediately on any command failure

source $AIR_AR_PATH/Software/systemctls/scripts/ros2_config.bash
source $AIR_AR_PATH/Software/install/setup.bash

ros2 launch data_collector data_collector.launch.py
