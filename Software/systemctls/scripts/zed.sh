#!/bin/bash

set -e # exit immediately on any command failure

source $AIR_AR_PATH/Software/systemctls/scripts/ros2_config.bash
source $AIR_AR_PATH/Software/drivers/src/install/setup.bash

ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2 publish_tf:=false
