#!/bin/bash

set -e # exit immediately on any command failure

source /home/robotics/repos/AI.R-Autonomous_Robot/Software/systemctls/scripts/ros2_config.bash
source /home/robotics/repos/AI.R-Autonomous_Robot/Software/drivers/install/setup.bash

ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2 publish_tf:=false
