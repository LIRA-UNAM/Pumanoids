#!/bin/bash
echo "[START GAMECONTROL]"

cd `dirname $0`
cd ..

source ./install/setup.bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
export FASTDDS_DEFAULT_PROFILES_FILE=./configs/fastdds.xml

ros2 launch game_controller launch.py
