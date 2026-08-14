#!/bin/bash

cd `dirname $0`
cd ..
WORKSPACE_ROOT=$(pwd)
VISION_CONFIG_PATH="${WORKSPACE_ROOT}/src/vision/config"

echo "[STOP EXISTING NODES (IF ANY), TO AVOID CONFILICT]"
sudo killall -9 booster-video-stream
# sudo systemctl stop booster-rtc-speech.service
./scripts/stop.sh
sudo jetson_clocks
sudo systemctl mask apt-daily.timer apt-daily-upgrade.timer
sudo systemctl mask unattended-upgrades.service
sudo rm -f /var/lib/systemd/timers/stamp-apt-daily.timer
sudo pkill -9 update_manager
sudo pkill -9 python3
systemctl --user disable robocup_game_assist.service
systemctl stop booster-rtc-speech.service
sudo systemctl disable --now booster-agent-manager.service

echo "[START ROBOCUP NODES]"
source ./install/setup.bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
export FASTDDS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml
# export RMW_FASTRTPS_USE_SHM=0


echo "[START VISION]"
# ZED deployments may install the vendor package documented below to enable automatic startup:
# https://booster.feishu.cn/wiki/XodtwX56AiCtZtkewo3cPgbrn8d#share-MDrvdyWa2o87qixU3TccE72NnNc
# Do not apply that setup to RealSense deployments; it prevents RealSense startup and requires reinstalling daemon-perception from ~/Documents/recovery/.
# source ~/ThirdParty/zed-ros/install/setup.bash
# nohup ros2 launch zed_wrapper zed_camera.launch.py camera_model:="zed2i" > zed.log 2>&1 &
# nohup ros2 launch vision launch.py save_data:=true > vision.log 2>&1 &
nohup ros2 launch vision launch.py save_data:=true > vision.log 2>&1 &
# nohup ros2 run ros2_sync_package sync_node > sync_node.log 2>&1 &
# nohup sh src/vision_segmentation/run.sh > vision_segmentation.log 2>&1 &
echo "[START BRAIN]"
BRAIN_ARGS=()
START_GAME_CONTROLLER=true
for arg in "$@"; do
  if [ "${arg}" = "no_game_controller:=true" ]; then
    START_GAME_CONTROLLER=false
  else
    BRAIN_ARGS+=("${arg}")
  fi
done
nohup ros2 launch brain launch.py "${BRAIN_ARGS[@]}" > brain.log 2>&1 &
# nohup ros2 launch brain launch.py "$@"  > brain.log 2>&1 &
echo "[START GAME_CONTROLLER]"
if [ "${START_GAME_CONTROLLER}" = true ]; then
  nohup ros2 launch game_controller launch.py > game_controller.log 2>&1 &
else
  echo "[SKIP GAME_CONTROLLER: laboratory mode]"
fi
echo "[START GOALKEEPER WEB PANEL]"
bash ./scripts/start_goalkeeper_web.sh
#echo "[START SOUND]"
#nohup ros2 run sound_play sound_play_node > sound.log 2>&1 &
echo "[DONE]"
sudo jetson_clocks
