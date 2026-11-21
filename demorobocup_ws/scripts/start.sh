#!/bin/bash

cd "$(dirname "$0")"
cd ..
WORKSPACE_ROOT=$(pwd)
VISION_CONFIG_PATH="${WORKSPACE_ROOT}/src/vision/config"

SIM_MODE=0
ENABLE_BRIDGE=1
ENABLE_VISION=1
ENABLE_BRAIN=1
ENABLE_GC=1
ROBOT_NAME="blue_1"
ROBOT_TYPE="t1"
ROBOTS_CSV=""
ROSTER_FILE=""
SAVE_DATA="false"
USE_MUJOCO_LOC="false"
SECOND_HALF_SWAPS="false"
GC_IP="127.0.0.1"
GC_PORT="3838"
GC_WHITELIST=""
GC_WHITELIST_IPS=""
BRAIN_ARGS=()
RAW_ARGS=("$@")

for arg in "$@"; do
  case "$arg" in
    --sim|--simulation)
      SIM_MODE=1
      ;;
    sim:=true|sim:=True|sim:=1)
      SIM_MODE=1
      ;;
    sim:=false|sim:=False|sim:=0)
      SIM_MODE=0
      ;;
    --no-bridge)
      ENABLE_BRIDGE=0
      ;;
    --no-vision)
      ENABLE_VISION=0
      ;;
    --no-brain)
      ENABLE_BRAIN=0
      ;;
    --no-gc)
      ENABLE_GC=0
      ;;
    robot:=*|robot_name:=*)
      ROBOT_NAME="${arg#*=}"
      ;;
    robots:=*)
      ROBOTS_CSV="${arg#*=}"
      ;;
    roster:=*|roster_file:=*)
      ROSTER_FILE="${arg#*=}"
      ;;
    robot_type:=*)
      ROBOT_TYPE="${arg#*=}"
      ;;
    save_data:=*)
      SAVE_DATA="${arg#*=}"
      ;;
    use_mujoco_localization:=*|mujoco_localization:=*)
      USE_MUJOCO_LOC="${arg#*=}"
      ;;
    second_half_swaps_teams:=*)
      SECOND_HALF_SWAPS="${arg#*=}"
      ;;
    localization:=mujoco|localization:=sim)
      USE_MUJOCO_LOC="true"
      ;;
    localization:=demo|localization:=internal)
      USE_MUJOCO_LOC="false"
      ;;
    gc_port:=*)
      GC_PORT="${arg#*=}"
      ;;
    gc_whitelist:=*)
      GC_WHITELIST="${arg#*=}"
      ;;
    gc_whitelist_ips:=*)
      GC_WHITELIST_IPS="${arg#*=}"
      ;;
    gc_ip:=*)
      GC_IP="${arg#*=}"
      BRAIN_ARGS+=("game_control_ip:=${GC_IP}")
      ;;
    *)
      BRAIN_ARGS+=("$arg")
      ;;
  esac
done

if [[ "$SIM_MODE" -eq 0 ]]; then
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
  sudo systemctl disable --now booster-agent-manager.service

  echo "[START ROBOCUP NODES]"
  source ./install/setup.bash
  export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml
  # export RMW_FASTRTPS_USE_SHM=0

  echo "[START VISION]"
  # 如果是用的zed相机，https://booster.feishu.cn/wiki/XodtwX56AiCtZtkewo3cPgbrn8d#share-MDrvdyWa2o87qixU3TccE72NnNc 文档中下载安装包，可以自启动zed
  # 注意: 如果是realsense相机，千万不要改，否则realsense摄像头会起不来，这个时候只能cd ~/Documents/recovery/ 重装daemon-perception恢复
  # source ~/ThirdParty/zed-ros/install/setup.bash
  # nohup ros2 launch zed_wrapper zed_camera.launch.py camera_model:="zed2i" > zed.log 2>&1 &
  # nohup ros2 launch vision launch.py save_data:=true > vision.log 2>&1 &
  nohup ros2 launch vision launch.py save_data:=true > vision.log 2>&1 &
  # nohup ros2 run ros2_sync_package sync_node > sync_node.log 2>&1 &
  # nohup sh src/vision_segmentation/run.sh > vision_segmentation.log 2>&1 &
  echo "[START BRAIN]"
  nohup ros2 launch brain launch.py "${RAW_ARGS[@]}" > brain.log 2>&1 &
  # nohup ros2 launch brain launch.py "$@"  > brain.log 2>&1 &
  echo "[START GAME_CONTROLLER]"
  nohup ros2 launch game_controller launch.py > game_controller.log 2>&1 &
  #echo "[START SOUND]"
  #nohup ros2 run sound_play sound_play_node > sound.log 2>&1 &
  echo "[DONE]"
  sudo jetson_clocks
  exit 0
fi

echo "[STOP EXISTING SIMULATION NODES (IF ANY)]"
./scripts/stop.sh --sim

echo "[START ROBOCUP NODES (SIMULATION)]"
if [[ -z "${AMENT_PREFIX_PATH:-}" ]]; then
  for distro in "${ROS_DISTRO:-}" jazzy humble; do
    if [[ -n "$distro" ]] && [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
      source "/opt/ros/${distro}/setup.bash"
      break
    fi
  done
fi
source ./install/setup.bash

PUMASIM_SHM_XML="$(cd ../.. && pwd)/PumaSim/ros2_ws/config/fastdds_local_shm.xml"
if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
  if [[ -f "${PUMASIM_SHM_XML}" ]]; then
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
    export FASTRTPS_DEFAULT_PROFILES_FILE="${PUMASIM_SHM_XML}"
    export FASTDDS_DEFAULT_PROFILES_FILE="${PUMASIM_SHM_XML}"
  elif [[ -f "./configs/fastdds.xml" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="./configs/fastdds.xml"
  fi
fi

if [[ -n "$ROBOTS_CSV" || -n "$ROSTER_FILE" ]]; then
  echo "[START MULTI-ROBOT MATCH (SIM)] (robots=${ROBOTS_CSV:-from_roster}, use_mujoco_localization=${USE_MUJOCO_LOC})"
  ENABLE_VISION_STR=$([[ "$ENABLE_VISION" -eq 1 ]] && echo "true" || echo "false")
  ENABLE_BRAIN_STR=$([[ "$ENABLE_BRAIN" -eq 1 ]] && echo "true" || echo "false")
  ENABLE_GC_STR=$([[ "$ENABLE_GC" -eq 1 ]] && echo "true" || echo "false")
  MATCH_LAUNCH_ARGS=(
    "robot_type:=${ROBOT_TYPE}"
    "enable_vision:=${ENABLE_VISION_STR}" "enable_brain:=${ENABLE_BRAIN_STR}"
    "enable_gc:=${ENABLE_GC_STR}" "enable_com:=true"
    "use_mujoco_localization:=${USE_MUJOCO_LOC}"
    "second_half_swaps_teams:=${SECOND_HALF_SWAPS}"
    "game_control_ip:=${GC_IP}" "gc_port:=${GC_PORT}"
  )
  [[ -n "$ROBOTS_CSV" ]] && MATCH_LAUNCH_ARGS+=("robots:=${ROBOTS_CSV}")
  [[ -n "$ROSTER_FILE" ]] && MATCH_LAUNCH_ARGS+=("roster_file:=${ROSTER_FILE}")
  nohup ros2 launch pumasim_booster_bridge robocup_demo_match.launch.py \
    "${MATCH_LAUNCH_ARGS[@]}" \
    > match.log 2>&1 &
  echo "[DONE]"
  exit 0
fi

if [[ "$ENABLE_BRIDGE" -eq 1 ]]; then
  echo "[START BOOSTER BRIDGE] (${ROBOT_NAME}, ${ROBOT_TYPE})"
  nohup ros2 launch pumasim_booster_bridge booster_bridge.launch.py \
    "robots:=${ROBOT_NAME}" "robot_type:=${ROBOT_TYPE}" layout:=root relay_rgb:=true \
    > booster_bridge.log 2>&1 &
  sleep 1
fi

if [[ "$ENABLE_VISION" -eq 1 ]]; then
  echo "[START VISION (SIM ONNX)]"
  nohup ros2 launch vision launch.py sim:=true \
    "robot_name:=${ROBOT_NAME}" "robot_type:=${ROBOT_TYPE}" "save_data:=${SAVE_DATA}" \
    > vision.log 2>&1 &
fi

if [[ "$ENABLE_BRAIN" -eq 1 ]]; then
  echo "[START BRAIN (SIM)] (use_mujoco_localization=${USE_MUJOCO_LOC})"
  nohup ros2 launch brain launch.py sim:=true \
    "robot_name:=${ROBOT_NAME}" "robot_type:=${ROBOT_TYPE}" \
    "use_mujoco_localization:=${USE_MUJOCO_LOC}" "${BRAIN_ARGS[@]}" \
    > brain.log 2>&1 &
fi

if [[ "$ENABLE_GC" -eq 1 ]]; then
  echo "[START GAME_CONTROLLER (SIM)]"
  GC_ARGS=("sim:=true" "port:=${GC_PORT}")
  if [[ -n "$GC_WHITELIST" ]]; then
    GC_ARGS+=("enable_ip_white_list:=${GC_WHITELIST}")
  fi
  if [[ -n "$GC_WHITELIST_IPS" ]]; then
    GC_ARGS+=("ip_white_list:=${GC_WHITELIST_IPS}")
  fi
  nohup ros2 launch game_controller launch.py "${GC_ARGS[@]}" \
    > game_controller.log 2>&1 &
fi

echo "[DONE]"
