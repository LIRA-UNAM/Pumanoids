#!/bin/bash
echo "[START VISION]"
cd "$(dirname "$0")/.."

SIM_MODE=0
EXTRA_ARGS=()
for arg in "$@"; do
  case "$arg" in
    --sim|--simulation)
      SIM_MODE=1
      ;;
    sim:=true|sim:=True|sim:=1)
      SIM_MODE=1
      EXTRA_ARGS+=("$arg")
      ;;
    *)
      EXTRA_ARGS+=("$arg")
      ;;
  esac
done

source ./install/setup.bash
#export FASTRTPS_DEFAULT_PROFILES_FILE=./configs/fastdds.xml

if [[ "$SIM_MODE" -eq 1 ]]; then
  ros2 launch vision launch.py sim:=true show_det:=false save_data:=false "${EXTRA_ARGS[@]}"
else
  # ros2 launch vision launch.py show_det:=true > vision_d.log 2>&1
  ros2 launch vision launch.py show_det:=false save_data:=false "${EXTRA_ARGS[@]}"
fi
