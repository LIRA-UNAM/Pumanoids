#!/bin/bash
echo "[START BRAIN]"

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
    mujoco_localization:=*)
      EXTRA_ARGS+=("use_mujoco_localization:=${arg#*=}")
      ;;
    localization:=mujoco|localization:=sim)
      EXTRA_ARGS+=("use_mujoco_localization:=true")
      ;;
    localization:=demo|localization:=internal)
      EXTRA_ARGS+=("use_mujoco_localization:=false")
      ;;
    *)
      EXTRA_ARGS+=("$arg")
      ;;
  esac
done

source ./install/setup.bash
if [[ "$SIM_MODE" -eq 1 ]]; then
  PUMASIM_SHM_XML="$(cd ../.. && pwd)/PumaSim/ros2_ws/config/fastdds_local_shm.xml"
  if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]] && [[ -f "${PUMASIM_SHM_XML}" ]]; then
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT
    export FASTRTPS_DEFAULT_PROFILES_FILE="${PUMASIM_SHM_XML}"
    export FASTDDS_DEFAULT_PROFILES_FILE="${PUMASIM_SHM_XML}"
  fi
  ros2 launch brain launch.py sim:=true "${EXTRA_ARGS[@]}"
else
  export FASTRTPS_DEFAULT_PROFILES_FILE=./configs/fastdds.xml
  ros2 launch brain launch.py "${EXTRA_ARGS[@]}"
fi
