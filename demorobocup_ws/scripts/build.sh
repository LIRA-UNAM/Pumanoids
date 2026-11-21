#!/bin/bash
set -e

cd "$(dirname "$0")/.."
WORKSPACE_ROOT=$(pwd)

SIM_MODE=0
EXTRA_ARGS=()
for arg in "$@"; do
  case "$arg" in
    --sim|--simulation)
      SIM_MODE=1
      ;;
    *)
      EXTRA_ARGS+=("$arg")
      ;;
  esac
done

export PYTHONWARNINGS="ignore::DeprecationWarning,ignore::UserWarning,ignore::FutureWarning"

if [[ "$SIM_MODE" -eq 0 ]]; then
  # colcon build  --symlink-install  --parallel-workers $(nproc) "$@"
  colcon build --parallel-workers $(nproc) "${EXTRA_ARGS[@]}"
  espeak "build complete" >/dev/null 2>&1 || echo "Build complete"
  exit 0
fi

# Modo simulacion (--sim):
# - vision usa backend ONNX (sin CUDA/TensorRT ni booster_robotics_sdk.a)
# - brain mantiene rerun_sdk si esta instalado, y omite booster_internal privado
if [[ -z "${AMENT_PREFIX_PATH:-}" ]]; then
  for distro in "${ROS_DISTRO:-}" jazzy humble; do
    if [[ -n "$distro" ]] && [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
      set +u
      # shellcheck disable=SC1090
      source "/opt/ros/${distro}/setup.bash"
      set -u
      break
    fi
  done
fi

UPSTREAM_BASE="$(cd "${WORKSPACE_ROOT}/../.." && pwd)/BoosterSoccerSim/.upstream"

if [[ -z "${ONNXRUNTIME_ROOT:-}" ]]; then
  for cand in /opt/onnxruntime "${UPSTREAM_BASE}/onnxruntime-linux-x64-1.23.2"; do
    if [[ -d "$cand" ]]; then
      export ONNXRUNTIME_ROOT="$cand"
      break
    fi
  done
fi

CMAKE_ARGS=("-DENABLE_ONNX=ON" "-DENABLE_TENSORRT=OFF" "-DPUMASIM_SIMULATION_PROFILE=ON")

if [[ -z "${BOOSTER_SDK_INCLUDE_DIR:-}" ]]; then
  for cand in /usr/local/include /usr/include "${UPSTREAM_BASE}/booster_robotics_sdk/include"; do
    if [[ -f "${cand}/booster/robot/b1/b1_api_const.hpp" ]]; then
      export BOOSTER_SDK_INCLUDE_DIR="$cand"
      break
    fi
  done
fi
if [[ -n "${BOOSTER_SDK_INCLUDE_DIR:-}" ]]; then
  CMAKE_ARGS+=("-DBOOSTER_SDK_INCLUDE_DIR=${BOOSTER_SDK_INCLUDE_DIR}")
fi

if [[ -z "${behaviortree_cpp_DIR:-}" ]] && [[ ! -d "/opt/ros/${ROS_DISTRO:-jazzy}/share/behaviortree_cpp" ]]; then
  cand="${UPSTREAM_BASE}/robocup_demo/install_jazzy_full/behaviortree_cpp/share/behaviortree_cpp/cmake"
  if [[ -d "$cand" ]]; then
    CMAKE_ARGS+=("-Dbehaviortree_cpp_DIR=${cand}")
  fi
fi

RERUN_FOUND=0
for cand in "${RERUN_SDK_PREFIX:-}" "/opt/rerun_sdk" "${HOME}/.local/rerun_sdk" "/usr/local"; do
  if [[ -n "$cand" ]] && [[ -f "${cand}/lib/cmake/rerun_sdk/rerun_sdkConfig.cmake" ]]; then
    export RERUN_SDK_PREFIX="$cand"
    export CMAKE_PREFIX_PATH="${cand}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
    CMAKE_ARGS+=("-DENABLE_RERUN=ON" "-DRERUN_SDK_PREFIX=${cand}")
    RERUN_FOUND=1
    break
  fi
done
if [[ "$RERUN_FOUND" -eq 0 ]]; then
  echo "[build.sh --sim] rerun_sdk no encontrado; compilando brain con -DENABLE_RERUN=OFF (ejecuta beijing/scripts/install_rerun_cpp.sh para habilitarlo)."
  CMAKE_ARGS+=("-DENABLE_RERUN=OFF")
fi

# Compilar primero paquetes base de mensajes/puente si aun no existen en install/
colcon build --symlink-install --parallel-workers "$(nproc)" \
  --packages-select booster_msgs booster_interface vision_interface game_controller_interface game_controller pumasim_booster_bridge vision brain \
  "${EXTRA_ARGS[@]}" \
  --cmake-args "${CMAKE_ARGS[@]}"

espeak "build complete" >/dev/null 2>&1 || echo "Build complete"
