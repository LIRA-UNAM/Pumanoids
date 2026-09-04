#!/bin/bash

# ==========================================
# 1. VALORES POR DEFECTO Y LECTURA DE PARÁMETROS
# ==========================================
team="0"
id="2"
role="striker"
ROS_ARGS=() # Guarda cualquier otro argumento para pasarlo al launch.py

while [[ "$#" -gt 0 ]]; do
    case $1 in
        -t|--team) TEAM_ID="$2"; shift ;;
        -p|--player) PLAYER_ID="$2"; shift ;;
        -r|--role) PLAYER_ROLE="$2"; shift ;;
        *) ROS_ARGS+=("$1") ;; # Conserva parámetros desconocidos para ROS
    esac
    shift
done

cd `dirname $0`
cd ..
WORKSPACE_ROOT=$(pwd)

# ==========================================
# 2. ACTUALIZACIÓN AUTOMÁTICA DEL YAML
# ==========================================

BRAIN_CONFIG_PATH="${WORKSPACE_ROOT}/beijing_ws/src/brain/config/config.yaml"

if [ -f "$BRAIN_CONFIG_PATH" ]; then
    echo "[CONFIG] Ajustando brain_node.yaml -> Team: $TEAM_ID | Player: $PLAYER_ID | Role: $PLAYER_ROLE"
    # Estas expresiones regulares cambian solo el valor, respetando la indentación y los comentarios #
    sed -i -E "s/(team_id:[[:space:]]*)[0-9]+/\1$TEAM_ID/" "$BRAIN_CONFIG_PATH"
    sed -i -E "s/(player_id:[[:space:]]*)[0-9]+/\1$PLAYER_ID/" "$BRAIN_CONFIG_PATH"
    sed -i -E "s/(player_role:[[:space:]]*)\"[^\"]+\"/\1\"$PLAYER_ROLE\"/" "$BRAIN_CONFIG_PATH"
else
    echo "[ADVERTENCIA] No se encontró $BRAIN_CONFIG_PATH. Ignorando autoconfiguración."
fi

VISION_CONFIG_PATH="${WORKSPACE_ROOT}/src/vision/config"

# ==========================================
# 3. EJECUCIÓN ORIGINAL
# ==========================================
echo "[STOP EXISTING NODES (IF ANY), TO AVOID CONFILICT]"
sudo killall -9 booster-video-stream
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
source ./beijing_ws/install/setup.bash
unset FASTRTPS_DEFAULT_PROFILES_FILE
export FASTDDS_DEFAULT_PROFILES_FILE=/opt/booster/BoosterRos2/fastdds_profile_udp_only.xml

echo "[START VISION]"
nohup ros2 launch vision launch.py save_data:=true > vision.log 2>&1 &

echo "[START BRAIN]"
# Usamos "${ROS_ARGS[@]}" para pasar cualquier argumento extra que no hayamos interceptado
nohup ros2 launch brain launch.py "${ROS_ARGS[@]}" > brain.log 2>&1 &

echo "[START GAME_CONTROLLER]"
nohup ros2 launch game_controller launch.py > game_controller.log 2>&1 &

echo "[DONE]"
sudo jetson_clocks
