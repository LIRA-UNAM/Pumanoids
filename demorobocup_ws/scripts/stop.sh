#!/bin/bash

SIM_MODE=0
for arg in "$@"; do
  case "$arg" in
    --sim|--simulation|sim:=true|sim:=True|sim:=1)
      SIM_MODE=1
      ;;
  esac
done

if [[ "$SIM_MODE" -eq 1 ]]; then
  echo "[STOP SIMULATION NODES]"
  pkill -f "vision/lib/vision/vision_node" 2>/dev/null || true
  pkill -f "brain/lib/brain/brain_node" 2>/dev/null || true
  pkill -f "booster_bridge" 2>/dev/null || true
  pkill -f "pumasim_booster_bridge" 2>/dev/null || true
  pkill -f "match_supervisor" 2>/dev/null || true
  pkill -f "robocup_demo_" 2>/dev/null || true
  pkill -f "game_controller_node" 2>/dev/null || true
  pkill -f "sound_play_node" 2>/dev/null || true
  exit 0
fi

echo ["STOP VISION"]
sudo killall -9 vision_node
echo ["STOP BRAIN"]
sudo killall -9 brain_node
echo ["STOP SOUND"]
sudo killall -9 sound_play_node
echo ["STOP GAMECONTROLLER"]
sudo killall -9 game_controller
