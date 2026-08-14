#!/bin/bash
set -euo pipefail

cd "$(dirname "$0")/.."
SIDE="${1:-left}"
if [ "${SIDE}" != "left" ] && [ "${SIDE}" != "right" ]; then
  echo "Usage: $0 [left|right]"
  exit 2
fi

echo "WARNING: goalkeeper_lab moves the head and body without GameController."
echo "Place the robot upright, clear the field, and keep emergency control ready."
read -r -p "Type ARMAR to start autonomous localization and positioning: " CONFIRM
if [ "${CONFIRM}" != "ARMAR" ]; then
  echo "Cancelled. No nodes were started."
  exit 1
fi

exec ./scripts/start.sh \
  tree:=goalkeeper_lab \
  role:=goal_keeper \
  pos:="${SIDE}" \
  disable_com:=true \
  no_game_controller:=true
