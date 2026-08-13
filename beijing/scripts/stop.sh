#!/bin/bash
echo ["STOP VISION"]
sudo killall -9 vision_node
echo ["STOP BRAIN"]
sudo killall -9 brain_node
echo ["STOP SOUND"]
sudo killall -9 sound_play_node
echo ["STOP GAMECONTROLLER"]
# Some robot images start GameController from a Restart=always system service.
# Stop the owner first; otherwise it can reclaim UDP 3838 after killall.
sudo systemctl stop robocup_game_controller.service 2>/dev/null || true
sudo pkill -f "ros2 launch game_controller launch.py" 2>/dev/null || true
# Linux truncates game_controller_node to 15 characters in /proc/*/comm, so
# match the full executable command line instead of relying on killall naming.
sudo pkill -9 -f '[/]game_controller_node([[:space:]]|$)' 2>/dev/null || true
# Keep the truncated/legacy name as a fallback for older installed packages.
sudo killall -9 game_controller 2>/dev/null || true
echo ["STOP GOALKEEPER WEB PANEL"]
sudo pkill -f '[g]oalkeeper_web/server.py' 2>/dev/null || true
