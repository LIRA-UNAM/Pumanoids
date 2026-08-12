#!/bin/sh
# Resolve the absolute script path.
script_path=$(readlink -f "$0")
echo "Script path: $script_path"

# Use the parent directory as the project root.
root_path=$(dirname $script_path)
echo "root_path: $root_path"

service_path=$root_path/service

# Stop the service before updating it.
systemctl --user stop robocup_game_assist.service
mkdir -p ~/.config/systemd/user/
cp $service_path/robocup_game_assist.service ~/.config/systemd/user/

# Restart the service.
systemctl --user daemon-reload

systemctl --user start robocup_game_assist.service
systemctl --user enable robocup_game_assist.service
