#!/bin/bash
script_path=$(readlink -f "$0")

echo "Script path: $script_path"
# Use the parent directory as the project root.
root_path=$(dirname $script_path)
echo "Proj root path: $root_path"

rm -rf /home/booster/Workspace/robocup
mkdir -p /home/booster/Workspace/robocup

cp -r $root_path/* /home/booster/Workspace/robocup
bash /home/booster/Workspace/robocup/utils/install_auto_start_assist.sh # Enable automatic startup.
echo "Install success"
