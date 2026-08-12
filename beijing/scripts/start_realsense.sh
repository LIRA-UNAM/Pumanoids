#!/bin/bash

# Resolve the workspace root from this script location.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
LOG_DIR="$PROJECT_DIR/log"

# Create the log directory when needed.
mkdir -p "$LOG_DIR"

# Log file path.
LOG_FILE="$LOG_DIR/realsense_log"

# RealSense ROS workspace.
REALSENSE_PATH="$HOME/ThirdParty/realsense-ros"

# Validate the RealSense ROS workspace.
if [ ! -d "$REALSENSE_PATH" ]; then
    echo "Error: RealSense ROS workspace does not exist: $REALSENSE_PATH"
    exit 1
fi

# Enter the RealSense workspace and source setup.bash.
cd "$REALSENSE_PATH"

# Source the ROS 2 environment.
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
else
    echo "Error: ROS 2 environment not found"
    exit 1
fi

# Start the RealSense camera in the background and redirect output to the log.
echo "Starting RealSense camera..."
echo "Log file: $LOG_FILE"
echo "========================================" > "$LOG_FILE"
echo "RealSense camera start time: $(date)" >> "$LOG_FILE"
echo "========================================" >> "$LOG_FILE"
nohup ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true >> "$LOG_FILE" 2>&1 &

# Capture the process ID.
REALSENSE_PID=$!

# Save the process ID for later shutdown.
echo $REALSENSE_PID > "$LOG_DIR/realsense.pid"

echo "RealSense camera started in the background"
echo "Process ID: $REALSENSE_PID"
echo "View logs: tail -f $LOG_FILE"
echo "Stop camera: kill $REALSENSE_PID or run stop_realsense.sh"
