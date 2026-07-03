#!/bin/bash

echo "             ⢀⣠⣄   ⢀⣀⡀"
echo "             ⢻⣿⣿⣶⣶⣶⣾⣿⣿⠇"
echo "          ⢄⣀⣴⣿⣿⣿⣿⣿⣿⣿⣿⣿⣦⡀⢀⡀"
echo "           ⠈⠙⠻⠿⠋⠉⠉⠉⠛⢿⣿⣿⣿⣿⣿⡄"
echo "⢰⣶⡆  ⢰⣶⡆⢰⣶⣶⣶⣶⣄ ⢀⣶⣶⣆ ⠈⣿⣿⣿⣿⠉"
echo "⢸⣿⡇  ⢸⣿⡇⢸⣿⣇⣀⣿⠟ ⣼⣿⣹⣿⡄ ⠉⠛⠿⣿⡀"
echo "⢸⣿⣷⣶⣶⢸⣿⡇⢸⣿⡟⠻⣿⣧⣸⣿⠟⠛⢿⣿⡀    ⠈"
echo "\n\n"

echo "-------------------------------------"
echo "|   POST-COMPILATION ROBOT SCRIPT   |"
echo "-------------------------------------"

echo "\n[Script] Initializing Pumanoids workspace..."
source /home/${USER}/Pumanoids/colcon_ws/install/setup.bash

echo "\n[Script] Generating a .engine model for this robot"
cd /home/${USER}/Pumanoids/colcon_ws/src/vision/new_ball_detector/models/
echo "\n[Script] Renaming old model"
mv /home/${USER}/Pumanoids/colcon_ws/src/vision/new_ball_detector/models/yolov8_center_sys_low_t1.engine /home/${USER}/Pumanoids/colcon_ws/src/vision/new_ball_detector/models/old_yolov8_center_sys_low_t1.engine
echo "\n[Script] Converting the .pt to .onnx..."
python3 -c "from ultralytics import YOLO; model = YOLO('yolov8_center.pt'); model.export(format='onnx', imgsz=320, opset=17)"

echo "\n[Script] Converting the .onnx to .engine..."
/usr/src/tensorrt/bin/trtexec --onnx=yolov8_center.onnx --saveEngine=yolov8_center_sys_low_t1.engine --fp16 --memPoolSize=workspace:256 --builderOptimizationLevel=0

echo "\n[Script] Please, cd to the colcon_ws and rebuild the new_ball_detector package:"
echo "colcon build --packages-select new_ball_detector\n"

echo "--------------------"
echo "|     FINISHED     |"
echo "--------------------"
