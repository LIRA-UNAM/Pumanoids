#!/bin/bash

echo "             ⢀⣠⣄   ⢀⣀⡀"
echo "             ⢻⣿⣿⣶⣶⣶⣾⣿⣿⠇"
echo "          ⢄⣀⣴⣿⣿⣿⣿⣿⣿⣿⣿⣿⣦⡀⢀⡀"
echo "           ⠈⠙⠻⠿⠋⠉⠉⠉⠛⢿⣿⣿⣿⣿⣿⡄"
echo "⢰⣶⡆  ⢰⣶⡆⢰⣶⣶⣶⣶⣄ ⢀⣶⣶⣆ ⠈⣿⣿⣿⣿⠉"
echo "⢸⣿⡇  ⢸⣿⡇⢸⣿⣇⣀⣿⠟ ⣼⣿⣹⣿⡄ ⠉⠛⠿⣿⡀"
echo "⢸⣿⣷⣶⣶⢸⣿⡇⢸⣿⡟⠻⣿⣧⣸⣿⠟⠛⢿⣿⡀    ⠈"
echo "\n\n"

echo "------------------------------------"
echo "|   PRE-COMPILATION ROBOT SCRIPT   |"
echo "------------------------------------"

echo "\n[Script] Printing Jetpack version..."
sleep 2
sudo apt show nvidia-jetpack -a

echo "\n[Script] Installing lastest torchvision dependencies..."
sleep 1
pip install torch torchvision --index-url pypi.jetson-ai-lab.io/jp6/cu126

echo "\n[Script] Installing constructor..."
pip3 install /home/${USER}/Pumanoids/setup/Dependencies/construct-2.10.70-py3-none-any.whl

echo "\n[Script] Installing extra dependencies..."
mkdir -p /home/booster/python-v-env/lib/python3.10/site-packages/numpy/core/
ln -s /home/booster/.local/lib/python3.10/site-packages/numpy/core/include /home/booster/python-v-env/lib/python3.10/site-packages/numpy/core/include

echo "\n[Script] Initializing Booster ROS2 workspace (Ignore the warning)"
source /opt/booster/BoosterRos2/install/setup.bash

echo "\n[Script] Adding workspace initialization to bashrc"

echo "" >> /home/${USER}/.bashrc
echo "#===== PUMANOIDS ADDITIONS =====" >> /home/${USER}/.bashrc
echo "source /home/${USER}/Pumanoids/colcon_ws/install/setup.bash" >> /home/${USER}/.bashrc
echo "source /opt/booster/BoosterRos2/install/setup.bash" >> /home/${USER}/.bashrc
echo "#===============================" >> /home/${USER}/.bashrc

echo "\n\n"
echo "Pre compilation script finished"
echo "Please, run make t1 to compile the project "
echo "--------------------"
echo "|     FINISHED     |"
echo "--------------------"
