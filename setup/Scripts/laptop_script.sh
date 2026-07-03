#!/bin/bash

echo "             ⢀⣠⣄   ⢀⣀⡀"
echo "             ⢻⣿⣿⣶⣶⣶⣾⣿⣿⠇"
echo "          ⢄⣀⣴⣿⣿⣿⣿⣿⣿⣿⣿⣿⣦⡀⢀⡀"
echo "           ⠈⠙⠻⠿⠋⠉⠉⠉⠛⢿⣿⣿⣿⣿⣿⡄"
echo "⢰⣶⡆  ⢰⣶⡆⢰⣶⣶⣶⣶⣄ ⢀⣶⣶⣆ ⠈⣿⣿⣿⣿⠉"
echo "⢸⣿⡇  ⢸⣿⡇⢸⣿⣇⣀⣿⠟ ⣼⣿⣹⣿⡄ ⠉⠛⠿⣿⡀"
echo "⢸⣿⣷⣶⣶⢸⣿⡇⢸⣿⡟⠻⣿⣧⣸⣿⠟⠛⢿⣿⡀    ⠈"
echo "\n\n"

echo "---------------------"
echo "|   LAPTOP SCRIPT   |"
echo "---------------------"
echo "\nClonning the Pumanoids repo to robot..."
scp -r /home/${USER}/Pumanoids booster@192.168.10.102:/home/booster/
echo "\n\n"
echo "Repository clonned to /home/booster/ on robot"
echo "--------------------"
echo "|     FINISHED     |"
echo "--------------------"
