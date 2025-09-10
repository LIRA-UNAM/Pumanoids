#!/bin/bash

TARGET_DIR="luckfox-pico"
SEARCH_PATH="/home/$(whoami)"
INSTALL_URL="https://github.com/LuckfoxTECH/luckfox-pico.git"

# ANSI escape codes for colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m' # Yellow text for the title bar (bold yellow)
NC='\033[0m' # No Color (resets to default)


# --- Function to create a title bar ---
TITLE_TEXT="PUMAS SOFTWARE FOR LUCKFOX BOARD"
create_title_bar() {
  local title="$1"
  local width=50 # Width of the title bar
  local padding_left=$(( (width - ${#title}) / 2 ))
  local padding_right=$(( width - ${#title} - padding_left ))

  echo -e "${YELLOW}╔$(printf '═%.0s' $(seq 1 $width))╗${NC}"
  echo -e "${YELLOW}║$(printf ' %.0s' $(seq 1 $padding_left))${title}$(printf ' %.0s' $(seq 1 $padding_right))║${NC}"
  echo -e "${YELLOW}╚$(printf '═%.0s' $(seq 1 $width))╝${NC}"
  echo # Add a newline for spacing
}

# --- Main Script Execution ---

# Display the title bar
create_title_bar "$TITLE_TEXT"

FOUND_PATH=$(find "$SEARCH_PATH" -type d -name "$TARGET_DIR" -print -quit 2>/dev/null)

# Check if the directory was found
if [ -n "$FOUND_PATH" ]; then
  export LUCKFOX_SDK_PATH=$FOUND_PATH
  echo -e "${GREEN}LUCKFOX_SDK was found at: ${LUCKFOX_SDK_PATH}${NC}"
else
  echo -e "${RED}Error: LUCKFOX_SDK '$TARGET_DIR' not found in '$SEARCH_PATH'.${NC}" >&2
  echo -e "${RED}Please install it from: $INSTALL_URL${NC}" >&2
  exit 1 # Exit with an error status
fi

ROOT_PWD=$(cd "$(dirname $0)" && cd -P "$(dirname "$SOURCE")" && pwd)

if [ "$1" = "clean" ]; then
	if [ -d "${ROOT_PWD}/build" ]; then
		rm -rf "${ROOT_PWD}/build"
		echo " ${ROOT_PWD}/build has been deleted!"
	fi

	if [ -d "${ROOT_PWD}/install" ]; then
		rm -rf "${ROOT_PWD}/install"
		echo " ${ROOT_PWD}/install has been deleted!"
	fi

	exit
fi

options=(
	"luckfox_pico_rtsp_yolov5")

PS3="Enter your choice [1-${#options[@]}]: "

select opt in "${options[@]}"; do
	if [[ -n "$opt" ]]; then
		echo "You selected: $opt"

		src_dir="src/$opt"
		if [[ -d "$src_dir" ]]; then
			if [ -d ${ROOT_PWD}/build ]; then
				rm -rf ${ROOT_PWD}/build
			fi
			mkdir ${ROOT_PWD}/build
			cd ${ROOT_PWD}/build
			cmake .. -DEXAMPLE_DIR="$src_dir" -DEXAMPLE_NAME="$opt"
			make install
		else
			echo "Error: Directory $src_dir does not exist!"
		fi
		break
	else
		echo "Invalid selection, please try again."
	fi
done
