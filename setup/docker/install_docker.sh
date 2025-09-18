#!/bin/bash

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

create_title_bar "Uninstalling docker components for clean install"

for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do sudo apt-get remove $pkg; done

# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

sudo systemctl status docker

sudo groupadd docker

sudo usermod -aG docker $USER

create_title_bar "Docker succesfully installed. Reboot to start using docker"

