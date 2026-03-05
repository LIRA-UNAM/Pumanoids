#!/bin/bash
set -e

echo "Installing Pumanoids dependencies..."

apt-get update
apt-get install -y \
    libpcl-dev \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    nano

echo "Dependencies installed."
