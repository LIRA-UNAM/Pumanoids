#!/bin/bash

# ================================= Edit Here ================================ #
DISTRO=jazzy
# Change these values to use different versions of ROS or different base images. The rest of the script should be left unchanged.
BASE_IMAGE=osrf/ros
BASE_TAG=$DISTRO-desktop-full
IMAGE_NAME=ros2-jazzy
IMAGE_TAG=0.1

USERNAME=miguel
USER_UID="$(id -u $USER)"
USER_GID="$(id -g $USER)"

# =============================== Help Function ============================== #

helpFunction()
{
   echo ""
   echo -e "\t-h   --help          Script used to build the image."
   exit 1 # Exit script after printing help
}

# =============================== BUILD ============================== #

# Auxiliary functions
die() { echo "$*" >&2; exit 2; }  # complain to STDERR and exit with error
needs_arg() { if [ -z "$OPTARG" ]; then die "No arg for --$OPT option"; fi; }
no_arg() { if [ -n "$OPTARG" ]; then die "No arg allowed for --$OPT option"; fi; }

# Get the script options. This accepts both single dash (e.g. -a) and double dash options (e.g. --all)
while getopts h-: OPT; do
  # support long options: https://stackoverflow.com/a/28466267/519360
  if [ "$OPT" = "-" ]; then   # long option: reformulate OPT and OPTARG
    OPT="${OPTARG%%=*}"       # extract long option name
    OPTARG="${OPTARG#$OPT}"   # extract long option argument (may be empty)
    OPTARG="${OPTARG#=}"      # if long option argument, remove assigning `=`
  fi
  case "$OPT" in
    h | help )          no_arg; helpFunction ;;
  esac
done

docker compose build \
--build-arg BASE_IMAGE=$BASE_IMAGE \
--build-arg BASE_TAG=$BASE_TAG \
--build-arg IMAGE_NAME=$IMAGE_NAME \
--build-arg IMAGE_TAG=$IMAGE_TAG \
--build-arg USERNAME=$USERNAME \
--build-arg USER_UID=$USER_UID \
--build-arg USER_GID=$USER_GID \
--build-arg DISTRO=$DISTRO 