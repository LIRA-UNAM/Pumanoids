FROM osrf/ros:noetic-desktop-full
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    git \
    python-is-python3 \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-ros-control \
    ros-noetic-controller-manager \
    ros-noetic-effort-controllers \
    ros-noetic-joint-state-controller \
    ros-noetic-moveit-kinematics \
    ros-noetic-dynamixel-sdk \
    ros-noetic-moveit-core \
    ros-noetic-moveit-ros-planning-interface
WORKDIR /
RUN ./ros_entrypoint.sh
WORKDIR /home/miguel