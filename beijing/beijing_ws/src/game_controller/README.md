# robocup game_controller
## Overview

Receives RoboCup GameController UDP packets from the local network and publishes them as ROS 2 messages for the brain node.

The implementation supports RoboCupGameControlData v20:

- Control packets: UDP 3838, `RGme`, 158 bytes
- Robot status packets: UDP 3939, `RGrt` v4, 32 bytes
- The GameController IP allowlist is enabled by default and accepts `172.168.208.2`; launch arguments can override it.
- Set `game_control_ip` in `brain/config/config.yaml` to the GameController address. Robot status packets are sent to UDP port 3939 at that address.

Status packets contain the fallen state, field pose in millimeters, ball age, and robot-frame ball position in millimeters.

## Usage
```
# Enter the workspace
> cd robocup_demo
# Build
> ./script/build.sh
# Run
> ./start_game_controller.sh
# Inspect the topic
> ros2 topic info -v /robocup/game_controller

# Temporarily disable the allowlist when using a different GameController network
> ros2 launch game_controller launch.py enable_ip_white_list:=false
```

After startup, `GameController status` reports received and accepted packet counts every five seconds.
If it repeatedly reports `no datagram received`, check `ip -brief addr`,
`ip route get <game-controller-ip>`, and `sudo tcpdump -ni any udp port 3838`
on the robot. This indicates a network interface, routing, or GameController transmission issue rather than a ROS topic issue.
