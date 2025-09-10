# Software for the Pumas Humanoid Robot

This repository contains the software developed for the humanoid robot of the Biorrobotics Lab.

![Team photo](/Documentation/Images/team.jpg)

## Getting Started

### Prerequisites

* [Ubuntu 20.04](https://releases.ubuntu.com/focal/)
* [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) (`ros-noetic-desktop-full` required)
* Gazebo (included in `ros-noetic-desktop-full`)
* RViz (included in `ros-noetic-desktop-full`)
* A lot of patience

## Installation

```sh
 git clone https://github.com/mnegretev/Humanoids.git 
 cd Humanoids
 cd setup/workspace/
 ./SetupDraft.sh
 cd catkin_ws
 catkin_make -j1 -l1
```
### Testing in Desktop
```sh
 source devel/setup.bash
 roslaunch surge_et_ambula humanoid_simul.launch
```
Then, in Gazebo click `Play` down below.

![Humanoid photo](/Documentation/Images/launch_surge_desktop.png)
---
### Testing in Humanoid
```sh
source devel/setup.bash
roslaunch surge_et_ambula humanoid_hardware.launch
```
---
### Configure remote ssh to Raspberry PI

<details><summary>First Method (Static IP with Router)</summary>

To setup a static ip in the raspberry Ethernet port where you can connect to a router, run:
```sh
cd setup/raspberry/
sudo cp 01-static-ip.yaml /etc/netplan/01-static-ip.yaml
sudo netplan apply
```
The static IP is now set to `192.168.0.10`. You can now do `ssh humanoid@192.168.0.10` to connect with Ethernet cable directly to the humanoid.

</details>

<details><summary>Second Method (Static IP with no router)</summary>

If you want to connect directly to Raspberry Pi through Ethernet and share internet at the same time, go to `Settings -> Network -> Wired`, the open settings. In `IPv4` tab, select `Shared to other computers`

![Humanoid photo](/Documentation/Images/network_config.png)

```sh
cd setup/raspberry/
sudo cp 01-static-ip.yaml /etc/netplan/02-static-ip.yaml
sudo netplan apply
```
The static IP is now set to `10.42.0.2`. You can now do `ssh humanoid@10.42.0.2` to connect with Ethernet cable directly to the humanoid.

</details>