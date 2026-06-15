# Use bash instead of sh
SHELL := /bin/bash

# Define the relative paths to project within main_folder
PUMANOIDS_PROJECT_PATH := $(CURDIR)/colcon_ws
# Define the launch file

# Default target to clean, build, start roscore, and launch both projects
all: build source

# Rule to clean project's build and devel folders
clean:
	@echo "Cleaning Project..."
	sleep 2
	rm -rf $(PUMANOIDS_PROJECT_PATH)/build $(PUMANOIDS_PROJECT_PATH)/install $(PUMANOIDS_PROJECT_PATH)/log

# Rule to build project
build:
	@echo "Building Project..."
	cd $(PUMANOIDS_PROJECT_PATH) && colcon build
	sleep 2

# Rule to launch project's node
source:
	@echo "Launching Project..."
	source $(PUMANOIDS_PROJECT_PATH)/install/setup.bash

pi:
	@echo "Building Project..."
	cd $(PUMANOIDS_PROJECT_PATH) && colcon build --packages-skip gazebo_envs

sim:
	@echo "Launching Project..."
	cd $(PUMANOIDS_PROJECT_PATH) && colcon build --packages-skip boosterk1_image_proc

g1sim:
	@echo "Launching Project..."
	source $(PUMANOIDS_PROJECT_PATH)/install/setup.bash
	ros2 launch surge_et_ambula g1_sim_humble.launch.py

g1:
	@echo "Launching ..."
	cd $(PUMANOIDS_PROJECT_PATH) && colcon build --packages-skip boosterk1_image_proc gazebo_envs dynamixel

startg1:
	@echo "Launching ..."
	source $(PUMANOIDS_PROJECT_PATH)/install/setup.bash
	ros2 launch surge_et_ambula g1_state_machine.launch.py

k1:
	@echo "Building for Booster K1..."
	cd $(PUMANOIDS_PROJECT_PATH) && colcon build --packages-skip gazebo_envs dynamixel new_twist_to_t1 social_navigation

t1:
	@echo "Building for Booster T1..."
	cd $(PUMANOIDS_PROJECT_PATH) && colcon build --packages-skip gazebo_envs dynamixel new_twist_to_k1 social_navigation boosterk1_image_proc
