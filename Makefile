# Use bash instead of sh
SHELL := /bin/bash

# Define the relative paths to project within main_folder
PUMANOIDS_PROJECT_PATH := $(CURDIR)/colcon_ws
SIM_PROJECT_PATH := $(CURDIR)/simulation_ws
T1_PROJECT_PATH := $(CURDIR)/t1_ws
# Define the launch file

# Default target to clean, build, start roscore, and launch both projects
all: build source

# Rule to clean project's build and devel folders
clean:
	@echo "Cleaning All Projects and Workspaces..."
	@sleep 1
	@echo "Cleaning colcon_ws..."
	@rm -rf $(PUMANOIDS_PROJECT_PATH)/build $(PUMANOIDS_PROJECT_PATH)/install $(PUMANOIDS_PROJECT_PATH)/log
	@sleep 1
	@echo "Cleaning simulation_ws..."
	@rm -rf $(SIM_PROJECT_PATH)/build $(SIM_PROJECT_PATH)/install $(SIM_PROJECT_PATH)/log
	@sleep 1
	@echo "Cleaning t1_ws..."
	@rm -rf $(T1_PROJECT_PATH)/build $(T1_PROJECT_PATH)/install $(T1_PROJECT_PATH)/log

# Rule to build project
build:
	@echo "Building Pumanoids..."
	@echo "cd colcon_ws && colcon build --packages-skip new_ball_detector"
	@sleep 2
	@cd $(PUMANOIDS_PROJECT_PATH) && colcon build --packages-skip new_ball_detector

sim:
	@echo "Launching Project..."
	cd $(SIM_PROJECT_PATH) && colcon build --packages-skip gazebo_envs


testbag:
	@echo "Building for Booster T1..."
	cd $(PUMANOIDS_PROJECT_PATH) && source install/setup.bash
	cd $(T1_PROJECT_PATH) && source install/setup.bash
	ros2 launch particle_filter rosbag_localization.launch.py