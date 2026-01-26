# Booster IK dependencies

sudo apt install -y \
ros-jazzy-moveit \
ros-jazzy-moveit-setup-assistant \
ros-jazzy-ros2-control \
ros-jazzy-ros2-controllers \
ros-jazzy-joint-state-publisher-gui \
ros-jazzy-xacro

# Run test

ros2 launch booster_ik demo.launch.py

Don´t forget to do colcon build and source before