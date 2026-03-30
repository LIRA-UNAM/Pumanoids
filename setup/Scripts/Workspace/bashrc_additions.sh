#!/bin/bash

# Add the ROS2 workspac environment
echo "source /home/$USER/Pumanoids/colcon_ws/install/setup.bash" >> /home/$USER/.bashrc

# Add bash functions for ROS2 launch files running as systemctl services
cat << 'EOF' >> /home/$USER/.bashrc

robot() {
    sudo systemctl "$1" "$2"
}
EOF

cat << 'EOF' >> /home/$USER/.bashrc

robotlog() {
    journalctl -u "$1" -f -o cat
}
EOF