#!/bin/bash

# Add comment to mark script additions
echo "# --- PUMAS INSTALLATION SCRIPT ADDITIONS ---" >> /home/$USER/.bashrc

# Add the ROS2 workspace environment
echo "# Initialization of ROS2 workspace environment" >> /home/$USER/.bashrc
echo "source /home/$USER/Pumanoids/colcon_ws/install/setup.bash" >> /home/$USER/.bashrc

# Add bash functions to manage ROS2 launch files running as systemctl services
echo "# Bash functions for ROS2 launch as systemd services" >> /home/$USER/.bashrc
cat << 'EOF' >> /home/$USER/.bashrc

# Usage: robot <action> <service_name.service>
# Where <action> can be start, stop, restart, status, etc.
robot() {
    sudo systemctl "$1" "$2"
}
EOF

cat << 'EOF' >> /home/$USER/.bashrc

# Usage: robotlog <service_name.service>
robotlog() {
    journalctl -u "$1" -f -o cat
}
EOF