set -e

sudo apt-get remove -y unattended-upgrades
sudo apt-get install i2c-tools

sudo cp $(git rev-parse --show-toplevel)/setup/raspberry/80-humanoid.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger

echo "source ~/Humanoids/catkin_ws/devel/setup.bash" >> ~/.bashrc

echo "Finished setting up raspberry"