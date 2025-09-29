#!/bin/bash
set -e

echo "🚀 Setting up Drone Demo Project..."

# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS2 deps
sudo apt install -y python3-colcon-common-extensions ros-humble-desktop \
    ros-humble-mavros ros-humble-mavros-extras

# Setup PX4
cd px4_ws
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash PX4-Autopilot/Tools/setup/ubuntu.sh

echo "✅ Setup complete!"
