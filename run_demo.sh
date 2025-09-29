#!/bin/bash
# Run PX4 SITL + Gazebo + ROS2 mission

# Terminal 1: PX4 SITL
gnome-terminal -- bash -c "cd px4_ws/PX4-Autopilot && make px4_sitl gz_sitl; exec bash"

# Terminal 2: ROS2 launch
sleep 5
gnome-terminal -- bash -c "cd ros2_ws && source install/setup.bash && ros2 launch drone_demo demo.launch.py; exec bash"

#!/usr/bin/env bash
set -e
ROOT="$(cd "$(dirname "$0")" && pwd)"

# Terminal 1: run PX4 SITL gz_x500
gnome-terminal -- bash -c "cd $ROOT/px4_ws/PX4-Autopilot && make px4_sitl gz_x500; exec bash" || \
{ echo "Could not open gnome-terminal; run PX4 SITL manually: cd px4_ws/PX4-Autopilot && make px4_sitl gz_x500"; exit 1; }

# wait for PX4 to boot
sleep 8

# Terminal 2: run mission in venv
gnome-terminal -- bash -c "cd $ROOT && source venv/bin/activate && python3 simple_mission.py; exec bash" || \
{ echo "Run the mission manually: source venv/bin/activate && python3 simple_mission.py"; exit 1; }
