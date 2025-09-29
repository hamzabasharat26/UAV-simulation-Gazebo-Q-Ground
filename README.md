# UAV-simulation-Gazebo-Q-Ground
# Drone Demo Project — PX4 + Gazebo + MAVSDK + (optional ROS2)

**Author:** Hamza  
**Goal:** Beginner-friendly, internship-ready autonomous drone demo in simulation.  
This repo shows a complete pipeline: PX4 SITL + Gazebo simulation → automated mission via MAVSDK (Python). Optional ROS2 integration is provided.

---

![Q_Ground_mission_plan](https://github.com/user-attachments/assets/1072de83-e2fb-4aa9-b219-26af9198e828)

## What this repo contains
- `simple_mission.py` — MAVSDK Python mission (takeoff → to finish line → land)
- `ros/` — optional ROS2 package scaffold with a mission node + launch file
- `px4_ws/` — helper script to clone PX4 (not included as submodule due to size)
- `run_demo.sh` — convenience script to run the demo
- `install.sh` — automated dependency install (Ubuntu)
- `Dockerfile` — optional container for running mission without touching host setup
- `docs/` — step-by-step instructions & troubleshooting
- `media/` — screenshot / demo assets (you add demo GIF/video here)

---

[Demo.webm](https://github.com/user-attachments/assets/1226f11a-e616-4e0d-8b4b-ce15ad755248)


## Quick native setup (Ubuntu)
> Recommended: use an Ubuntu machine with enough disk (30+ GB free). If you only have 24.04 (Noble), the repo works but some ROS/Gazebo packages may need tweaks — Docker path is safest.

<img width="1658" height="897" alt="Gazebo_Quad_Drone" src="https://github.com/user-attachments/assets/b1e1d1a7-0b04-4e92-926f-108304f3a3d0" />

![Mission](https://github.com/user-attachments/assets/2264fd44-e603-4b3c-9cb4-72a94615f93d)

1. Clone this repo:
```bash
git clone https://github.com/hamzabasharat26/UAV simulation Gazebo & Q-Ground.git
cd drone-demo-project

2. Install dependencies (native):
```bash
chmod +x install.sh
./install.sh

3. Python3 -m venv venv:
```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

4. Get PX4 (one-time, large download; ~1.5–3GB depending on branch):
```bash
mkdir -p px4_ws
cd px4_ws
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
# PX4 setup script (may take a while)
bash ./Tools/setup/ubuntu.sh


5. Run PX4 SITL with Gazebo(Garden):
```bash
cd px4_ws/PX4-Autopilot
make px4_sitl gz_x500

6. Run mission:
```bash
cd <repo root>
source venv/bin/activate
python3 simple_mission.py


