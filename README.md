#Project Overview

This project provides a complete drone simulation environment integrating:
It is designed for students, researchers, and developers to experiment with UAV missions, autopilot, and robotics workflows.

**Author:** Hamza  
**Goal:** Beginner-friendly, internship-ready autonomous drone demo in simulation.  
This repo shows a complete pipeline: PX4 SITL + Gazebo simulation → automated mission via MAVSDK (Python).ROS2 integration is provided.

---

![Q_Ground_mission_plan](https://github.com/user-attachments/assets/1072de83-e2fb-4aa9-b219-26af9198e828)

📂 Project Structure
UAV-simulation-Gazebo-Q-Ground/
│── PX4-Autopilot/        # PX4 source (cloned)
│── ros2_ws/              # ROS2 workspace
│── mission/              # Mission scripts
│── simple_mission.py     # Example MAVSDK mission
│── install.sh            # Dependency installer
│── run_demo.sh           # Quick demo script
│── Demo video/           # Example results
│── requirements.txt      # Python dependencies
└── README.md             # Documentation
---

[Demo.webm](https://github.com/user-attachments/assets/1226f11a-e616-4e0d-8b4b-ce15ad755248)


## Quick native setup (Ubuntu)
> Recommended: use an Ubuntu machine with enough disk (30+ GB free). If you only have 24.04 (Noble), the repo works but some ROS/Gazebo packages may need tweaks — Docker path is safest.

<img width="1658" height="897" alt="Gazebo_Quad_Drone" src="https://github.com/user-attachments/assets/b1e1d1a7-0b04-4e92-926f-108304f3a3d0" />

![Mission](https://github.com/user-attachments/assets/2264fd44-e603-4b3c-9cb4-72a94615f93d)

🚀 Getting Started
1️⃣ Clone this repository
git clone "https://github.com/hamzabasharat26/UAV-simulation-Gazebo-Q-Ground.git"
cd UAV-simulation-Gazebo-Q-Ground

2️⃣ Install dependencies (native)
chmod +x install.sh
./install.sh

3️⃣ Setup Python virtual environment
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

4️⃣ Get PX4 (large download, one-time setup)
mkdir -p px4_ws
cd px4_ws
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh

5️⃣ Run PX4 SITL with Gazebo (Garden)
cd px4_ws/PX4-Autopilot
make px4_sitl gz_x500

6️⃣ Run a sample MAVSDK mission
cd <repo-root>
source venv/bin/activate
python3 simple_mission.py

🎯 Features

✅ Full PX4 SITL simulation in Gazebo
✅ MAVSDK-based mission planning (Python)
✅ ROS2 integration ready
✅ Modular structure for easy extension
✅ Demo mission included

📹 Demo

(Insert screenshots / link to your demo video here)

📌 Requirements

Ubuntu 20.04 / 22.04 (recommended)

Python 3.8+

PX4 Autopilot

Gazebo Garden

MAVSDK-Python

(Optional) ROS2 Humble

📜 License

This project is licensed under the MIT License – feel free to use and modify for your own UAV research and projects.
