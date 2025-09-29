# UAV Simulation: PX4 + Gazebo + QGroundControl

A comprehensive drone simulation environment integrating PX4 SITL, Gazebo, QGroundControl, MAVSDK (Python), and ROS2. This repository enables students, researchers, and developers to experiment with UAV missions, autonomous flight, and robotics workflows.

---

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Project Structure](#project-structure)
- [Demo](#demo)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)
- [Author](#author)

---

## Overview

This repository demonstrates a complete UAV simulation pipeline:

- **PX4 SITL** for software-in-the-loop drone simulation.
- **Gazebo** as a 3D robotics simulator.
- **QGroundControl** for mission planning and monitoring.
- **MAVSDK (Python)** for automated mission scripting.
- **ROS2 Humble** for robotic integration and extension.

Intended for learning, research, and rapid prototyping in UAV autonomy.

---

## Features

- ✅ PX4 SITL drone simulation in Gazebo
- ✅ Automated mission planning via MAVSDK (Python)
- ✅ ROS2 integration ready
- ✅ Modular and extensible project structure
- ✅ Included demo scripts and sample missions

---

## Project Structure

```
UAV-simulation-Gazebo-Q-Ground/
├── PX4-Autopilot/        # PX4 source (cloned)
├── ros2_ws/              # ROS2 workspace
├── mission/              # Mission scripts
├── simple_mission.py     # Example MAVSDK mission
├── install.sh            # Dependency installer
├── run_demo.sh           # Quick demo script
├── Demo video/           # Example results
├── requirements.txt      # Python dependencies
└── README.md             # Documentation
```

---

## Demo

- [Demo.webm](https://github.com/user-attachments/assets/1226f11a-e616-4e0d-8b4b-ce15ad755248)
- ![Q_Ground_mission_plan](https://github.com/user-attachments/assets/1072de83-e2fb-4aa9-b219-26af9198e828)
- <img width="800" alt="Gazebo_Quad_Drone" src="https://github.com/user-attachments/assets/b1e1d1a7-0b04-4e92-926f-108304f3a3d0" />

---

## Requirements

- Ubuntu 20.04 / 22.04 (recommended)
- Python 3.8+
- PX4 Autopilot
- Gazebo Garden
- MAVSDK-Python
- ROS2 Humble
- Disk space: 30+ GB

---

## Installation

> **Tip:** For Ubuntu 24.04 (Noble), some ROS/Gazebo packages may require manual tweaks. Docker installation is recommended for compatibility.

**1. Clone the repository:**
```bash
git clone https://github.com/hamzabasharat26/UAV-simulation-Gazebo-Q-Ground.git
cd UAV-simulation-Gazebo-Q-Ground
```

**2. Install dependencies:**
```bash
chmod +x install.sh
./install.sh
```

**3. Set up Python environment:**
```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

**4. Download PX4 Autopilot:**
```bash
mkdir -p px4_ws
cd px4_ws
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```

---

## Usage

**1. Run PX4 SITL with Gazebo:**
```bash
cd px4_ws/PX4-Autopilot
make px4_sitl gz_x500
```

**2. Run a sample MAVSDK mission:**
```bash
cd <repo-root>
source venv/bin/activate
python3 simple_mission.py
```

---

## Troubleshooting

- Ensure all dependencies are installed (see `requirements.txt`).
- For ROS2/Gazebo issues on Ubuntu 24.04, consult [ROS2 Humble documentation](https://docs.ros.org/en/humble/Installation.html) and [Gazebo Garden docs](https://gazebosim.org/docs).
- If you encounter `make` or build errors, verify PX4 and Gazebo versions.

---

## Contributing

Contributions are welcome! Please open issues or submit pull requests for improvements, bug fixes, or new features.

---

## License

This project is licensed under the [MIT License](LICENSE). You are free to use and modify the code for your own UAV research and projects.

---

## Author

**Hamza Basharat**  
Beginner-friendly, autonomous drone simulation.  
Contact: [GitHub Profile](https://github.com/hamzabasharat26)

---
