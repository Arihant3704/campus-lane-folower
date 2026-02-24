# Autonomous Campus Vehicle & Lane Following Simulation

A production-grade ROS 2 autonomous vehicle simulation system. This project features a mathematically accurate "Digital Twin" of a college campus, real-time computer vision for lane keeping, and a premium web-based remote cockpit for manual/autonomous control.

##  File Location
The main entrance to the project documentation is located at:
**`/home/arihant/ros2_ws/README.md`**

## �🚀 System Architecture

- **Simulated Environment:** A precise 1:1 scale recreation of a campus using OpenStreetMap (OSM) data.
- **Perception (Computer Vision):** A high-aerial Dual-Boundary lane detection pipeline. It uses an Intel RealSense camera mounted at 5m height with a 90° FOV to capture road markings (Yellow/White) across 50% of the image.
- **Control (Brain):** A 4-Wheel Skid Steer Proportional Controller with adaptive speed management (maintains momentum on sharp turns) and 200Nm of high-torque physics.
- **Web Interface:** A premium "Agribot" cockpit with an analog virtual joystick, real-time MJPEG streaming, and a HUD overlay for live correction telemetry.

---

## 📂 Project Structure

- `src/lane_simulation`: The Gazebo world, URDF models (4-wheel drive Ebot robot), and launch configurations.
- `src/lane_detection`: Core Python logic for the autonomous lane following algorithm.
- `remote_control_gui.py`: The Web-based cockpit application (Flask + ROS 2).
- `tools/`: Utility scripts for environment generation.
  - `tools/parse_osm_road.py`: Generates high-resolution (10px/m) OSM-based road textures.
  - `tools/osm_to_gazebo_buildings.py`: Procedurally extrudes 3D buildings from OSM data.

---

## 🛠 Prerequisites

- **OS:** Ubuntu 22.04
- **ROS 2:** Humble Hawksbill
- **Simulator:** Gazebo (Classic)
- **Python Dependencies:**
  ```bash
  pip install Flask opencv-python numpy
  ```

---

## 🚦 Getting Started

### 1. Build & Source
```bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch the Simulation
Starts the campus world with the Ebot robot spawned at the optimized starting position (-9.2, -126.3):
```bash
ros2 launch lane_simulation synthetic_sim.launch.py
```
*Note: You can override the spawn point using `x:=val y:=val yaw:=val` arguments.*

### 3. Start the Control Systems
In a new terminal, launch the vision brain and the web cockpit:
```bash
# Terminal 2: Start Lane Following Brain
ros2 run lane_detection lane_keeper

# Terminal 3: Start Web Cockpit
python3 remote_control_gui.py
```

### 4. Access the Cockpit
Open your browser and navigate to:
**`http://localhost:5000`** (or your machine's IP address on the network).

---

## 🧠 Technical Highlights
1. **Dual-Boundary Perception:** Detects both yellow side-lines and white center-dashes separately. It can navigate even if one boundary is obscured by offsetting from the visible line.
2. **High-Torque Physics:** Configured with specific `mu1`/`mu2` friction coefficients to allow realistic skid-steering "drifts" and pivoting on different road surfaces.
3. **Adaptive Drive Mode:** Automatically reduces speed to ~75% during sharp turns to maintain stability, while maximizing speed on straights.

## 📡 Remote Control Features

https://github.com/user-attachments/assets/d5d600b5-5068-4062-a823-05bac59b9b67


- **Analog Navigator:** Premium virtual joystick (powered by `nipplejs`) providing 360° fluid control and variable speed.
- **Real-Time HUD:** Live overlay on the camera feed showing Detection Mode (Both/Left/Right), Error (px), Steer magnitude, and an analog steering-bar.
- **Drone-Perspective:** Aerial camera view (5m altitude) providing superior visibility for lane navigation.
- **Emergency Stop:** Integrated safety cut-off to halt all robot movement immediately.

---




## 📜 License
This project is licensed under the MIT License. Developed for educational-grade autonomous vehicle research.
