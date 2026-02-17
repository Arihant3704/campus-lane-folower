# Autonomous Maize Farm Navigation

This project simulates an autonomous robot navigating a maize farm in Gazebo using ROS 2. The robot uses GPS for waypoint navigation and LiDAR for obstacle avoidance.

## Project Overview

- **Environment**: A generated maize field with 6 rows of crops and a grass ground plane.
- **Robot**: A 4-wheeled differential drive robot equipped with:
    - **GPS**: Simulated to provide latitude/longitude.
    - **LiDAR**: Used for obstacle detection and avoidance.
    - **IMU/Odom**: Used for orientation (yaw).
- **Navigation**:
    - **Smart Gap Path**: Reads `crop_coordinates.csv` to snake through crop row gaps (Top-Right -> Bottom-Left).
    - **Collision Recovery**: Implements "Reverse and Turn" logic if the robot gets too close.
    - **GPS & LiDAR**: Combines global waypoints with local obstacle avoidance.

## Prerequisites

- **ROS 2 Humble** (or compatible)
- **Gazebo Classic**
- **Python 3**
- Key Packages: `gazebo_ros`, `robot_state_publisher`, `xacro`

## Installation

1.  **Clone/Copy the workspace**: Ensure `farm_sim`, `virtual_maize_field`, and `car_simulation` packages are in `src/`.
2.  **Install Dependencies**:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```
3.  **Build**:
    ```bash
    colcon build
    source install/setup.bash
    ```
4.  **Model Setup** (Important):
    Ensure the `virtual_maize_field` models are accessible to Gazebo. The `setup.py` and launch file handle this, but you can also copy them manually to `~/.gazebo/models` if issues persist.

## Running the Simulation

1.  **Launch the Simulation**:
    ```bash
    source install/setup.bash
    ros2 launch farm_sim farm_simulation.launch.py
    ```
    - This opens Gazebo.
    - Spawns the maize field and the robot at the starting corner (-24, -24).
    - Starts the `serpentine_navigator` node.

2.  **Monitor the Robot**:
    - The terminal will show navigation status:
        - `[MOVING] Dist: ...`
        - `Scan: Min Front Dist = ...`
    - If the robot hits a crop, it will log `Obstacle detected! Starting Recovery: REVERSING`.

## Data Output

- **`robot_waypoints.csv`**: Generated in the directory where you launch the simulation. Contains the planned path.
- **`crop_coordinates.csv`**: Source data located in `src/farm_sim/config/`. Define the crop positions.

## Troubleshooting

- **Robot spins in circles**: Check if the target waypoint is too close or if GPS coordinates are fluctuating.
- **Robot hits crops**: The collision box for maize has been inflated (0.25m) to ensure detection. If it still hits, the robot might be moving too fast (`vehicle_speed` in launch file).
- **"Waiting for GPS fix..."**: Ensure the Gazebo simulation is running (not paused).
- **Robot driving away from crops**: The GPS coordinate system might be inverted relative to the Gazebo world. The `navigation_node.py` handles this by inverting signs (`dx = -...`, `dy = -...`). If you change the spawn orientation, you might need to adjust this.
