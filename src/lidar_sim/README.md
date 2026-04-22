# Lite3 Lidar Simulation

This folder contains a modified MuJoCo model for the Lite3 robot with an integrated 8-beam horizontal lidar.

## Files

- `lite3_lidar.xml`: The MuJoCo model file.
- `meshes`: Symlink to the original robot meshes.
- `visualize_lidar.py`: A Python script to run the simulation and monitor lidar data.

## Lidar Configuration

The lidar is mounted on the front of the TORSO.
- **Sensor Type**: `rangefinder`
- **Number of Beams**: 8 (Horizontal scan, 45-degree increments)
- **Mount Position**: `[0.15, 0, 0.05]` relative to TORSO center.

## How to Run

### 1. Simple Visualization (MuJoCo Viewer)
Ensure you have `mujoco` installed:
```bash
pip install mujoco
```
Run:
```bash
python3 visualize_lidar.py
```

### 2. ROS2 & Rviz2 Visualization
Ensure you have a ROS2 environment sourced (e.g., Foxy, Humble).

1. Run the ROS2 bridge:
   ```bash
   python3 mujoco_ros2_bridge.py
   ```

2. In a NEW terminal, open Rviz2:
   ```bash
   rviz2
   ```

3. In Rviz2:
   - Change **Fixed Frame** to `odom`.
   - Add a **LaserScan** display and set the topic to `/scan`.
   - Add a **TF** display to see the robot's coordinate frames.
   - (Optional) Set LaserScan 'Style' to `Points` or `Boxes` and increase 'Size' for better visibility.
