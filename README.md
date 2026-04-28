# Lite3 Bridge & SDK Deployment (Airbotix Fork)

This workspace contains the core ROS 2 and C++ infrastructure for **High-Fidelity Simulation-to-Reality Deployment** of RL policies on the Lite3 quadruped platform.

---

> [!IMPORTANT]
> **Airbotix is exclusively focused on the Lite3 platform.** 
> While foundational code for other platforms (M20) is preserved as legacy reference, our research is dedicated to achieving state-of-the-art results on the Jueying Lite3.

---

**Official Airbotix Source**: [Airbotix-Technology-Pvt-Ltd/Lite3_sdk_deploy](https://github.com/Airbotix-Technology-Pvt-Ltd/Lite3_sdk_deploy)

---

### 🌐 Project Central Hubs
- [**Master Hub (Root)**](https://github.com/Airbotix-Technology-Pvt-Ltd/Lite3Robot): Mission, specialized workspaces, and organizational identity.
- [**Master Integration Guide**](../documentation/README.md): **Single Source of Truth** for technical milestones, roadmaps, and reproduction steps.
- [**Contributors Hub**](../Contributors.md): Full technical attribution for the Airbotix development team.

---

## 📋 Prerequisites & Tooling
To reproduce our high-fidelity SLAM results, ensure the following specialized Technical dependencies are installed:
- **`pcl-tools`**: Professional PCL utilities for map inspection and format conversion.
  ```bash
  sudo apt update && sudo apt install -y pcl-tools
  ```
- **`FAST_LIO_ROS2`**: Optimized, hardware-agnostic SLAM stack for ROS 2 Humble.
- **Python Utilities**: Standard **`numpy`** and **`opencv-python`** for 2D map projection.

---

## 🛠️ Workspace Components

### **1. `isaac_bridge` (NVIDIA Isaac Sim Integration)**
The heart of our simulation infrastructure. Re-engineered as a zero-latency hub synchronized to the simulation `/clock`, ensuring stable locomotion and PD alignment between training and deployment.

### **2. `lite_transfer` (Hardware Bridging)**
The C++ communication layer between simulation-trained policies and physical hardware, using the deep-rooted M20/Lite3 UDP protocols.

### **3. `Perception Suite` (LiDAR/Depth)**
Integrated 360 Lidar and Depth camera stack for PointCloud2 streams and spatial TF frames (`odom -> base_link -> sensors`).

### **4. `Fast-LIO SLAM` (3D LiDAR Navigation)**
Integrated a high-performance, optimized version of **FAST_LIO_ROS2** for the Lite3. Stripped of hardware-specific dependencies (`livox_ros_driver2`) to ensure universal simulation and physical LiDAR compatibility.

### **5. `Mapping & Navigation Artifacts`**
Generated a high-fidelity **2D Occupancy Grid Map** (`map.pgm`/`map.yaml`) for Nav2 integration, derived from our globally registered 3D pointcloud maps using the specialized **`pcd_to_grid.py`** projection utility.

---

## 🚀 Navigation & SLAM Architecture

### 1. TF & Odometry Logic
Our system uses a dual-mode strategy to ensure clean TF trees:
- **Mapping Mode**: Run MuJoCo **without** the `--navigation` flag. The simulation will publish the `odom -> base_link` ground truth transform for raw mapping.
- **Navigation Mode**: Run MuJoCo **with** the `--navigation` flag. This **suppresses** the internal simulation TF, allowing **FAST-LIO** to own the `map -> odom -> base_link` transform. This prevents "dual-authority" jitter in the TF tree.

### 2. Sensor Pipeline
- **FAST-LIO Integration**: We use FAST-LIO for high-fidelity 3D odometry.
- **Pointcloud to Scan**: Added a `pointcloud_to_laserscan` node within the FAST-LIO launch to project 360° 3D data into a 2D `/scan` topic, which Nav2 uses for global/local costmap obstacle avoidance.

### 3. Controller Server (Smooth Motion)
We have shifted from the default DWB controller to **Regulated Pure Pursuit (RPP)**. This provides significantly smoother, "curvy" paths that are better suited for the quadruped's gait compared to the jerky movements of DWA/DWB.

### 4. Footprint & Self-Hit Avoidance
To prevent the robot from considering its own chassis as an obstacle:
- **Footprint Clearing**: Enabled `footprint_clearing_enabled: True` in both global and local costmaps.
- **Chassis Masking**: The robot's rectangular footprint `[[0.225, 0.14], ...]` is strictly defined to mask the Lidar's "blind zone," ensuring zero false-positive collisions from the robot's own body.

---

## 💻 Running Navigation (Home Simulation)

Follow these steps to launch the full Lite3 navigation stack (FAST-LIO + Nav2) in the multi-room home environment.

### 1. Launch the Simulation (MuJoCo)
Starts the physics engine and ROS2 bridge for Lidar and IMU.
```bash
python3 src/Lite3_sdk_deploy/interface/robot/simulation/mujoco_simulation_lidar_ros2.py --navigation --ros-args -p use_sim_time:=true
```

### 2. Start SLAM (FAST-LIO)
Provides high-fidelity odometry and mapping (`map -> odom`).
```bash
ros2 launch fast_lio mapping.launch.py config_file:=xt32.yaml rviz:=false use_sim_time:=true
```

### 3. Start Navigation (Nav2)
Starts the path planning and controller stack using our **custom mission file** which excludes AMCL to avoid conflicts with FAST-LIO.
```bash
ros2 launch launch/lite3_bringup.launch.py \
  map:=$(pwd)/map.yaml \
  params_file:=$(pwd)/nav2_lite3_params.yaml \
  use_sim_time:=true
```

---
*Airbotix Technology Pvt Ltd - Lite3 P2P Autonomous Navigation Project*
