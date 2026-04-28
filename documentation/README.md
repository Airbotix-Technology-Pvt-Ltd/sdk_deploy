# Master Integration Guide - Airbotix Fork

This document provides a detailed technical narrative of the **Lite3 RL & Navigation Development (Airbotix Fork)**. It tracks our architectural progression from locomotion synchronization to high-level perception.

---

## 📊 Project Roadmap & Milestones

### **Phase 1: Foundation (COMPLETED)**
- [x] **Locomotion Sync**: Zero-latency C++ bridge with simulation-time awareness.
- [x] **Simulation Logic Setup**: Following NVIDIA's ROS 2 RL Controller standards.
- [x] **Perception Suite (LiDAR/Depth)**: Integrated 360 Lidar and Depth camera for PointCloud2 streams.
- [x] **Spatial Frames (TF Tree)**: Established the full `odom -> base_link -> sensors` transform hierarchy.

### **Phase 2: Intelligent Navigation (COMPLETED ✅)**
- [x] **Fast-LIO SLAM**: High-performance 3D LiDAR Odometry integrated via `/Odometry` and `/cloud_registered` topics.
- [x] **Nav2 Path Planning**: Costmap, DWB planner, and BT navigator configured with `/point_cloud` sensor source.
- [x] **P2P Goal Interface**: Nav2 `/cmd_vel` connected to RL policy velocity interface via P-key toggle in `keyboard_interface.hpp`.
- [x] **Isaac Sim TF Fix**: Resolved broken `odom → base_link` TF by tracking `TORSO` world pose directly (Compute Odometry failed due to articulation mismatch). Full chain: `map → odom → base_link → Lite3 → TORSO → legs`.

### **Phase 3: Real-World Deployment (PENDING)**
- [ ] **Hardware Transfer**: Transitioning from Isaac Sim to physical Lite3 hardware via the transfer layer.
- [ ] **Autonomous Navigation Tests**: Validating P2P success in complex indoor environments.

---

## 🚀 How to Run the Full Stack

### Simulation (MuJoCo)
Starts the physics engine and ROS2 bridge for Lidar and IMU.
```bash
# Complex House environment (High-fidelity)
python3 ./src/Lite3_sdk_deploy/interface/robot/simulation/mujoco_simulation_lidar_ros2.py --env big --navigation

# Small Room environment (Performance mode)
python3 ./src/Lite3_sdk_deploy/interface/robot/simulation/mujoco_simulation_lidar_ros2.py --env small --navigation
```

**2. Start the Lite3 RL Locomotion Policy:**
```bash
source install/setup.bash
ros2 run lite3_sdk_deploy rl_deploy
```

**3. Start FAST-LIO SLAM:**
```bash
source install/setup.bash
ros2 launch fast_lio mapping.launch.py use_sim_time:=true config_file:=xt32.yaml rviz:=false
```

**4. Start Nav2 Navigation:**
```bash
source install/setup.bash
ros2 launch launch/lite3_bringup.launch.py \
   use_sim_time:=true \
   map:=$(pwd)/map/big_area/map.yaml \
   params_file:=$(pwd)/config/nav2_lite3_params.yaml \
   use_composition:=False \
   use_respawn:=False
```


---

## 🛠️ Core Components

| Component | Role |
|-----------|------|
| **MuJoCo** | The simulation environment (Physics, Sensors). |
| **Lite3 SDK** | The master control platform. Runs the RL policy (`.onnx` model). |
| **Nav2** | The ROS2 Navigation stack used for autonomous path planning. |


---

## 📈 Active & Pending Milestone Targets

### **Ongoing (Active Research): Real-World Deployment**
- [ ] **Hardware Transfer**: Transitioning from simulation-trained policies to physical Lite3 hardware.
- [ ] **Autonomous Navigation Tests**: Validating P2P success in complex indoor environments.

---


---

## 🏆 Key Breakthrough: Bridging the Sim2Sim Domain Gap

### The Deployment Jitter Problem
During initial deployments, ML models trained in IsaacLab using standard `TerrainGeneratorCfg` procedural stairs exhibited massive instability and violent "jittering" when exported and run via the `isaac_bridge` inside Isaac Sim. 

### The Solution: Direct CAD Integration
We discovered that the "domain gap" was caused purely by collision geometry discrepancies between mathematically perfect procedural planes in training versus the highly-detailed 3D triangulation in deployment. 

**Our golden rule for training:** We completely stripped out the procedural generator in our RL config and replaced it with a `TerrainImporterCfg` specifically pointing to our deployment CAD `.usda` mesh:
```python
self.scene.terrain = TerrainImporterCfg(
    prim_path="/World/ground",
    terrain_type="usd",
    usd_path="/Path/To/deployment_mesh.usda",
)
```
By forcing the RL Agent to train against the exact collision triangulation, friction, and mesh physics it will see in deployment, the Sim2Sim gap dropped to zero. The resulting model deploys flawlessly into Isaac Sim with zero jitter.

---

## 💾 Replication & Documentation Reference
- **Master Simulation File**: [mujoco_simulation_lidar_ros2.py](../src/Lite3_sdk_deploy/interface/robot/simulation/mujoco_simulation_lidar_ros2.py)
- **SDK Service Guide**: [README_lite3_sdk_service.md](README_lite3_sdk_service.md) (Original DeepRobotics tech specs).
- **Nav2 Params**: [nav2_lite3_params.yaml](../config/nav2_lite3_params.yaml)

---

## ❤️ Credits & Tribute
We pay tribute and express our sincere gratitude to **DeepRobotics** for providing the foundational Lite3/M20 SDK and hardware interfaces. Their original work is the baseline upon which we built our simulation-to-reality pipeline. 

---
*Developed by Airbotix Technology Pvt Ltd for Lite3 Locomotion Research.*
*Sumit Bhardwaj ([@smtbhd32-ABX](https://github.com/smtbhd32-ABX))* | *Last updated: 2026-04-01*
