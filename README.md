# Lite3 RL & Navigation Development (Airbotix Fork)

This repository is a specialized fork of the DeepRobotics Lite3/M20 SDK, significantly enhanced and rebuilt for **High-Fidelity Simulation-to-Reality RL Deployment** and **Autonomous Robotic Navigation**.

---

## 📊 Project Roadmap & Milestones

### **Phase 1: Foundation (COMPLETED)**
- [x] **Locomotion Sync**: Zero-latency C++ bridge with simulation-time awareness.
- [x] **Simulation Logic Setup**: Following NVIDIA's ROS 2 RL Controller standards.
- [x] **Perception Suite (LiDAR/Depth)**: Integrated 360 Lidar and Depth camera for PointCloud2 streams.
- [x] **Spatial Frames (TF Tree)**: Established the full `odom -> base_link -> sensors` transform hierarchy.

### **Phase 2: Intelligent Navigation (ONGOING - ACTIVE TARGET)**
- [ ] **Fast-LIO SLAM**: Integrating high-performance 3D LiDAR Odometry and Mapping.
- [ ] **Nav2 Path Planning**: Configuring the costmap and global planner using the SLAM map.
- [ ] **P2P Goal Interface**: Connecting Nav2 goals directly to the RL policy's velocity interface.

### **Phase 3: Real-World Deployment (PENDING)**
- [ ] **Hardware Transfer**: Transitioning from Isaac Sim to physical Lite3 hardware via the transfer layer.
- [ ] **Autonomous Navigation Tests**: Validating P2P success in complex indoor environments.

---

## 🚀 Key Enhancements in this Fork

Unlike the standard SDK, this fork has been engineered to bridge the gap between RL training and high-fidelity simulation in NVIDIA Isaac Sim:

- **Zero-Latency Sim-Sync Bridge**: Transformed the core `isaac_bridge` into a zero-latency hub synchronized to the simulation `/clock`.
- **Locomotion Stability & PD Alignment**: Re-engineered the bridge to compute local PD efforts, matching MuJoCo training physics and achieving jitter-free movement.
- **Multi-Modal Perception Suite**: Successfully integrated LiDAR (PointCloud2) and Depth Camera sensors.
- **Simulator Startup Safety**: Implemented a custom "Settlement Filter" and high-gain "Freeze Pose" mechanism.

---

## 📖 Project Documentation & Reference Hub

Everything you need to replicate this technical journey is consolidated here:

- [**documentation/README.md**](documentation/README.md): **Master Integration Guide**. Detailed milestones, calibration logic, and reproduction steps.
- [**documentation/README_lite3_sdk_service.md**](documentation/README_lite3_sdk_service.md): **Original SDK Service Guide**. Foundational technical documentation from DeepRobotics.
- [**documentation/isaac_action_graph/**](documentation/isaac_action_graph/): **Visual Gallery**. Screenshots of all internal simulation node wiring.

---

## 🛠️ Project Identity & Ownership

- **Organization**: Airbotix Technology Pvt Ltd.
- **Lead Developer**: **Sumit Bhardwaj** (@smtbhd32-ABX).
- **Project Mission**: Enabling stable, autonomous quadruped locomotion for P2P Navigation.

---

## ❤️ Credits & Tribute

We would like to express our sincere gratitude and pay tribute to **DeepRobotics** for providing the foundational Lite3 SDK and robust hardware interfaces. Their original work is the bedrock upon which our simulation-to-reality pipeline was built. 

---
*Developed by Airbotix Technology Pvt Ltd for Lite3 Locomotion Research.*
