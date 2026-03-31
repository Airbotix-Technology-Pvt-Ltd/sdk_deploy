# Master Integration Guide - Airbotix Fork

This document provides a detailed technical narrative of the **Lite3 RL & Navigation Development (Airbotix Fork)**. It tracks our architectural progression from locomotion synchronization to high-level perception.

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

## 🛠️ Core Components

| Component | Role |
|-----------|------|
| **Isaac Sim** | The simulation environment (Worlds, Physics, Sensors). |
| **Lite3 SDK** | The master control platform. Runs the RL policy (`.onnx` model). |
| **isaac_bridge** | The bi-directional connector between Isaac Sim and the SDK. |
| **Nav2** | The ROS2 Navigation stack used for autonomous path planning. |

---

## 📖 The `isaac_bridge` (High-Fidelity Connector)

### Architecture: Zero-Latency & Sim-Time Synchronized
Following the [NVIDIA Isaac Sim ROS 2 RL Controller Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_rl_controller.html), the bridge is configured as a **Pure Pos/Vel Passthrough**:
- **Event-Driven Forwarding**: Sensor data is forwarded to the SDK immediately upon receipt from Isaac Sim.
- **Simulation Time Sync**: Nodes strictly listen to the `/clock` topic (`use_sim_time: True`), ensuring all messages are timestamped consistently with the physics engine.

### Topic Mapping Specification
| Topic Source (Isaac Sim) | Message Type | drdds Destination | Purpose |
|--------------------------|--------------|-------------------|---------|
| `/joint_states` | `sensor_msgs/JointState` | `/JOINTS_DATA` | Servo positions and velocities. |
| `/imu/data` | `/imu/data` | `/IMU_DATA` | Orientation (RPY), Angular Velocity, and Accel (Gravity=9.8). |
| `/JOINTS_CMD` | `drdds/JointsDataCmd` | `/joint_commands` | Sending Position/Velocity targets to Isaac. |

---

## 📈 Active & Pending Milestone Targets

### **Ongoing (Active Research): Intelligent Navigation**
- [ ] **Fast-LIO SLAM**: Integrating high-performance 3D LiDAR Odometry and Mapping.
- [ ] **Nav2 Path Planning**: Configuring the costmap and global planner using the SLAM map.
- [ ] **P2P Goal Interface**: Connecting Nav2 goals to the RL policy's velocity interface.

### **Pending: Real-World Deployment**
- [ ] **Hardware Transfer**: Validating the architecture on the physical Lite3 hardware.

---

## 💾 Replication & Documentation Reference
- **Master Simulation File**: [demo3.usd](../src/isaac_bridge/isaacsim/environment/demo3.usd)
- **SDK Service Guide**: [README_lite3_sdk_service.md](README_lite3_sdk_service.md) (Original DeepRobotics tech specs).
- **ActionGraphs Gallery**: [isaac_action_graph/](isaac_action_graph/) (Screenshots of simulation wiring).
- **Frames PDF**: [frames_2026-03-31_11.32.31.pdf](frames_2026-03-31_11.32.31.pdf)

---

## ❤️ Credits & Tribute
We pay tribute and express our sincere gratitude to **DeepRobotics** for providing the foundational Lite3/M20 SDK and hardware interfaces. Their original work is the baseline upon which we built our simulation-to-reality pipeline. 

---
*Developed by Airbotix Technology Pvt Ltd for Lite3 Locomotion Research.*
*Sumit Bhardwaj ([@smtbhd32-ABX](https://github.com/smtbhd32-ABX))*
