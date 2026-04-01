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

## 🔧 Isaac Sim TF Fix (Nav2 Integration)

### Problem: Broken TF Chain
Before the fix, the TF chain was incomplete:
```
map → odom → base_link    ← BROKEN (stopped here)
              Lite3 → TORSO → legs  ← floating, not connected
```

**Root Causes:**
1. **Wrong prim tracked**: OmniGraph used `Isaac Compute Odometry Node` pointed at `Lite3` — a **container prim** that doesn't move. Physics moves `TORSO` directly.
2. **Compute Odometry failure**: Failed due to **articulation root mismatch** — the physics root didn't match the configured prim.
3. **Result**: `odom → base_link` TF was always `(0, 0, 0)`, robot appeared frozen in RViz, Nav2 thought it never moved.

### Fix: World Pose Tracking via Track World Pose node
Switched to **directly reading the world pose of `TORSO`** using the **Track World Pose node** in Isaac Sim OmniGraph:
- The **Track World Pose node** is pointed at `TORSO` and publishes its world pose as `odom → base_link`.
- Isaac Sim's built-in joint TF handles the rest: `base_link → Lite3 → TORSO → legs`.

### Result: Complete TF Chain
```
map → odom → base_link → Lite3 → TORSO → legs
```

---

## 💾 Replication & Documentation Reference
- **Master Simulation File**: [demo3.usd](../src/isaac_bridge/isaacsim/environment/demo3.usd)
- **SDK Service Guide**: [README_lite3_sdk_service.md](README_lite3_sdk_service.md) (Original DeepRobotics tech specs).
- **ActionGraphs Gallery**: [isaac_action_graph/](isaac_action_graph/) (Screenshots of simulation wiring).
- **Frames PDF**: [frames_2026-04-01_16.14.52.pdf](frames_2026-04-01_16.14.52.pdf)
- **Nav2 Params**: [nav2_lite3_params.yaml](../nav2_lite3_params.yaml)

---

## ❤️ Credits & Tribute
We pay tribute and express our sincere gratitude to **DeepRobotics** for providing the foundational Lite3/M20 SDK and hardware interfaces. Their original work is the baseline upon which we built our simulation-to-reality pipeline. 

---
*Developed by Airbotix Technology Pvt Ltd for Lite3 Locomotion Research.*
*Sumit Bhardwaj ([@smtbhd32-ABX](https://github.com/smtbhd32-ABX))* | *Last updated: 2026-04-01*
