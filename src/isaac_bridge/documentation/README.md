# Project Integration Guide: Point-to-Point (P2P) Autonomous Navigation

## 1. Project Objectives
The primary goal is the deployment of autonomous **Point-to-Point (P2P) Navigation** on the DeepRobotics Lite3 quadruped. 
1. **Simulation (First)**: Achieving stable P2P navigation in NVIDIA Isaac Sim.
2. **Real Hardware (Second)**: Transferring the exact same control stack to the physical robot.

To achieve this, the system requires a trained **RL Policy Runner** (contained within the Lite3 SDK) to handle locomotion, while **Nav2** handles high-level path planning.

---

## 2. Core Components

| Component | Role |
|-----------|------|
| **Isaac Sim** | The simulation environment (Worlds, Physics, Sensors). |
| **Lite3 SDK** | The master control platform. Runs the RL policy (`.onnx` model). |
| **isaac_bridge** | The bi-directional connector between Isaac Sim and the SDK. |
| **Nav2** | The ROS2 Navigation stack used for autonomous path planning. |

---

## 3. The `isaac_bridge` (High-Fidelity Sim-to-SDK Connector)

The bridge handles data flow in two directions with specialized logic to match the RL policy requirements.

### 3.1 Architecture: Zero-Latency & Sim-Time Synchronized
Following the [NVIDIA Isaac Sim ROS 2 RL Controller Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_rl_controller.html), the bridge is configured as a **Pure Pos/Vel Passthrough**:
- **Event-Driven Forwarding**: Sensor data is forwarded to the SDK immediately upon receipt from Isaac Sim.
- **Simulation Time Sync**: Nodes strictly listen to the `/clock` topic (`use_sim_time: True`), ensuring all messages are timestamped consistently with the physics engine.
- **High-Performance QoS**: Utilizes `rclcpp::SensorDataQoS()` (Best-Effort) for all Isaac-related topics.

---

## 4. Completed Project Milestone Steps

### **Step 1: High-Fidelity Isaac Sim Connectivity (Locomotion Sync)**
Implemented `state_bridge_node` and `cmd_bridge_node` with Sim-Time awareness.
- Handled IMU Orientation conversion (Quat to RPY).
- Added safety checks for Zero-Quaternions to prevent RL policy crashes.

### **Step 2: Simulation Logic Setup (RL Controller Sync)**
Configured simulation environment and bridge logic following the [NVIDIA Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_rl_controller.html).
- Established the **On-Demand** simulation pipeline for synchronization.

### **Step 3: Multi-Modal Perception Integration (LiDAR & Depth)**
Integrated both LiDAR and Depth Camera sensors following the [NVIDIA Tutorial](https://www.youtube.com/watch?v=mMaWWAIDXH8) and [Depth Camera Tutorial](https://www.youtube.com/watch?v=yuV8AYAeW_c).
- Confirmed `sensor_msgs/PointCloud2` and `sensor_msgs/Image` (Depth) output.
- Verified stable visualization of perception data in `rviz2`.

### **Step 4: Spatial Frame Integration (TF Tree)**
Established the full `odom -> base_link -> sensors` transform chain following the [NVIDIA Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_tf.html).
- Verified full tree representation in `rviz2`.
- Confirmed correct alignment of sensor frames relative to the robot's base link.

---

## 5. Reproduction & Demos

To reproduce the work above, use the latest simulation assets (internal to package):
- **Master Simulation File**: [isaacsim/environment/demo3.usd](../isaacsim/environment/demo3.usd)
- **ActionGraphs Gallery**: [isaac_action_graphs/](isaac_action_graphs/) (Screenshots of all internal simulation wiring).

---

## 6. Pending Project Milestone Steps

1. **Nav2 Configuration**: Finalizing the costmap using combined LiDAR and Depth data.
2. **P2P Goal Interface**: Connecting Nav2's `/cmd_vel` output to the RL policy's velocity interface.
3. **Hardware Deployment**: Final testing on the physical Lite3 via the `lite3_transfer` node.

---
*DeepRobotics Lite3 - P2P Navigation Project Documentation*
*References:*
- [NVIDIA Tutorial - ROS 2 RL Controller](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_rl_controller.html)
- [NVIDIA Tutorial - LiDAR Integration](https://www.youtube.com/watch?v=mMaWWAIDXH8)
- [NVIDIA Tutorial - Depth Camera](https://www.youtube.com/watch?v=yuV8AYAeW_c)
- [NVIDIA Tutorial - TF Integration](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_tf.html)
