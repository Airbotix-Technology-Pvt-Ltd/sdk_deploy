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
- **Simulation Time Sync**: Nodes strictly listen to the `/clock` topic (`use_sim_time: True`).
- **High-Performance QoS**: Utilizes `rclcpp::SensorDataQoS()` (Best-Effort) for all Isaac-related topics.

### 3.2 Topic Mapping Specification

**Subscriber (Input to SDK):**
| Topic Source (Isaac Sim) | Message Type | drdds Destination | Purpose |
|--------------------------|--------------|-------------------|---------|
| `/joint_states` | `sensor_msgs/JointState` | `/JOINTS_DATA` | Servo positions and velocities. |
| `/imu/data` | `sensor_msgs/Imu` | `/IMU_DATA` | Orientation (RPY), Angular Velocity, and Accel. |

**Publisher (Output to Sim):**
| Topic Source (SDK) | Message Type | Isaac Sim Topic | Purpose |
|--------------------|--------------|-----------------|---------|
| `/JOINTS_CMD` | `drdds/JointsDataCmd` | `/joint_commands` | Sending Position/Velocity targets to Isaac. |

### 3.3 Control Mode: Position/Velocity
The bridge forwards the RL policy's desired joint angles and velocities directly to Isaac Sim's `Articulation Controller`. Effort calculation is handled simulator-side to leverage high-frequency solver stability.

---

## 4. Completed Project Tasks

### ✅ High-Fidelity Isaac Sim Connectivity
Implemented `state_bridge_node` and `cmd_bridge_node` with Sim-Time awareness.
- Handled IMU Orientation conversion (Quat to RPY).
- Added safety checks for Zero-Quaternions.

### ✅ SDK Safety Harmonization
Expanded joint limits in `lite3_control_parameters.cpp` to accommodate the policy range.

### ✅ Locomotion Synchronization
Aligned bridge QoS and timestamping with the simulator's native ROS 2 frequency.

---

## 5. Pending Project Tasks (Next Steps)

1. **Nav2 Configuration**: Finalizing the costmap and planner sensors in Isaac Sim.
2. **P2P Goal Interface**: Connecting Nav2's `/cmd_vel` output to the RL policy's velocity interface.
3. **Hardware Deployment**: Final testing on the physical Lite3 via the `lite3_transfer` node.

---
*DeepRobotics Lite3 - P2P Navigation Project Documentation*
*Reference: [NVIDIA Tutorial - ROS 2 RL Controller](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_rl_controller.html)*
