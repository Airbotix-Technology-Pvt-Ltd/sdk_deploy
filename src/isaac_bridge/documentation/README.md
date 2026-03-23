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

## 3. The `isaac_bridge` (Sim-to-SDK Connector)

The SDK utilizes the `drdds` protocol for high-performance communication. We developed the `isaac_bridge` to translate Isaac Sim's standard ROS2 topics into these `drdds` messages.

### 3.1 Topic Mapping Specification

The bridge handles data flow in two directions:

**Subscriber (Input to SDK):**
| Topic Source (Isaac Sim) | Message Type | drdds Destination | Purpose |
|--------------------------|--------------|-------------------|---------|
| `/joint_states` | `sensor_msgs/JointState` | `/JOINTS_DATA` | Servo positions, velocities, and torques. |
| `/imu/data` | `sensor_msgs/Imu` | `/IMU_DATA` | Orientation (RPY), Angular Velocity, and Accel. |

**Publisher (Output to Sim):**
| Topic Source (SDK) | Message Type | Isaac Sim Topic | Purpose |
|--------------------|--------------|-----------------|---------|
| `/JOINTS_CMD` | `drdds/JointsDataCmd` | `/joint_commands` | Sending PD control (Kp/Kd) and torque to Sim. |

### 3.2 Calibration Logic (The Mapping)
To match the RL model's training frame with the Isaac Sim URDF, we implemented standard offsets in the bridge:
- **Knee (SDK)** = URDF Knee + 1.076 rad (Maps URDF 0.524 to RL 1.6).
- **HipY (SDK)** = URDF HipY - 0.800 rad (Maps URDF 0.0 to RL -0.8).

---

## 4. Completed Project Tasks

### ✅ Isaac Sim Connectivity (isaac_bridge)
Implemented both `state_bridge_node` and `cmd_bridge_node` for full sensor/command feedback.
- Verified 500Hz loop rate for real-time control.
- Added sensor jitter (±1e-5) to prevent SDK initialization hangs in simulation.

### ✅ SDK Safety Harmonization
Expanded joint limits in `lite3_control_parameters.cpp` to accommodate the wider range of the RL policy:
- **HipY Lower Limit**: `-4.00` rad.
- **Knee Upper Limit**: `+4.00` rad.

### ✅ Initial Locomotion Verification
Confirmed that the robot can successfully stand up and remain stable in the the simulation environment using its trained RL policy.

---

## 5. Pending Project Tasks (Next Steps)

1. **Nav2 Configuration**: Finalizing the costmap and planner sensors in Isaac Sim.
2. **P2P Goal Interface**: Connecting Nav2's `/cmd_vel` output to the RL policy's velocity interface.
3. **Hardware Deployment**: Final testing on the physical Lite3 via the `lite3_transfer` node.

---
*DeepRobotics Lite3 - P2P Navigation Project Documentation*
