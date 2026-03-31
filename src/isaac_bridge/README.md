# Isaac Sim ROS2 Bridge for Lite3

This package bridges **Isaac Sim (ROS2)** with the **Lite3 SDK (drdds)**, enabling high-performance RL deployment and autonomous navigation.

---

## 📖 Master Documentation

Detailed project architecture, calibration logic, and replication steps are available in a single comprehensive guide:

- [**documentation/README.md**](documentation/README.md): **Start Here**. A direct narrative for the complete project story, milestones, and replication source.

---

## 🛠️ Step-by-Step Project Milestones

### **Step 1: Locomotion Synchronization**
Establishing high-fidelity C++ bridging, `/clock`-synced control, and `Best-Effort` QoS.

### **Step 2: Simulation Logic Setup (RL Controller Sync)**
Configured simulation environment and bridge logic following the [NVIDIA Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_rl_controller.html).

### **Step 3: Perception Integration (LiDAR & Depth)**
Integrated LiDAR and Depth sensors for `PointCloud2` and `Depth Image` output within the simulation.

### **Step 4: Spatial Frame Integration (TF Tree)**
Established the full `odom -> base_link -> sensors` transform chain for Nav2 compliance.
- **Frames PDF**: [frames_2026-03-31_11.31.23.pdf](documentation/frames_2026-03-31_11.32.31.pdf)
- **Tutorial Reference**: [NVIDIA ROS 2 TF Integration](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_tf.html)

---

## 🚀 Quick Replication & Deployment

To reproduce the latest architecture:
1. **Source Simulation**: Load [**`demo3.usd`**](../../isaacsim/environment/demo3.usd).
2. **Review Graphs**: Verify simulation wiring using the [ActionGraphs Gallery](documentation/isaac_action_graphs/).
3. **Launch Bridge**:
   ```bash
   ros2 launch isaac_bridge bridge.launch.py
   ```
4. **Deploy RL**:
   ```bash
   ros2 run lite3_sdk_deploy rl_deploy
   ```

---
*Developed for Airbotix Technology Pvt Ltd - Lite3 RL Project.*
