# Isaac Sim ROS2 Bridge for Lite3

This package bridges **Isaac Sim (ROS2)** with the **Lite3 SDK (drdds)**, enabling high-performance RL deployment and autonomous navigation.

---

## 📖 Master Documentation

Detailed project architecture, calibration logic, and replication steps are available in a single comprehensive guide:

- [**MASTER_INTEGRATION_GUIDE.md**](documentation/MASTER_INTEGRATION_GUIDE.md): **Start Here**. A direct narrative for the complete project story, hurdles, and roadmap.

---

## 🚀 Quick Start

### 1. Build the Workspace
```bash
cd ~/work/Lite3Robot/Lite3_sdk_deploy
colcon build --symlink-install --packages-select isaac_bridge lite3_sdk_deploy
. install/setup.bash
```

### 2. Launch the Bridge
Ensure Isaac Sim is running and configured to publish `/joint_states` and `/imu/data`.
```bash
ros2 launch isaac_bridge bridge.launch.py
```

### 3. Run the Robot Controller
```bash
ros2 run lite3_sdk_deploy rl_deploy
```

---
*Developed for Airbotix Technology Pvt Ltd - Lite3 RL Project.*
