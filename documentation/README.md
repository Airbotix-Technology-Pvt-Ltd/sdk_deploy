# Master Integration Guide - Airbotix Fork

This document provides a detailed technical narrative of the **Lite3 RL & Navigation Development (Airbotix Fork)**. It tracks our architectural progression from locomotion synchronization to high-level perception.

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
```bash
source install/setup.bash
ros2 launch launch/lite3_bringup.launch.py \
   use_sim_time:=true \
   map:=$(pwd)/map/small_area/map.yaml \
   params_file:=$(pwd)/config/nav2_lite3_params.yaml \
   use_composition:=False \
   use_respawn:=False
```

**5. Start RViz Visualization:**
```bash
source install/setup.bash
ros2 launch nav2_bringup rviz_launch.py use_sim_time:=true
```


---

## 🛠️ Core Components

| Component | Role |
|-----------|------|
| **MuJoCo** | The simulation environment (Physics, Sensors). |
| **Lite3 SDK** | The master control platform. Runs the RL policy (`.onnx` model). |
| **Nav2** | The ROS2 Navigation stack used for autonomous path planning. |


---

## 💾 Replication & Documentation Reference
- **Master Simulation File**: [mujoco_simulation_lidar_ros2.py](../src/Lite3_sdk_deploy/interface/robot/simulation/mujoco_simulation_lidar_ros2.py)
- **SDK Service Guide**: [README_lite3_sdk_service.md](README_lite3_sdk_service.md) (Original DeepRobotics tech specs).
- **Nav2 Params**: [nav2_lite3_params.yaml](../config/nav2_lite3_params.yaml)

---
## Software Setup

### Python Environment

```bash
python3 -m venv venv
source venv/bin/activate
pip3 install mujoco
```

### MuJoCo LiDAR

Reference: https://pypi.org/project/mujoco_lidar/0.2.6/

```bash
# Clone and install
git clone https://github.com/TATP-233/MuJoCo-LiDAR.git
cd MuJoCo-LiDAR

# 1. Core install (CPU backend)
pip install .

# 2. (Optional) Taichi GPU backend
pip install -e ".[taichi]"

# Verify Taichi
python -c "import taichi as ti; ti.init(ti.gpu)"
# Expected output:
# [Taichi] version 1.7.3, llvm 15.0.4, commit 5ec301be, linux, python 3.10.16
# [Taichi] Starting on arch=cuda

# 3. (Optional) JAX backend
pip install -e ".[jax]"

# Verify JAX
python -c "import jax; print(jax.default_backend())"
# Expected output: gpu
```

After setup, proceed to [Step 1](#step-1--start-mujoco-simulation) of the Quick Start.

---

## ❤️ Credits & Tribute
We pay tribute and express our sincere gratitude to **DeepRobotics** for providing the foundational Lite3/M20 SDK and hardware interfaces. Their original work is the baseline upon which we built our simulation-to-reality pipeline. 

---
*Developed by Airbotix Technology Pvt Ltd for Lite3 Locomotion Research.*
*Sumit Bhardwaj ([@smtbhd32-ABX](https://github.com/smtbhd32-ABX))* | *Last updated: 2026-04-01*


