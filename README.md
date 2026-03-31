# Lite3 Bridge & SDK Deployment (Airbotix Fork)

This workspace contains the core ROS 2 and C++ infrastructure for **High-Fidelity Simulation-to-Reality Deployment** of RL policies on the Lite3 quadruped platform.

---

> [!IMPORTANT]
> **Airbotix is exclusively focused on the Lite3 platform.** 
> While foundational code for other platforms (M20) is preserved as legacy reference, our research is dedicated to achieving state-of-the-art results on the Jueying Lite3.

---

**Official Airbotix Source**: [Airbotix-Technology-Pvt-Ltd/Lite3_sdk_deploy](https://github.com/Airbotix-Technology-Pvt-Ltd/Lite3_sdk_deploy)

---

### 🌐 Project Central Hubs
- [**Master Hub (Root)**](https://github.com/Airbotix-Technology-Pvt-Ltd/Lite3Robot): Mission, specialized workspaces, and organizational identity.
- [**Master Integration Guide**](../documentation/README.md): **Single Source of Truth** for technical milestones, roadmaps, and reproduction steps.
- [**Contributors Hub**](../Contributors.md): Full technical attribution for the Airbotix development team.

---

## 🛠️ Workspace Components

### **1. `isaac_bridge` (NVIDIA Isaac Sim Integration)**
The heart of our simulation infrastructure. Re-engineered as a zero-latency hub synchronized to the simulation `/clock`, ensuring stable locomotion and PD alignment between training and deployment.

### **2. `lite_transfer` (Hardware Bridging)**
The C++ communication layer between simulation-trained policies and physical hardware, using the deep-rooted M20/Lite3 UDP protocols.

### **3. `Perception Suite` (LiDAR/Depth)**
Integrated 360 Lidar and Depth camera stack for PointCloud2 streams and spatial TF frames (`odom -> base_link -> sensors`).

---

## 📖 Quick Reference
For detailed replication steps, simulation ActionGraphs, and hardware manuals, please refer to the centralized [**Technical Library**](../documentation/README.md).

---
*Airbotix Technology Pvt Ltd - Lite3 P2P Autonomous Navigation Project*
