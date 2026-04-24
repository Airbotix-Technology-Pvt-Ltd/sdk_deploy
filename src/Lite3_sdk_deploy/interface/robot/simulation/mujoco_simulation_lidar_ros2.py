"""
 * @file mujoco_simulation_lidar_ros2.py
 * @brief High-resolution Lidar (Hesai XT32) and RGB-D Camera Simulation
 * @author Antigravity (using mujoco-lidar library)
 * @version 2.3
 *
 * Changes from v2.2:
 *  - Fixed backend-specific args (bodyexclude valid for all backends per lidar_wrapper.py)
 *  - Fixed scan pattern: theta=azimuth, phi=elevation (matches scan_gen convention)
 *  - Fixed busy-wait loop: spin_once uses DT*0.5 timeout instead of 0.0
 *  - Fixed exit(1) in __init__ replaced with RuntimeError
 *  - Fixed sensordata bounds check: len(sensordata) not nsensor
 *  - Fixed depth render: update_scene called before each render pass
 *  - Fixed viewer: is_running() checked before sync()
 *  - Fixed main: node stored, clean shutdown with try/finally
 *  - Added GPU (Taichi) backend with RTX 4060 optimized args
 *  - Backend priority: taichi(GPU) -> jax -> cpu
"""

import os
import time
from pathlib import Path
import numpy as np
import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import TransformStamped
import tf2_ros

def _quat_mul(q1, q2):
    """Hamilton product of two [w,x,y,z] quaternions."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])

# GPU/CPU-based Lidar Library
try:
    from mujoco_lidar import MjLidarWrapper
except ImportError:
    print("Error: mujoco-lidar library not found. Run: pip install mujoco-lidar")
    exit(1)

# Custom Lite3 messages
try:
    from drdds.msg import ImuData, JointsData, JointsDataCmd, MetaType, ImuDataValue, JointsDataValue, JointData, JointDataCmd
except ImportError:
    class MetaType:
        def __init__(self, **kwargs): pass
    class ImuData:
        def __init__(self, **kwargs): pass
    class JointsData:
        def __init__(self, **kwargs): pass
    class JointsDataCmd:
        def __init__(self, **kwargs): pass
    class ImuDataValue: pass
    class JointsDataValue: pass
    class JointData: pass
    class JointDataCmd: pass

MODEL_NAME = "Lite3"
CURRENT_DIR = Path(__file__).resolve().parent

XML_PATH = CURRENT_DIR / ".." / ".." / ".." / "Lite3_description" / "lite3_mjcf" / "mjcf" / "Lite3_stair_lidar.xml"
XML_PATH = str(XML_PATH.resolve())

USE_VIEWER = True
DT = 0.001
RENDER_INTERVAL = 50
STATE_PUB_DIVIDER = 5      # 200 Hz
VISION_PUB_DIVIDER = 30    # ~33 Hz

JOINT_INIT = {
    "Lite3": np.array([0, -1.35453, 2.54948,
                       0, -1.35453, 2.54948,
                       0, -1.35453, 2.54948,
                       0, -1.35453, 2.54948], dtype=np.float32),
}

# ---------------------------------------------------------------------------
# Backend-specific args for MjLidarWrapper
# bodyexclude is supported by all backends (cpu, taichi, jax) per lidar_wrapper.py
# RTX 4060 has 8GB VRAM — 4GB reserved for Taichi is safe alongside Isaac/ROS
# ---------------------------------------------------------------------------
def _make_backend_args(body_id: int) -> dict:
    return {
        "taichi": {
            'bodyexclude': body_id,
            'max_candidates': 64,           # good balance for warehouse mesh scenes
            'ti_init_args': {
                'device_memory_GB': 4.0,    # RTX 4060 8GB — leave 4GB for system/ROS
            }
        },
        "jax": {
            'bodyexclude': body_id,
        },
        "cpu": {
            'bodyexclude': body_id,
            'geomgroup': None,              # detect all geometry groups
        },
    }


class MuJoCoLidarSimulationNode(Node):
    def __init__(self, model_key: str = MODEL_NAME, xml_path: str = str(XML_PATH)):
        super().__init__('mujoco_lidar_simulation')

        if not os.path.isfile(xml_path):
            raise FileNotFoundError(f"Cannot find MJCF: {xml_path}")

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.model.opt.timestep = DT
        self.data = mujoco.MjData(self.model)
        self.dof_num = 12

        self._set_initial_pose(model_key)

        # Cache
        self.kp_cmd  = np.zeros((self.dof_num, 1), np.float32)
        self.kd_cmd  = np.zeros_like(self.kp_cmd)
        self.pos_cmd = np.zeros_like(self.kp_cmd)
        self.vel_cmd = np.zeros_like(self.kp_cmd)
        self.tau_ff  = np.zeros_like(self.kp_cmd)
        self.input_tq = np.zeros_like(self.kp_cmd)
        self.timestamp = 0.0

        # High-Speed QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Standard Publishers
        self.imu_pub    = self.create_publisher(ImuData,    '/IMU_DATA',    qos_profile)
        self.joints_pub = self.create_publisher(JointsData, '/JOINTS_DATA', qos_profile)

        # Lidar/Camera Publishers
        self.pc_pub    = self.create_publisher(PointCloud2, 'lidar_points',               10)
        self.rgb_pub   = self.create_publisher(Image,       'camera/image_raw',           10)
        self.depth_pub = self.create_publisher(Image,       'camera/depth/image_raw',     10)
        self.info_pub  = self.create_publisher(CameraInfo,  'camera/camera_info',         10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.cmd_sub = self.create_subscription(
            JointsDataCmd, '/JOINTS_CMD', self._cmd_callback, qos_profile)

        self.renderer = mujoco.Renderer(self.model, height=240, width=320)

        # -----------------------------------------------------------------------
        # Initialize Lidar — priority: taichi (RTX 4060 GPU) -> jax -> cpu
        # All backends support bodyexclude per mujoco_lidar 0.2.6 lidar_wrapper.py
        # -----------------------------------------------------------------------
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "TORSO")
        backend_args = _make_backend_args(body_id)

        self.lidar = None
        self.lidar_backend = None
        for backend in ["taichi", "jax", "cpu"]:
            try:
                self.get_logger().info(
                    f"Attempting to initialize Lidar with {backend} backend...")
                self.lidar = MjLidarWrapper(
                    self.model,
                    site_name="lidar_site",
                    backend=backend,
                    cutoff_dist=400.0,
                    args=backend_args[backend]
                )
                self.lidar_backend = backend
                self.get_logger().info(
                    f"Successfully initialized Lidar with {backend} backend.")
                break
            except Exception as e:
                self.get_logger().warn(
                    f"Failed to initialize {backend} backend: {e}")

        if self.lidar is None:
            raise RuntimeError(
                "Could not initialize any Lidar backend! "
                "Install from source: git clone https://github.com/TATP-233/MuJoCo-LiDAR.git && "
                "cd MuJoCo-LiDAR && python3 -m pip install -e '.[taichi]'")

        # -----------------------------------------------------------------------
        # Hesai XT32 scan pattern
        # mujoco_lidar convention (from scan_gen.py):
        #   rays_theta = azimuth  (horizontal, 0..2π)
        #   rays_phi   = elevation (vertical,  -16°..+15° for XT32)
        # -----------------------------------------------------------------------
        azim = np.deg2rad(np.linspace(0, 360, 2000, endpoint=False))   # 1024 azimuth steps
        elev = np.deg2rad(np.linspace(-16, 15, 32))                     # 32 elevation channels
        aa, ee = np.meshgrid(azim, elev)   # aa=azimuth, ee=elevation
        self.rays_theta = aa.flatten()     # theta = azimuth  ✓
        self.rays_phi   = ee.flatten()     # phi   = elevation ✓
        # Total rays: 32 × 1024 = 32768 per scan

        self.viewer = None
        if USE_VIEWER:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

    def _set_initial_pose(self, key: str):
        qpos0 = self.data.qpos.copy()
        qpos0[7:7 + self.dof_num] = JOINT_INIT[key]
        qpos0[:3] = np.array([0, 0, 0.4])
        qpos0[3:7] = np.array([1, 0, 0, 0])
        self.data.qpos[:] = qpos0
        mujoco.mj_forward(self.model, self.data)

    def _cmd_callback(self, msg: 'JointsDataCmd'):
        if not hasattr(msg, 'data') or len(msg.data.joints_data) not in (12, 16):
            return
        for i in range(self.dof_num):
            joint_cmd = msg.data.joints_data[i]
            self.kp_cmd[i]  = joint_cmd.kp
            self.kd_cmd[i]  = joint_cmd.kd
            self.pos_cmd[i] = joint_cmd.position
            self.vel_cmd[i] = joint_cmd.velocity
            self.tau_ff[i]  = joint_cmd.torque

    def start(self):
        step = 0
        last_time = time.time()
        self.get_logger().info(
            f"Simulation loop started. Lidar backend: {self.lidar_backend}, "
            f"rays/scan: {len(self.rays_theta)}")

        while rclpy.ok():
            now = time.time()
            if now - last_time >= DT:
                last_time = now
                step += 1

                q  = self.data.qpos[7:7 + self.dof_num].reshape(-1, 1)
                dq = self.data.qvel[6:6 + self.dof_num].reshape(-1, 1)
                self.input_tq = (
                    self.kp_cmd * (self.pos_cmd - q) +
                    self.kd_cmd * (self.vel_cmd - dq) +
                    self.tau_ff
                )
                self.data.ctrl[:] = self.input_tq.flatten()

                mujoco.mj_step(self.model, self.data)
                self.timestamp = step * DT

                if step % STATE_PUB_DIVIDER  == 0: self._publish_robot_state()
                if step % VISION_PUB_DIVIDER == 0: self._publish_vision_and_tf()
                if self.viewer and self.viewer.is_running() and step % RENDER_INTERVAL == 0:
                    self.viewer.sync()

            # Use DT*0.5 timeout so spin doesn't busy-wait at 100% CPU
            rclpy.spin_once(self, timeout_sec=DT * 0.5)

    def quaternion_to_euler(self, q):
        w, x, y, z = q
        roll  = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
        yaw   = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return np.array([roll, pitch, yaw], dtype=np.float32)

    def _publish_robot_state(self):
        q_world  = self.data.qpos[3:7]
        rpy      = self.quaternion_to_euler(q_world)
        # Fix: check sensordata length, not nsensor count
        body_acc = (self.data.sensordata[16:19]
                    if len(self.data.sensordata) >= 19
                    else np.zeros(3))
        angvel_b = self.data.qvel[3:6]
        stamp    = Time(sec=int(self.timestamp),
                        nanosec=int((self.timestamp % 1) * 1e9))

        try:
            imu_msg = ImuData()
            imu_msg.header = MetaType(stamp=stamp, frame_id=0)
            imu_msg.data   = ImuDataValue()
            imu_msg.data.roll,  imu_msg.data.pitch, imu_msg.data.yaw   = map(float, rpy)
            imu_msg.data.omega_x, imu_msg.data.omega_y, imu_msg.data.omega_z = map(float, angvel_b)
            imu_msg.data.acc_x,   imu_msg.data.acc_y,   imu_msg.data.acc_z   = map(float, body_acc)
            self.imu_pub.publish(imu_msg)

            q, dq = self.data.qpos[7:19], self.data.qvel[6:18]
            joints_msg = JointsData()
            joints_msg.header = MetaType(stamp=stamp, frame_id=0)
            joints_msg.data   = JointsDataValue()
            joints_msg.data.joints_data = [JointData() for _ in range(16)]
            for i in range(12):
                j = joints_msg.data.joints_data[i]
                j.status_word = 1
                j.position = float(q[i])
                j.velocity = float(dq[i])
                j.torque   = float(self.input_tq[i])
            for i in range(12, 16):
                joints_msg.data.joints_data[i].status_word = 1
            self.joints_pub.publish(joints_msg)
        except Exception:
            pass  # graceful fallback when using mocked message classes

    def _publish_vision_and_tf(self):
        now = self.get_clock().now().to_msg()

        # -------------------------------------------------------------------
        # Lidar — trace rays and publish PointCloud2
        # MjLidarWrapper.trace_rays handles pose update internally via site_name
        # -------------------------------------------------------------------
        self.lidar.trace_rays(self.data, self.rays_theta, self.rays_phi)
        points = self.lidar.get_hit_points()   # (N, 3) in lidar local frame
        dist   = self.lidar.get_distances()    # (N,)

        header   = Header(stamp=now, frame_id='lidar_link')
        valid    = (dist > 0.60) & (dist < 400.9)
        pc2_msg  = point_cloud2.create_cloud_xyz32(header, points[valid])
        self.pc_pub.publish(pc2_msg)

        # -------------------------------------------------------------------
        # Camera — RGB + Depth
        # update_scene must be called before EACH render pass
        # -------------------------------------------------------------------
        cam_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, 'front_vision_camera')
        if cam_id != -1:
            # RGB pass
            self.renderer.disable_depth_rendering()
            self.renderer.update_scene(self.data, camera='front_vision_camera')
            rgb = np.flipud(self.renderer.render())

            # Depth pass — update_scene again so depth is from current state
            self.renderer.enable_depth_rendering()
            self.renderer.update_scene(self.data, camera='front_vision_camera')
            depth = np.flipud(self.renderer.render())
            self.renderer.disable_depth_rendering()  # reset for next RGB pass

            rgb_msg = Image(
                header=Header(stamp=now, frame_id='camera_optical_frame'),
                height=240, width=320, encoding='rgb8',
                step=960, data=rgb.tobytes())
            self.rgb_pub.publish(rgb_msg)

            depth_msg = Image(
                header=Header(stamp=now, frame_id='camera_optical_frame'),
                height=240, width=320, encoding='32FC1',
                step=1280, data=depth.tobytes())
            self.depth_pub.publish(depth_msg)

            info = CameraInfo(header=Header(stamp=now, frame_id='camera_optical_frame'))
            info.height, info.width = 240, 320
            f = (320 / 2) / np.tan(np.deg2rad(60 / 2))
            info.k = [f, 0.0, 160.0, 0.0, f, 120.0, 0.0, 0.0, 1.0]
            info.p = [f, 0.0, 160.0, 0.0, 0.0, f, 120.0, 0.0, 0.0, 0.0, 1.0, 0.0]
            self.info_pub.publish(info)

        # -------------------------------------------------------------------
        # TF transforms — broadcast EVERY body from MuJoCo with name mapping
        # -------------------------------------------------------------------
        NAME_MAP = {
            "TORSO": "base_link",
            "vision_mount": "camera_link",
            "lidar_mount": "lidar_link",
        }

        for i in range(1, self.model.nbody):  # skip 0 = world body
            body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            if not body_name:
                continue
                
            parent_id = self.model.body_parentid[i]
            parent_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, parent_id)
            
            # Map names
            ros_body_name = NAME_MAP.get(body_name, body_name)
            ros_parent_name = NAME_MAP.get(parent_name, parent_name)
            
            if parent_name == "world" or parent_name is None:
                ros_parent_name = "odom"

            # MuJoCo gives world-frame pos/quat (xpos/xquat)
            pos = self.data.xpos[i] - self.data.xpos[parent_id]
            
            qp = self.data.xquat[parent_id]  # [w,x,y,z]
            qc = self.data.xquat[i]          # [w,x,y,z]
            qp_inv = np.array([qp[0], -qp[1], -qp[2], -qp[3]])
            q_local = _quat_mul(qp_inv, qc)

            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = ros_parent_name
            t.child_frame_id = ros_body_name
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = pos.tolist()
            t.transform.rotation.w = float(q_local[0])
            t.transform.rotation.x = float(q_local[1])
            t.transform.rotation.y = float(q_local[2])
            t.transform.rotation.z = float(q_local[3])
            self.tf_broadcaster.sendTransform(t)

        # imu_link — publish from imu_site (relative to base_link/TORSO)
        imu_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "imu_site")
        if imu_id != -1:
            t_imu = TransformStamped()
            t_imu.header.stamp = now
            t_imu.header.frame_id = 'base_link'
            t_imu.child_frame_id = 'imu_link'
            # site pos/quat are usually in body frame if not otherwise specified, 
            # but xpos/xquat are world frame.
            site_xpos = self.data.site_xpos[imu_id]
            site_xmat = self.data.site_xmat[imu_id].reshape(3, 3)
            # Simplest for Lite3: IMU site is at (0,0,0) of TORSO
            # We'll compute it from xpos/xquat of site vs TORSO
            torso_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "TORSO")
            p_pos = self.data.xpos[torso_id]
            p_quat = self.data.xquat[torso_id]
            
            rel_pos = site_xpos - p_pos
            # For simplicity if orientation is same as TORSO:
            t_imu.transform.translation.x, t_imu.transform.translation.y, t_imu.transform.translation.z = rel_pos.tolist()
            t_imu.transform.rotation.w = 1.0 # Standard orientation for Lite3 IMU site
            self.tf_broadcaster.sendTransform(t_imu)

        # Static rotation for Camera Optical Frame (relative to camera_link)
        t_o = TransformStamped()
        t_o.header.stamp = now
        t_o.header.frame_id = 'camera_link'
        t_o.child_frame_id = 'camera_optical_frame'
        t_o.transform.rotation.x = -0.5
        t_o.transform.rotation.y = 0.5
        t_o.transform.rotation.z = -0.5
        t_o.transform.rotation.w = 0.5
        self.tf_broadcaster.sendTransform(t_o)


if __name__ == "__main__":
    rclpy.init()
    node = MuJoCoLidarSimulationNode()
    try:
        node.start()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()