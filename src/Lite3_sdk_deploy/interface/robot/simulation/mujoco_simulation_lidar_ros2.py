"""
 * @file mujoco_simulation_lidar_ros2.py
 * @brief High-resolution Lidar (Hesai XT32) and RGB-D Camera Simulation
 * @author Antigravity (using mujoco-lidar library)
 * @version 2.4
 *
 * Changes from v2.3:
 *  - Added /clock publisher (rosgraph_msgs/Clock) — MuJoCo sim time broadcast
 *    every simulation step so all use_sim_time nodes sync to the simulator.
"""

import os
import sys
import argparse
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
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField, Imu
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
    def __init__(self, model_key: str = MODEL_NAME, xml_path: str = str(XML_PATH),
                 publish_odom_tf: bool = True):
        super().__init__('mujoco_lidar_simulation')

        # When running with FAST-LIO / Nav2, set publish_odom_tf=False to let
        # the SLAM stack own the odom -> base_link transform.
        self.publish_odom_tf = publish_odom_tf
        if not publish_odom_tf:
            self.get_logger().info(
                'Navigation mode: odom -> base_link TF suppressed '
                '(FAST-LIO/Nav2 owns that transform).')

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

        # Clock publisher — allows all use_sim_time nodes to follow MuJoCo time
        # Use BEST_EFFORT + depth=1 so we never queue stale clock messages
        clock_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.clock_pub = self.create_publisher(Clock, '/clock', clock_qos)

        # FAST-LIO IMU publisher — standard sensor_msgs/Imu on /imu/data
        self.imu_std_pub = self.create_publisher(Imu, '/imu/data', qos_profile)

        # Custom Lite3 publishers
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

                # Publish /clock every step — highest priority, smallest message
                self._publish_clock()

                if step % STATE_PUB_DIVIDER  == 0: self._publish_robot_state()
                if step % VISION_PUB_DIVIDER == 0: self._publish_vision_only()
                if self.viewer and self.viewer.is_running() and step % RENDER_INTERVAL == 0:
                    self.viewer.sync()

            # Use DT*0.5 timeout so spin doesn't busy-wait at 100% CPU
            rclpy.spin_once(self, timeout_sec=DT * 0.5)

    def _publish_clock(self):
        """Publish MuJoCo simulation time on /clock.

        Any ROS2 node launched with --ros-args -p use_sim_time:=true will
        use this time source instead of wall-clock. Critical for FAST-LIO
        timestamp alignment.
        """
        sec    = int(self.timestamp)
        nanosec = int((self.timestamp - sec) * 1e9)
        clock_msg = Clock()
        clock_msg.clock.sec    = sec
        clock_msg.clock.nanosec = nanosec
        self.clock_pub.publish(clock_msg)

    def _sim_time_msg(self) -> Time:
        """Return a builtin_interfaces/Time from MuJoCo sim time.

        Always use this (not get_clock().now()) for message headers so that
        the point cloud, IMU, and TF stamps are all consistent with /clock.
        """
        sec = int(self.timestamp)
        return Time(sec=sec, nanosec=int((self.timestamp - sec) * 1e9))

    def quaternion_to_euler(self, q):
        w, x, y, z = q
        roll  = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
        yaw   = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return np.array([roll, pitch, yaw], dtype=np.float32)

    def _publish_robot_state(self):
        now = self._sim_time_msg()
        # Sensor data indices (order defined in MJCF):
        # 16-18 accelerometer, 19-21 gyro
        if len(self.data.sensordata) >= 22:
            body_acc = self.data.sensordata[16:19]
            angvel_b = self.data.sensordata[19:22]
        else:
            body_acc = np.zeros(3)
            angvel_b = np.zeros(3)

        q_world  = self.data.qpos[3:7]
        rpy      = self.quaternion_to_euler(q_world)
        
        hdr   = Header(stamp=now, frame_id='imu_link')

        # --- sensor_msgs/Imu on /imu/data (consumed by FAST-LIO) ---
        imu_std = Imu()
        imu_std.header = hdr
        imu_std.angular_velocity.x    = float(angvel_b[0])
        imu_std.angular_velocity.y    = float(angvel_b[1])
        imu_std.angular_velocity.z    = float(angvel_b[2])
        imu_std.linear_acceleration.x = float(body_acc[0])
        imu_std.linear_acceleration.y = float(body_acc[1])
        imu_std.linear_acceleration.z = float(body_acc[2])
        imu_std.orientation_covariance[0] = -1.0
        self.imu_std_pub.publish(imu_std)

        # --- Custom Lite3 SDK messages (for controller compatibility) ---
        try:
            imu_msg = ImuData()
            imu_msg.header = MetaType(stamp=now, frame_id=0)
            imu_msg.data   = ImuDataValue()
            imu_msg.data.roll, imu_msg.data.pitch, imu_msg.data.yaw = map(float, rpy)
            imu_msg.data.omega_x, imu_msg.data.omega_y, imu_msg.data.omega_z = map(float, angvel_b)
            imu_msg.data.acc_x, imu_msg.data.acc_y, imu_msg.data.acc_z = map(float, body_acc)
            self.imu_pub.publish(imu_msg)

            # 12 Motored joints start at qpos index 7
            q, dq = self.data.qpos[7:19], self.data.qvel[6:18]
            joints_msg = JointsData()
            joints_msg.header = MetaType(stamp=now, frame_id=0)
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
        except (NameError, Exception):
            pass # fallback if custom messages aren't installed

        # --- High-Frequency TF Broadcasting (at 200 Hz) ---
        # Standalone mode only: publish map -> odom -> base_link chain
        # Navigation mode: FAST-LIO owns map -> base_link directly, so we skip map->odom
        if self.publish_odom_tf:
            t_map = TransformStamped()
            t_map.header.stamp = now
            t_map.header.frame_id = 'map'
            t_map.child_frame_id = 'odom'
            t_map.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t_map)


        NAME_MAP = {"vision_mount": "camera_link", "lidar_mount": "lidar_link"}

        for i in range(1, self.model.nbody):
            body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            if not body_name: continue
                
            parent_id = self.model.body_parentid[i]
            parent_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, parent_id)
            
            if body_name == "TORSO":
                if self.publish_odom_tf:
                    t_base = TransformStamped()
                    t_base.header.stamp = now
                    t_base.header.frame_id = "odom"
                    t_base.child_frame_id = "base_link"
                    t_base.transform.translation.x, t_base.transform.translation.y, t_base.transform.translation.z = self.data.xpos[i].tolist()
                    t_base.transform.rotation.w, t_base.transform.rotation.x, t_base.transform.rotation.y, t_base.transform.rotation.z = self.data.xquat[i].tolist()
                    self.tf_broadcaster.sendTransform(t_base)

                for link in ["Lite3", "TORSO"]:
                    t_link = TransformStamped()
                    t_link.header.stamp = now
                    t_link.header.frame_id = "base_link" if link == "Lite3" else "Lite3"
                    t_link.child_frame_id = link
                    t_link.transform.rotation.w = 1.0
                    self.tf_broadcaster.sendTransform(t_link)
                continue

            ros_body_name = NAME_MAP.get(body_name, body_name)
            ros_parent_name = NAME_MAP.get(parent_name, parent_name)
            # In navigation mode, root frame is map (FAST-LIO: map->base_link).
            # In standalone mode, root frame is odom (sim: map->odom->base_link).
            if parent_name == "world" or parent_name is None:
                ros_parent_name = "map" if not self.publish_odom_tf else "odom"

            # World-to-Local conversion via Rotation Matrix (xmat) for limb offsets
            pos_w = self.data.xpos[i] - self.data.xpos[parent_id]
            rot_p = self.data.xmat[parent_id].reshape(3, 3)
            pos_l = rot_p.T @ pos_w
            
            qp = self.data.xquat[parent_id]
            qc = self.data.xquat[i]
            qp_inv = np.array([qp[0], -qp[1], -qp[2], -qp[3]])
            q_rel = _quat_mul(qp_inv, qc)

            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = ros_parent_name
            t.child_frame_id = ros_body_name
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = pos_l.tolist()
            t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z = q_rel.tolist()
            self.tf_broadcaster.sendTransform(t)

        # 2. TORSO -> imu_link
        imu_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "imu_site")
        if imu_id != -1:
            torso_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "TORSO")
            t_imu = TransformStamped()
            t_imu.header.stamp = now
            t_imu.header.frame_id = 'TORSO'
            t_imu.child_frame_id = 'imu_link'
            t_imu.transform.translation.x, t_imu.transform.translation.y, t_imu.transform.translation.z = (self.data.site_xpos[imu_id] - self.data.xpos[torso_id]).tolist()
            t_imu.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t_imu)

    def _publish_vision_only(self):
        now = self._sim_time_msg()

        # 1. Lidar Scans
        self.lidar.trace_rays(self.data, self.rays_theta, self.rays_phi)
        points = self.lidar.get_hit_points()
        dist   = self.lidar.get_distances()

        header = Header(stamp=now, frame_id='lidar_link')
        valid  = (dist > 0.30) & (dist < 100.0)
        pts_v  = points[valid].astype(np.float32)
        intensity = np.clip(1.0 / np.maximum(dist[valid], 0.01), 0.0, 255.0).astype(np.float32)

        _FIELDS_XYZI = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        n = len(pts_v)
        cloud_arr = np.empty(n, dtype=np.dtype([
            ('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32)]))
        cloud_arr['x'] = pts_v[:, 0]
        cloud_arr['y'] = pts_v[:, 1]
        cloud_arr['z'] = pts_v[:, 2]
        cloud_arr['intensity'] = intensity
        pc2_msg = PointCloud2(
            header=header, height=1, width=n,
            fields=_FIELDS_XYZI, is_bigendian=False,
            point_step=16, row_step=16 * n,
            data=cloud_arr.tobytes(), is_dense=True)
        self.pc_pub.publish(pc2_msg)

        # 2. Camera Rendering
        cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, 'front_vision_camera')
        if cam_id != -1:
            self.renderer.disable_depth_rendering()
            self.renderer.update_scene(self.data, camera='front_vision_camera')
            rgb = self.renderer.render().copy()

            self.renderer.enable_depth_rendering()
            self.renderer.update_scene(self.data, camera='front_vision_camera')
            depth = self.renderer.render().copy()
            self.renderer.disable_depth_rendering()

            cam_hdr = Header(stamp=now, frame_id='camera_optical_frame')
            self.rgb_pub.publish(Image(header=cam_hdr, height=240, width=320, encoding='rgb8', step=960, data=rgb.tobytes()))
            self.depth_pub.publish(Image(header=cam_hdr, height=240, width=320, encoding='32FC1', step=1280, data=depth.tobytes()))

            info = CameraInfo(header=cam_hdr, height=240, width=320)
            f = (320 / 2) / np.tan(np.deg2rad(60 / 2))
            info.k = [f, 0.0, 160.0, 0.0, f, 120.0, 0.0, 0.0, 1.0]
            info.p = [f, 0.0, 160.0, 0.0, 0.0, f, 120.0, 0.0, 0.0, 0.0, 1.0, 0.0]
            self.info_pub.publish(info)

        # 3. Static Camera Optical Rotation
        t_o = TransformStamped()
        t_o.header.stamp = now
        t_o.header.frame_id = 'camera_link'
        t_o.child_frame_id = 'camera_optical_frame'
        t_o.transform.rotation.x, t_o.transform.rotation.y, t_o.transform.rotation.z, t_o.transform.rotation.w = -0.5, 0.5, -0.5, 0.5
        self.tf_broadcaster.sendTransform(t_o)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="MuJoCo Lidar Simulation Bridge",
        # strip ROS args so argparse doesn't choke on --ros-args etc.
        epilog="ROS2 args (--ros-args ...) are forwarded to rclpy and ignored here."
    )
    parser.add_argument(
        "--navigation",
        action="store_true",
        default=False,
        help="Enable navigation mode: suppresses odom->base_link TF so FAST-LIO/Nav2 "
             "can own that transform. Default: off (sim publishes ground-truth odom)."
    )
    # Parse only our args; leave ROS args for rclpy
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = MuJoCoLidarSimulationNode(publish_odom_tf=not args.navigation)
    try:
        node.start()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()