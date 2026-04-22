"""
 * @file mujoco_simulation_lidar_ros2.py
 * @brief simulation in mujoco with Lidar and RGB-D Camera
 * @author Antigravity (merged from Haokai Dai)
 * @version 1.1
"""

import os
import time
from pathlib import Path
# import scipy.spatial.transform skipped to avoid numpy conflict
import numpy as np
import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from geometry_msgs.msg import TransformStamped
import tf2_ros

# Custom Lite3 messages
try:
    from drdds.msg import ImuData, JointsData, JointsDataCmd, MetaType, ImuDataValue, JointsDataValue, JointData, JointDataCmd
except ImportError:
    # Use dummy classes if not found (though they should be present in this workspace)
    class ImuData: pass
    class JointsData: pass
    class JointsDataCmd: pass

MODEL_NAME = "Lite3"
CURRENT_DIR = Path(__file__).resolve().parent

# Default to the lidar-enabled XML generated previously
# If the file is not in this directory, we assume it's in the lidar_sim folder we created
XML_PATH = CURRENT_DIR / "lite3_lidar.xml"
if not XML_PATH.exists():
    # Fallback to the lidar_sim directory
    XML_PATH = Path("/home/lite3/work/Lite3Robot/Lite3_sdk_deploy/src/lidar_sim/lite3_lidar.xml")

USE_VIEWER = True
DT = 0.001
RENDER_INTERVAL = 50
STATE_PUB_DIVIDER = 5      # 200 Hz
VISION_PUB_DIVIDER = 30    # ~33 Hz

JOINT_INIT = {
    "Lite3": np.array([0, -1.35453, 2.54948,
                     0, -1.35453, 2.54948,
                     0, -1.35453, 2.54948,
                     0, -1.35453, 2.54948,], dtype=np.float32),
}

class MuJoCoLidarSimulationNode(Node):
    def __init__(self,
                 model_key: str = MODEL_NAME,
                 xml_path: str = str(XML_PATH)):

        super().__init__('mujoco_lidar_simulation')

        # Load MJCF
        if not os.path.isfile(xml_path):
            raise FileNotFoundError(f"Cannot find MJCF: {xml_path}")

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.model.opt.timestep = DT
        self.model.opt.gravity = [0, 0, -9.81] # Force gravity
        self.data = mujoco.MjData(self.model)

        # Robot DOF
        self.actuator_ids = [a for a in range(self.model.nu)] 
        self.dof_num = 12

        # Publishers (Official State)
        self.imu_pub = self.create_publisher(ImuData, '/IMU_DATA', 200)
        self.joints_pub = self.create_publisher(JointsData, '/JOINTS_DATA', 200)

        # Publishers (Sensors)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.rgb_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        
        # TF
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscriber (Official Cmd)
        self.cmd_sub = self.create_subscription(
            JointsDataCmd,
            '/JOINTS_CMD',
            self._cmd_callback,
            10
        )

        # Renderer for Camera
        self.renderer = mujoco.Renderer(self.model, height=240, width=320)

        # Lidar detection
        self.lidar_ids = []
        i = 0
        while True:
            name = f'lidar_{i}'
            sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            if sid == -1: break
            self.lidar_ids.append(sid)
            i += 1
        self.num_beams = len(self.lidar_ids)
        
        # Diagnostics
        self.cmd_count = 0
        self.last_diag_time = time.time()

        # Initialize
        self._set_initial_pose(model_key)
        self.kp_cmd = np.zeros(self.dof_num, np.float32)
        self.kd_cmd = np.zeros(self.dof_num, np.float32)
        self.pos_cmd = np.zeros(self.dof_num, np.float32)
        self.vel_cmd = np.zeros(self.dof_num, np.float32)
        self.tau_ff = np.zeros(self.dof_num, np.float32)
        self.input_tq = np.zeros(self.dof_num, np.float32)
        self.timestamp = 0.0

        self.get_logger().info(f"Integrated Simulation Started. Lidar: {self.num_beams} beams.")

        self.viewer = None
        if USE_VIEWER:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

    def _set_initial_pose(self, key: str):
        qpos0 = self.data.qpos.copy()
        qpos0[7:7 + self.dof_num] = JOINT_INIT[key]
        qpos0[:3] = np.array([0, 0, 0.25]) # Lowered to 0.25m
        qpos0[3:7] = np.array([1, 0, 0, 0])
        self.data.qpos[:] = qpos0
        mujoco.mj_forward(self.model, self.data)

    def _cmd_callback(self, msg):
        self.cmd_count += 1
        if not hasattr(msg, 'data'): return
        if len(msg.data.joints_data) < 12: return
        for i in range(12):
            cmd = msg.data.joints_data[i]
            self.kp_cmd[i] = cmd.kp
            self.kd_cmd[i] = cmd.kd
            self.pos_cmd[i] = cmd.position
            self.vel_cmd[i] = cmd.velocity
            self.tau_ff[i] = cmd.torque

    def start(self):
        step = 0
        last_time = time.time()
        self.get_logger().info("Physics engine starting...")
        while rclpy.ok():
            current_real_time = time.time()
            if current_real_time - last_time >= DT:
                last_time = current_real_time
                step += 1
                
                # Physics
                q = self.data.qpos[7:19]
                dq = self.data.qvel[6:18]
                self.input_tq = self.kp_cmd * (self.pos_cmd - q) + self.kd_cmd * (self.vel_cmd - dq) + self.tau_ff
                self.data.ctrl[:12] = self.input_tq
                
                mujoco.mj_step(self.model, self.data)
                self.timestamp = self.data.time

                # Robot State (200 Hz)
                if step % STATE_PUB_DIVIDER == 0:
                    self._publish_robot_state()

                # Vision (33 Hz)
                if step % VISION_PUB_DIVIDER == 0:
                    self._publish_vision_and_tf()

                # Viewer (20 Hz)
                if self.viewer and self.viewer.is_running() and step % RENDER_INTERVAL == 0:
                    self.viewer.sync()

                # Diagnostics (Every 5 seconds)
                if time.time() - self.last_diag_time >= 5.0:
                    self.get_logger().info(f"[DIAGNOSTIC] Commands Received: {self.cmd_count} | Simulation Time: {self.timestamp:.2f}s")
                    self.last_diag_time = time.time()

            rclpy.spin_once(self, timeout_sec=0.0)

    def _publish_robot_state(self):
        q_world = self.data.qpos[3:7]
        rpy = self.quaternion_to_euler(q_world)
        body_acc = self.data.sensordata[16:19] if self.model.nsensor >= 19 else np.zeros(3)
        angvel_b = self.data.qvel[3:6]

        stamp = Time(sec=int(self.timestamp), nanosec=int((self.timestamp % 1) * 1e9))
        
        # IMU
        imu_msg = ImuData()
        imu_msg.header = MetaType(stamp=stamp, frame_id=0)
        imu_msg.data = ImuDataValue()
        imu_msg.data.roll, imu_msg.data.pitch, imu_msg.data.yaw = map(float, rpy)
        imu_msg.data.omega_x, imu_msg.data.omega_y, imu_msg.data.omega_z = map(float, angvel_b)
        imu_msg.data.acc_x, imu_msg.data.acc_y, imu_msg.data.acc_z = map(float, body_acc)
        self.imu_pub.publish(imu_msg)

        # Joints
        joints_msg = JointsData()
        joints_msg.header = MetaType(stamp=stamp, frame_id=0)
        joints_msg.data = JointsDataValue()
        joints_msg.data.joints_data = [JointData() for _ in range(16)]
        q = self.data.qpos[7:19]
        dq = self.data.qvel[6:18]
        for i in range(12):
            j = joints_msg.data.joints_data[i]
            j.status_word = 1
            j.position, j.velocity, j.torque = float(q[i]), float(dq[i]), float(self.input_tq[i])
        for i in range(12, 16): joints_msg.data.joints_data[i].status_word = 1
        self.joints_pub.publish(joints_msg)

    def _publish_vision_and_tf(self):
        now = self.get_clock().now().to_msg()
        # Lidar
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = now, 'lidar_link'
        scan.angle_min, scan.angle_max = 0.0, 2 * np.pi
        scan.angle_increment = (2 * np.pi) / self.num_beams
        scan.range_min, scan.range_max = 0.25, 10.0
        scan.ranges = [float(self.data.sensordata[sid]) if 0 <= self.data.sensordata[sid] < 10.0 else float('inf') for sid in self.lidar_ids]
        self.scan_pub.publish(scan)

        # RGB-D
        self.renderer.update_scene(self.data, camera='front_vision_camera')
        self.renderer.disable_depth_rendering()
        rgb = np.flipud(self.renderer.render())
        self.renderer.enable_depth_rendering()
        depth = np.flipud(self.renderer.render())
        
        # RGB msg
        rgb_msg = Image(header=Header(stamp=now, frame_id='camera_optical_frame'), 
                        height=240, width=320, encoding='rgb8', step=960, data=rgb.tobytes())
        self.rgb_pub.publish(rgb_msg)
        
        # Depth msg
        depth_msg = Image(header=Header(stamp=now, frame_id='camera_optical_frame'),
                          height=240, width=320, encoding='32FC1', step=1280, data=depth.tobytes())
        self.depth_pub.publish(depth_msg)

        # Camera Info
        info = CameraInfo(header=Header(stamp=now, frame_id='camera_optical_frame'))
        info.header.frame_id = 'camera_optical_frame'
        info.height, info.width = 240, 320
        f = (320 / 2) / np.tan(np.deg2rad(60 / 2))
        info.k = [f, 0.0, 160.0, 0.0, f, 120.0, 0.0, 0.0, 1.0]
        info.p = [f, 0.0, 160.0, 0.0, 0.0, f, 120.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)

        # TF
        q_pos, q_quat = self.data.qpos[:3], self.data.qpos[3:7]
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = now, 'odom', 'base_link'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = q_pos
        t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z = q_quat
        self.tf_broadcaster.sendTransform(t)
        
        # Static frames
        tl = TransformStamped()
        tl.header.stamp, tl.header.frame_id, tl.child_frame_id = now, 'base_link', 'lidar_link'
        tl.transform.translation.x, tl.transform.translation.z, tl.transform.rotation.w = 0.15, 0.07, 1.0
        self.tf_broadcaster.sendTransform(tl)
        
        tc = TransformStamped()
        tc.header.stamp, tc.header.frame_id, tc.child_frame_id = now, 'base_link', 'camera_link'
        tc.transform.translation.x, tc.transform.translation.z, tc.transform.rotation.w = 0.25, 0.05, 1.0
        self.tf_broadcaster.sendTransform(tc)
        
        to = TransformStamped()
        to.header.stamp, to.header.frame_id, to.child_frame_id = now, 'camera_link', 'camera_optical_frame'
        to.transform.rotation.x, to.transform.rotation.y, to.transform.rotation.z, to.transform.rotation.w = -0.5, 0.5, -0.5, 0.5
        self.tf_broadcaster.sendTransform(to)

    def quaternion_to_euler(self, q):
        w, x, y, z = q
        roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
        yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return np.array([roll, pitch, yaw], dtype=np.float32)

if __name__ == "__main__":
    rclpy.init()
    node = MuJoCoLidarSimulationNode()
    try:
        node.start()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
