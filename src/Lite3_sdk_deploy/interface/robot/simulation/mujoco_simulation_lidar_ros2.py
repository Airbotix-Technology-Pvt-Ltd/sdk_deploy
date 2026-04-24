"""
 * @file mujoco_simulation_lidar_ros2.py
 * @brief simulation in mujoco with Lidar and RGB-D Camera (Recreated from standard baseline)
 * @author Antigravity (merged from Haokai Dai)
 * @version 1.3
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
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros

# Custom Lite3 messages
from drdds.msg import ImuData, JointsData, JointsDataCmd, MetaType, ImuDataValue, JointsDataValue, JointData, JointDataCmd

MODEL_NAME = "Lite3"
CURRENT_DIR = Path(__file__).resolve().parent

# Define the XML path relative to the Python file (Matching standard bridge)
XML_PATH = CURRENT_DIR / ".." / ".." / ".." / "Lite3_description" / "lite3_mjcf" / "mjcf" / "Lite3_stair.xml"
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
                     0, -1.35453, 2.54948,], dtype=np.float32),
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
        self.kp_cmd = np.zeros((self.dof_num, 1), np.float32)
        self.kd_cmd = np.zeros_like(self.kp_cmd)
        self.pos_cmd = np.zeros_like(self.kp_cmd)
        self.vel_cmd = np.zeros_like(self.kp_cmd)
        self.tau_ff = np.zeros_like(self.kp_cmd)
        self.input_tq = np.zeros_like(self.kp_cmd)
        self.timestamp = 0.0

        # High-Speed Best Effort QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Standard Publishers
        self.imu_pub = self.create_publisher(ImuData, '/IMU_DATA', qos_profile)
        self.joints_pub = self.create_publisher(JointsData, '/JOINTS_DATA', qos_profile)

        # Lidar/Camera Publishers
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.rgb_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.cmd_sub = self.create_subscription(JointsDataCmd, '/JOINTS_CMD', self._cmd_callback, qos_profile)

        self.renderer = mujoco.Renderer(self.model, height=240, width=320)
        self.lidar_ids = []
        i = 0
        while True:
            name = f'lidar_{i}'
            sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            if sid == -1: break
            self.lidar_ids.append(sid)
            i += 1
        self.num_beams = len(self.lidar_ids)

        self.viewer = None
        if USE_VIEWER:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            # Enable Sensor visualization (Shows Lidar rays)
            self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = True

    def _set_initial_pose(self, key: str):
        qpos0 = self.data.qpos.copy()
        qpos0[7:7 + self.dof_num] = JOINT_INIT[key]
        qpos0[:3] = np.array([0, 0, 0.2])
        qpos0[3:7] = np.array([1, 0, 0, 0])
        self.data.qpos[:] = qpos0
        mujoco.mj_forward(self.model, self.data)

    def _cmd_callback(self, msg: JointsDataCmd):
        if len(msg.data.joints_data) not in (12, 16): return
        for i in range(self.dof_num):
            joint_cmd = msg.data.joints_data[i]
            self.kp_cmd[i] = joint_cmd.kp
            self.kd_cmd[i] = joint_cmd.kd
            self.pos_cmd[i] = joint_cmd.position
            self.vel_cmd[i] = joint_cmd.velocity
            self.tau_ff[i] = joint_cmd.torque

    def start(self):
        step = 0
        last_time = time.time()
        self.get_logger().info("Lidar Simulation Baseline Started.")
        while rclpy.ok():
            if time.time() - last_time >= DT:
                last_time = time.time()
                step += 1
                
                # Control Law
                q = self.data.qpos[7:7 + self.dof_num].reshape(-1, 1)
                dq = self.data.qvel[6:6 + self.dof_num].reshape(-1, 1)
                self.input_tq = (self.kp_cmd * (self.pos_cmd - q) + self.kd_cmd * (self.vel_cmd - dq) + self.tau_ff)
                self.data.ctrl[:] = self.input_tq.flatten()

                mujoco.mj_step(self.model, self.data)
                self.timestamp = step * DT

                # State publishing (200 Hz)
                if step % STATE_PUB_DIVIDER == 0:
                    self._publish_robot_state()

                # Vision publishing (33 Hz)
                if step % VISION_PUB_DIVIDER == 0:
                    self._publish_vision_and_tf()

                # Viewer (20 Hz)
                if self.viewer and step % RENDER_INTERVAL == 0:
                    self.viewer.sync()

            rclpy.spin_once(self, timeout_sec=0.0)

    def quaternion_to_euler(self, q):
        w, x, y, z = q
        roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return np.array([roll, pitch, yaw], dtype=np.float32)

    def _publish_robot_state(self):
        q_world = self.data.qpos[3:7]
        rpy = self.quaternion_to_euler(q_world)
        body_acc = self.data.sensordata[16:19] if self.model.nsensor >= 19 else np.zeros(3)
        angvel_b = self.data.qvel[3:6]
        
        stamp = Time(sec=int(self.timestamp), nanosec=int((self.timestamp % 1) * 1e9))
        
        imu_msg = ImuData()
        imu_msg.header = MetaType(stamp=stamp, frame_id=0)
        imu_msg.data = ImuDataValue()
        imu_msg.data.roll, imu_msg.data.pitch, imu_msg.data.yaw = map(float, rpy)
        imu_msg.data.omega_x, imu_msg.data.omega_y, imu_msg.data.omega_z = map(float, angvel_b)
        imu_msg.data.acc_x, imu_msg.data.acc_y, imu_msg.data.acc_z = map(float, body_acc)
        self.imu_pub.publish(imu_msg)

        q, dq = self.data.qpos[7:19], self.data.qvel[6:18]
        joints_msg = JointsData()
        joints_msg.header = MetaType(stamp=stamp, frame_id=0)
        joints_msg.data = JointsDataValue()
        joints_msg.data.joints_data = [JointData() for _ in range(16)]
        for i in range(12):
            j = joints_msg.data.joints_data[i]
            j.status_word = 1
            j.position, j.velocity, j.torque = float(q[i]), float(dq[i]), float(self.input_tq[i])
        for i in range(12, 16): joints_msg.data.joints_data[i].status_word = 1
        self.joints_pub.publish(joints_msg)

    def _publish_vision_and_tf(self):
        now = self.get_clock().now().to_msg()
        # Lidar (Only if beams found)
        if self.num_beams > 0:
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = now, 'lidar_link'
            scan.angle_min, scan.angle_max = 0.0, 2 * np.pi
            scan.angle_increment = (2 * np.pi) / self.num_beams
            scan.range_min, scan.range_max = 0.25, 10.0
            scan.ranges = [float(self.data.sensordata[sid]) if 0 <= self.data.sensordata[sid] < 10.0 else float('inf') for sid in self.lidar_ids]
            self.scan_pub.publish(scan)

        # Camera (Only if 'front_vision_camera' exists)
        cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, 'front_vision_camera')
        if cam_id != -1:
            # 1. Render RGB
            self.renderer.update_scene(self.data, camera='front_vision_camera')
            self.renderer.disable_depth_rendering()
            rgb = np.flipud(self.renderer.render())
            
            # 2. Render Depth
            self.renderer.enable_depth_rendering()
            depth = np.flipud(self.renderer.render())
            
            # Publish RGB
            rgb_msg = Image(header=Header(stamp=now, frame_id='camera_optical_frame'), 
                            height=240, width=320, encoding='rgb8', 
                            step=960, data=rgb.tobytes())
            self.rgb_pub.publish(rgb_msg)
            
            # Publish Depth
            depth_msg = Image(header=Header(stamp=now, frame_id='camera_optical_frame'), 
                              height=240, width=320, encoding='32FC1', 
                              step=1280, data=depth.tobytes())
            self.depth_pub.publish(depth_msg)

            info = CameraInfo(header=Header(stamp=now, frame_id='camera_optical_frame'))
            info.height, info.width = 240, 320
            f = (320 / 2) / np.tan(np.deg2rad(60 / 2))
            info.k = [f, 0.0, 160.0, 0.0, f, 120.0, 0.0, 0.0, 1.0]
            info.p = [f, 0.0, 160.0, 0.0, 0.0, f, 120.0, 0.0, 0.0, 0.0, 1.0, 0.0]
            self.info_pub.publish(info)

        # TF (Always publish)
        # 1. Odom to Base Link
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = now, 'odom', 'base_link'
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = self.data.qpos[:3].tolist()
        t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z = self.data.qpos[3:7].tolist()
        self.tf_broadcaster.sendTransform(t)

        # 2. Base Link to Lidar Link
        t_l = TransformStamped()
        t_l.header.stamp, t_l.header.frame_id, t_l.child_frame_id = now, 'base_link', 'lidar_link'
        t_l.transform.translation.x, t_l.transform.translation.y, t_l.transform.translation.z = 0.15, 0.0, 0.07 
        t_l.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_l)

        # 3. Base Link to Camera Link
        t_c = TransformStamped()
        t_c.header.stamp, t_c.header.frame_id, t_c.child_frame_id = now, 'base_link', 'camera_link'
        t_c.transform.translation.x, t_c.transform.translation.y, t_c.transform.translation.z = 0.25, 0.0, 0.05
        t_c.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_c)

        # 4. Camera Link to Optical Frame (Standard ROS camera convention)
        t_o = TransformStamped()
        t_o.header.stamp, t_o.header.frame_id, t_o.child_frame_id = now, 'camera_link', 'camera_optical_frame'
        t_o.transform.rotation.x = -0.5
        t_o.transform.rotation.y = 0.5
        t_o.transform.rotation.z = -0.5
        t_o.transform.rotation.w = 0.5
        self.tf_broadcaster.sendTransform(t_o)

if __name__ == "__main__":
    rclpy.init()
    MuJoCoLidarSimulationNode().start()
    rclpy.shutdown()
