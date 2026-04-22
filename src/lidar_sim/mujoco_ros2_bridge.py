import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import mujoco
import mujoco.viewer
import numpy as np
import time
from pathlib import Path

class MujocoRos2Bridge(Node):
    def __init__(self, model_path):
        super().__init__('mujoco_ros2_bridge')
        
        # Load MuJoCo Model
        abs_model_path = str(Path(model_path).resolve())
        self.model = mujoco.MjModel.from_xml_path(abs_model_path)
        self.data = mujoco.MjData(self.model)
        
        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Vision Camera setup (RGB + Depth)
        self.rgb_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        
        self.renderer = mujoco.Renderer(self.model, height=240, width=320)
        
        # Dynamic Lidar detection
        self.lidar_ids = []
        i = 0
        while True:
            sensor_name = f'lidar_{i}'
            sensor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
            if sensor_id == -1:
                break
            self.lidar_ids.append(sensor_id)
            i += 1
        self.num_beams = len(self.lidar_ids)
        
        self.last_pub_time = 0.0
        self.get_logger().info(f"MuJoCo ROS2 Bridge started. Detected {self.num_beams} LiDAr beams and RGB-D camera.")

    def step(self):
        mujoco.mj_step(self.model, self.data)
        
        now_time = self.data.time
        if now_time - self.last_pub_time >= 0.05: # 20Hz
            self.publish_lidar()
            self.publish_vision()
            self.publish_state()
            self.last_pub_time = now_time

    def publish_lidar(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'lidar_link'
        scan.angle_min = 0.0
        scan.angle_max = 2 * np.pi
        scan.angle_increment = (2 * np.pi) / self.num_beams
        scan.range_min = 0.25 # Ignore robot body
        scan.range_max = 10.0
        
        ranges = []
        for sensor_id in self.lidar_ids:
            val = self.data.sensordata[sensor_id]
            if val < 0 or val >= 10.0:
                ranges.append(float('inf'))
            else:
                ranges.append(float(val))
        scan.ranges = ranges
        self.scan_pub.publish(scan)

    def publish_vision(self):
        now = self.get_clock().now().to_msg()
        self.renderer.update_scene(self.data, camera='front_vision_camera')
        
        # 1. Render RGB
        self.renderer.disable_depth_rendering()
        rgb_img = np.flipud(self.renderer.render())
        
        rgb_msg = Image()
        rgb_msg.header.stamp = now
        rgb_msg.header.frame_id = 'camera_optical_frame'
        rgb_msg.height, rgb_msg.width, _ = rgb_img.shape
        rgb_msg.encoding = 'rgb8'
        rgb_msg.step = rgb_msg.width * 3
        rgb_msg.data = rgb_img.tobytes()
        self.rgb_pub.publish(rgb_msg)
        
        # 2. Render Depth
        self.renderer.enable_depth_rendering()
        depth_img = np.flipud(self.renderer.render())
        
        depth_msg = Image()
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = 'camera_optical_frame'
        depth_msg.height, depth_msg.width = depth_img.shape
        depth_msg.encoding = '32FC1'
        depth_msg.step = depth_msg.width * 4
        depth_msg.data = depth_img.tobytes()
        self.depth_pub.publish(depth_msg)
        
        # 3. Camera Info
        info = CameraInfo()
        info.header.stamp = now
        info.header.frame_id = 'camera_optical_frame'
        info.height, info.width = 240, 320
        f = (320 / 2) / np.tan(np.deg2rad(60 / 2))
        info.k = [f, 0.0, 160.0, 0.0, f, 120.0, 0.0, 0.0, 1.0]
        info.p = [f, 0.0, 160.0, 0.0, 0.0, f, 120.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)

    def publish_state(self):
        now = self.get_clock().now().to_msg()
        q = self.data.qpos[0:3]
        quat = self.data.qpos[3:7] # w, x, y, z
        
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = q[0]
        odom.pose.pose.position.y = q[1]
        odom.pose.pose.position.z = q[2]
        odom.pose.pose.orientation.w = quat[0]
        odom.pose.pose.orientation.x = quat[1]
        odom.pose.pose.orientation.y = quat[2]
        odom.pose.pose.orientation.z = quat[3]
        self.odom_pub.publish(odom)

        # Base to Lidar
        t_l = TransformStamped()
        t_l.header.stamp = now
        t_l.header.frame_id = 'base_link'
        t_l.child_frame_id = 'lidar_link'
        t_l.transform.translation.x = 0.15
        t_l.transform.translation.z = 0.07
        t_l.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_l)

        # Base to Camera
        t_c = TransformStamped()
        t_c.header.stamp = now
        t_c.header.frame_id = 'base_link'
        t_c.child_frame_id = 'camera_link'
        t_c.transform.translation.x = 0.25
        t_c.transform.translation.z = 0.05
        t_c.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_c)

        # Camera to Optical
        t_o = TransformStamped()
        t_o.header.stamp = now
        t_o.header.frame_id = 'camera_link'
        t_o.child_frame_id = 'camera_optical_frame'
        t_o.transform.rotation.x = -0.5
        t_o.transform.rotation.y = 0.5
        t_o.transform.rotation.z = -0.5
        t_o.transform.rotation.w = 0.5
        self.tf_broadcaster.sendTransform(t_o)

def main():
    rclpy.init()
    bridge = MujocoRos2Bridge('lite3_lidar.xml')
    
    with mujoco.viewer.launch_passive(bridge.model, bridge.data) as viewer:
        while rclpy.ok():
            bridge.step()
            viewer.sync()
            time.sleep(0.01) # Simple loop

if __name__ == '__main__':
    main()
