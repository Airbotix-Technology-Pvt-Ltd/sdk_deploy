#
# stair-step-detector — ROS2 native, PointCloud2 variant
# Subscribes to PointCloud2 (default: /depth_pcl)
# Detects stair steps via RANSAC ground plane + height histogram.
#

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from stairs_msg.msg import Stairs, StairStep, Point2


# ── PointCloud2 parser ─────────────────────────────────────────────────────────

def parse_pointcloud2(msg: PointCloud2) -> np.ndarray:
    """Fast PointCloud2 → Nx3 float32 (x,y,z). Handles variable point_step."""
    n_pts = msg.width * msg.height
    if n_pts == 0:
        return np.empty((0, 3), dtype=np.float32)
        
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n_pts, msg.point_step)
    # Find x,y,z field offsets
    off = {f.name: f.offset for f in msg.fields if f.name in ('x', 'y', 'z')}
    if len(off) < 3:
        return np.empty((0, 3), dtype=np.float32)
        
    # Extract x, y, z blocks
    x = raw[:, off['x']:off['x']+4].copy().view(np.float32).reshape(-1)
    y = raw[:, off['y']:off['y']+4].copy().view(np.float32).reshape(-1)
    z = raw[:, off['z']:off['z']+4].copy().view(np.float32).reshape(-1)
    return np.stack([x, y, z], axis=1)


# ── RANSAC ground plane ────────────────────────────────────────────────────────

def ransac_ground_plane(pts: np.ndarray,
                         n_iter: int = 60,
                         dist_thresh: float = 0.025) -> np.ndarray:
    best_n = 0
    best_plane = None
    rng = np.random.default_rng(0)
    
    # Use a subset of points for RANSAC speed
    idx = rng.choice(len(pts), min(3000, len(pts)), replace=False)
    s = pts[idx]
    
    for _ in range(n_iter):
        tri = s[rng.choice(len(s), 3, replace=False)]
        n = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        nrm = np.linalg.norm(n)
        if nrm < 1e-6:
            continue
        n /= nrm
        d = -n.dot(tri[0])
        
        # Count inliers
        with np.errstate(invalid='ignore'):
            dists = np.abs(s @ n + d)
            cnt = int(np.sum(dists < dist_thresh))
            
        if cnt > best_n:
            best_n = cnt
            best_plane = np.append(n, d)
            
    return best_plane


# ── stair detection ────────────────────────────────────────────────────────────

def detect_stair_steps(pts, plane, min_h, max_h, bin_size=0.01, min_pts=80):
    n, d = plane[:3], plane[3]
    with np.errstate(invalid='ignore'):
        heights = pts @ n + d
        mask = (heights >= min_h) & (heights <= max_h)
        
    h_f, p_f = heights[mask], pts[mask]
    if len(h_f) < min_pts:
        return []
        
    bins = np.arange(min_h, max_h + bin_size, bin_size)
    hist, edges = np.histogram(h_f, bins=bins)
    threshold = max(min_pts // 5, 15)
    
    steps, i = [], 0
    while i < len(hist):
        if hist[i] >= threshold:
            j = i
            while j < len(hist) and hist[j] >= threshold:
                j += 1
            band = p_f[(h_f >= edges[i]) & (h_f < edges[j])]
            if len(band) >= min_pts:
                steps.append((float((edges[i]+edges[j])/2.0), band))
            i = j
        else:
            i += 1
    return steps


def band_to_quad(band_pts):
    # Project to ground plane (using X and Z here as a proxy for horizontal)
    x, z = band_pts[:, 0], band_pts[:, 2]
    # Simple percentile bbox to filter outliers
    x0, x1 = float(np.nanpercentile(x, 5)),  float(np.nanpercentile(x, 95))
    z0, z1 = float(np.nanpercentile(z, 5)),  float(np.nanpercentile(z, 95))
    return [Point2(x=x0, y=z0), Point2(x=x1, y=z0),
            Point2(x=x1, y=z1), Point2(x=x0, y=z1)]


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class StairDetectorNode(Node):
    def __init__(self):
        super().__init__('stair_step_detector')

        self.declare_parameter('pointcloud_topic', '/depth_pcl')
        self.declare_parameter('min_step_height',  0.05)
        self.declare_parameter('max_step_height',  0.30)
        self.declare_parameter('min_points',       80)
        self.declare_parameter('process_every_n',  2)

        pc_t         = self.get_parameter('pointcloud_topic').value
        self.min_h   = self.get_parameter('min_step_height').value
        self.max_h   = self.get_parameter('max_step_height').value
        self.min_pts = self.get_parameter('min_points').value
        self.every_n = self.get_parameter('process_every_n').value
        self._cnt    = 0

        self.pub = self.create_publisher(Stairs, 'stairs_topic', 10)
        self.sub = self.create_subscription(
            PointCloud2, pc_t, self._cb, 10)

        self.get_logger().info(
            f'StairDetector ready — topic: {pc_t}  '
            f'step height [{self.min_h:.2f}, {self.max_h:.2f}] m')

    def _cb(self, msg: PointCloud2):
        self._cnt += 1
        if self._cnt % self.every_n != 0:
            return

        try:
            pts = parse_pointcloud2(msg)
        except Exception as e:
            self.get_logger().warn(f'Parse error: {e}')
            return

        if len(pts) == 0:
            return

        # Filter invalid points
        valid = np.isfinite(pts).all(axis=1) & (pts[:, 2] > 0.1) & (pts[:, 2] < 6.0)
        pts = pts[valid]

        if len(pts) < 500:
            return

        # Detect ground plane
        plane = ransac_ground_plane(pts)
        if plane is None:
            return

        # Coordinate check: camera frame usually has Y pointing down.
        # Normal facing "up" relative to ground should have Y > 0 in cam frame
        if plane[1] < 0:
            plane = -plane

        # Detect steps
        steps = detect_stair_steps(
            pts, plane, self.min_h, self.max_h, min_pts=self.min_pts)

        if not steps:
            return

        # Publish results
        out = Stairs()
        for h_val, band in steps:
            ss = StairStep()
            ss.height = h_val
            for i, c in enumerate(band_to_quad(band)):
                ss.quadrilateral[i] = c
            out.stair_steps.append(ss)

        self.pub.publish(out)
        self.get_logger().info(f'Detected {len(out.stair_steps)} stair step(s).')


def main(args=None):
    rclpy.init(args=args)
    node = StairDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
