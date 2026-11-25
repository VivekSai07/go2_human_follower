"""Point cloud projection, plus a ROS node that publishes a planar scan."""

import math

import numpy as np


def project_points(points: np.ndarray, bins: int = 181, min_z: float = 0.05, max_z: float = 0.7,
                   min_range: float = 0.1, max_range: float = 8.0, angle_min: float = -math.pi / 2,
                   angle_max: float = math.pi / 2) -> np.ndarray:
    ranges = np.full(bins, np.inf, dtype=float)
    if points.size == 0:
        return ranges
    xyz = np.asarray(points, dtype=float).reshape(-1, 3)
    r = np.hypot(xyz[:, 0], xyz[:, 1])
    a = np.arctan2(xyz[:, 1], xyz[:, 0])
    good = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] >= min_z) & (xyz[:, 2] <= max_z)
    good &= (r >= min_range) & (r <= max_range) & (a >= angle_min) & (a <= angle_max)
    indices = np.clip(np.rint((a[good] - angle_min) * (bins - 1) / (angle_max - angle_min)).astype(int), 0, bins - 1)
    np.minimum.at(ranges, indices, r[good])
    return ranges


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header

    class LidarAdapter(Node):
        def __init__(self):
            super().__init__("go2_lidar_adapter")
            for name, value in {"cloud_topic": "/utlidar/cloud", "scan_topic": "/follower/scan",
                                "lidar_x_m": 0.0, "lidar_y_m": 0.0, "lidar_z_m": 0.0,
                                "lidar_yaw_rad": 0.0,
                                "min_z_m": 0.05, "max_z_m": 0.7}.items():
                self.declare_parameter(name, value)
            self.pub = self.create_publisher(LaserScan, self.get_parameter("scan_topic").value, 10)
            self.cloud_pub = self.create_publisher(PointCloud2, "/follower/cloud_base", qos_profile_sensor_data)
            self.create_subscription(PointCloud2, self.get_parameter("cloud_topic").value, self.on_cloud, qos_profile_sensor_data)

        def on_cloud(self, msg):
            try:
                points = np.asarray(list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)), dtype=float).reshape(-1, 3)
                yaw = self.get_parameter("lidar_yaw_rad").value
                c, s = math.cos(yaw), math.sin(yaw)
                if len(points):
                    xy = points[:, :2].copy()
                    points[:, 0] = c * xy[:, 0] - s * xy[:, 1] + self.get_parameter("lidar_x_m").value
                    points[:, 1] = s * xy[:, 0] + c * xy[:, 1] + self.get_parameter("lidar_y_m").value
                    points[:, 2] += self.get_parameter("lidar_z_m").value
                header = Header()
                header.stamp = msg.header.stamp
                header.frame_id = "base_link"
                self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points.astype(np.float32).tolist()))
                ranges = project_points(points, bins=721, angle_min=-math.pi, angle_max=math.pi,
                                        min_z=self.get_parameter("min_z_m").value,
                                        max_z=self.get_parameter("max_z_m").value)
                scan = LaserScan()
                scan.header = header
                scan.angle_min = -math.pi
                scan.angle_max = math.pi
                scan.angle_increment = 2 * math.pi / 720
                scan.range_min = 0.1
                scan.range_max = 8.0
                scan.ranges = ranges.tolist()
                self.pub.publish(scan)
            except Exception as exc:
                self.get_logger().error(f"LiDAR conversion failed: {exc}")

    rclpy.init()
    node = LidarAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
