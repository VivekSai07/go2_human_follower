"""Pose detection, target identity, and bearing-range fusion."""

import math
import time

import numpy as np

from .detector import PoseDetector
from .fusion import associate_bbox_cloud
from .tracker import TargetTracker


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_srvs.srv import Trigger
    from go2_follower_interfaces.msg import TargetEstimate

    class Perception(Node):
        def __init__(self):
            super().__init__("go2_follower_perception")
            for name, value in {"image_topic": "/go2/camera/image_raw", "cloud_base_topic": "/follower/cloud_base",
                                "model_path": "/models/yolov8n-pose.onnx", "camera_horizontal_fov_deg": 100.0,
                                "camera_vertical_fov_deg": 75.0, "camera_x_m": 0.0, "camera_y_m": 0.0,
                                "camera_z_m": 0.45, "camera_yaw_deg": 0.0, "camera_pitch_deg": 0.0,
                                "lock_dwell_s": 2.0, "loss_timeout_s": 5.0,
                                "cloud_max_age_s": 0.5}.items():
                self.declare_parameter(name, value)
            self.detector = None
            try:
                self.detector = PoseDetector(self.get_parameter("model_path").value)
            except Exception as exc:
                self.get_logger().error(f"Pose detector unavailable; follower stays stopped: {exc}")
            self.tracker = TargetTracker(self.get_parameter("lock_dwell_s").value,
                                         self.get_parameter("loss_timeout_s").value)
            self.cloud = None
            self.cloud_received = None
            self.publisher = self.create_publisher(TargetEstimate, "/follower/target", 10)
            self.create_subscription(Image, self.get_parameter("image_topic").value, self.on_image, qos_profile_sensor_data)
            self.create_subscription(PointCloud2, self.get_parameter("cloud_base_topic").value, self.on_cloud, qos_profile_sensor_data)
            self.create_service(Trigger, "/follower/lock_target", self.lock)
            self.create_service(Trigger, "/follower/reset_target", self.reset)

        def lock(self, _request, response):
            self.tracker.request_lock()
            response.success = True
            response.message = "Next centered detected person will lock"
            return response

        def reset(self, _request, response):
            self.tracker.reset()
            response.success = True
            response.message = "Target reset"
            return response

        def on_cloud(self, msg):
            try:
                self.cloud = np.asarray(list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)), dtype=float).reshape(-1, 3)
                self.cloud_received = time.monotonic()
            except Exception as exc:
                self.cloud = None
                self.get_logger().error(f"Point cloud read failed: {exc}")

        def on_image(self, msg):
            estimate = TargetEstimate()
            estimate.header.stamp = msg.header.stamp
            estimate.header.frame_id = "base_link"
            if self.detector is None or msg.encoding not in ("bgr8", "rgb8"):
                self.publisher.publish(estimate)
                return
            try:
                image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
                image = image.reshape(msg.height, msg.width, 3)
                if msg.encoding == "rgb8":
                    image = image[:, :, ::-1].copy()
                self.tracker.image_width = msg.width
                candidate = self.tracker.update(self.detector.detect(image), time.monotonic())
                if candidate is not None and self.cloud is not None and self.cloud_received is not None:
                    if time.monotonic() - self.cloud_received <= self.get_parameter("cloud_max_age_s").value:
                        p = lambda name: self.get_parameter(name).value
                        position = associate_bbox_cloud(self.cloud, candidate.box, msg.width, msg.height,
                                                        p("camera_horizontal_fov_deg"), p("camera_vertical_fov_deg"),
                                                        (p("camera_x_m"), p("camera_y_m"), p("camera_z_m")),
                                                        math.radians(p("camera_yaw_deg")),
                                                        math.radians(p("camera_pitch_deg")))
                        if position is not None:
                            estimate.x = position[0]
                            estimate.y = position[1]
                            estimate.confidence = float(candidate.confidence)
                            estimate.visible = True
            except Exception as exc:
                self.get_logger().error(f"Perception frame failed: {exc}")
            self.publisher.publish(estimate)

    rclpy.init()
    node = Perception()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
