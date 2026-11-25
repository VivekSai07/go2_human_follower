"""Go2 SDK2 or local webcam camera to sensor_msgs/Image."""

import cv2


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    class CameraAdapter(Node):
        def __init__(self):
            super().__init__("go2_camera_adapter")
            for name, value in {"camera_source": "sdk", "network_interface": "eth0", "webcam_index": 0,
                                "camera_fps": 8.0, "image_topic": "/go2/camera/image_raw"}.items():
                self.declare_parameter(name, value)
            source = self.get_parameter("camera_source").value
            self.publisher = self.create_publisher(Image, self.get_parameter("image_topic").value, 10)
            self.client = None
            self.cap = None
            if source == "sdk":
                from unitree_sdk2py.core.channel import ChannelFactoryInitialize
                from unitree_sdk2py.go2.video.video_client import VideoClient
                ChannelFactoryInitialize(0, self.get_parameter("network_interface").value)
                self.client = VideoClient()
                self.client.SetTimeout(0.5)
                self.client.Init()
            elif source == "webcam":
                self.cap = cv2.VideoCapture(int(self.get_parameter("webcam_index").value))
                if not self.cap.isOpened():
                    raise RuntimeError("webcam could not be opened")
            elif source != "disabled":
                raise ValueError("camera_source must be sdk, webcam, or disabled")
            if source != "disabled":
                self.create_timer(1.0 / self.get_parameter("camera_fps").value, self.tick)

        def tick(self):
            if self.client is not None:
                code, jpeg = self.client.GetImageSample()
                if code != 0 or not jpeg:
                    self.get_logger().warning(f"Go2 camera request failed: {code}")
                    return
                frame = cv2.imdecode(__import__("numpy").frombuffer(bytes(jpeg), dtype="uint8"), cv2.IMREAD_COLOR)
            else:
                ok, frame = self.cap.read()
                if not ok:
                    return
            if frame is None:
                return
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "go2_camera"
            msg.height, msg.width = frame.shape[:2]
            msg.encoding = "bgr8"
            msg.is_bigendian = False
            msg.step = msg.width * 3
            msg.data = frame.tobytes()
            self.publisher.publish(msg)

    rclpy.init()
    node = CameraAdapter()
    try:
        rclpy.spin(node)
    finally:
        if node.cap is not None:
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()
