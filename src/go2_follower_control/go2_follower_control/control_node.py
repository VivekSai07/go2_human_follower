"""ROS controller. Sole /cmd_vel publisher with an explicit enable gate."""

import time

import numpy as np

from .controller import FollowController


def main() -> None:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_srvs.srv import Trigger
    from go2_follower_interfaces.msg import FollowerState, TargetEstimate

    class Control(Node):
        def __init__(self):
            super().__init__("go2_follower_control")
            for name, value in {"scan_topic": "/follower/scan", "cmd_vel_topic": "/cmd_vel",
                                "safety_bubble_m": 0.35, "max_linear_mps": 0.4, "max_angular_rps": 0.8,
                                "follow_distance_m": 1.35, "sensor_timeout_s": 0.5,
                                "target_timeout_s": 5.0, "max_accel_mps2": 0.5,
                                "calibration_verified": False}.items():
                self.declare_parameter(name, value)
            p = lambda name: self.get_parameter(name).value
            self.controller = FollowController(p("safety_bubble_m"), p("max_linear_mps"), p("max_angular_rps"),
                                               p("follow_distance_m"), p("sensor_timeout_s"),
                                               p("target_timeout_s"), p("max_accel_mps2"))
            self.publisher = self.create_publisher(Twist, p("cmd_vel_topic"), 10)
            self.state_pub = self.create_publisher(FollowerState, "/follower/state", 10)
            self.create_subscription(LaserScan, p("scan_topic"), self.on_scan, qos_profile_sensor_data)
            self.create_subscription(TargetEstimate, "/follower/target", self.on_target, 10)
            self.create_service(Trigger, "/follower/enable", self.enable)
            self.create_service(Trigger, "/follower/disable", self.disable)
            self.create_timer(0.05, self.tick)

        def enable(self, _request, response):
            if not self.get_parameter("calibration_verified").value:
                response.success = False
                response.message = "Verify camera/LiDAR calibration and set calibration_verified=true first"
                return response
            if self.controller.scan_at is None or time.monotonic() - self.controller.scan_at > self.controller.sensor_timeout:
                response.success = False
                response.message = "Fresh LiDAR scan required"
                return response
            self.controller.set_enabled(True)
            response.success = True
            response.message = "Follower enabled"
            return response

        def disable(self, _request, response):
            self.controller.set_enabled(False)
            self.publisher.publish(Twist())
            response.success = True
            response.message = "Follower disabled"
            return response

        def on_target(self, msg):
            self.controller.update_target(msg.x if msg.visible else None, msg.y if msg.visible else None,
                                          time.monotonic())

        def on_scan(self, msg):
            self.controller.update_scan(np.asarray(msg.ranges), time.monotonic(), msg.angle_min, msg.angle_increment)

        def tick(self):
            v, w = self.controller.step(time.monotonic())
            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = w
            self.publisher.publish(cmd)
            state = FollowerState()
            state.header.stamp = self.get_clock().now().to_msg()
            state.state = self.controller.state
            state.detail = ""
            self.state_pub.publish(state)

    rclpy.init()
    node = Control()
    try:
        rclpy.spin(node)
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
