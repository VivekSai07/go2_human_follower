"""Offline launch: ROS bag feeds camera/cloud, or webcam feeds camera."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    params = os.path.join(get_package_share_directory("go2_follower_bringup"), "config", "follower_params.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("camera_source", default_value="disabled"),
        DeclareLaunchArgument("cloud_topic", default_value="/utlidar/cloud"),
        Node(package="go2_follower_perception", executable="camera_adapter", name="go2_camera_adapter",
             parameters=[params, {"camera_source": LaunchConfiguration("camera_source")}]),
        Node(package="go2_follower_perception", executable="lidar_adapter", name="go2_lidar_adapter",
             parameters=[params, {"cloud_topic": LaunchConfiguration("cloud_topic")}]),
        Node(package="go2_follower_perception", executable="perception_node", name="go2_follower_perception",
             parameters=[params]),
        Node(package="go2_follower_control", executable="control_node", name="go2_follower_control",
             parameters=[params, {"cmd_vel_topic": "/follower/offline_cmd_vel"}]),
    ])
