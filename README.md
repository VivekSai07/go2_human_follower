# Go2 Human Follower

Standalone ROS 2 Humble prototype for selecting one person and following them with a Unitree Go2. The project uses standard ROS 2 messages and the public Unitree SDK2 Python camera API.

**Status:** logic tested off-robot; live Go2 sensor geometry, model throughput, bridge behavior, and stopping distance still require on-robot validation. Keep the remote operator ready to stop the robot. The 0.35 m bubble is a software protective stop, not a certified emergency stop.

## Repository layout

```text
src/
  go2_follower_interfaces/    TargetEstimate and FollowerState messages
  go2_follower_perception/    SDK/webcam camera, LiDAR cloud adapter, pose detector,
                             appearance tracker, 3D camera/cloud fusion
  go2_follower_control/       sampled local planner, watchdog, /cmd_vel publisher
  go2_follower_bringup/       launch files and follower_params.yaml
tests/                       pure Python tests for tracking, fusion and control
Dockerfile                   ROS 2 Humble arm64 build, public Unitree SDK dependency
docker-compose.yml           Jetson host-network deployment
```

## Interfaces

| Producer | Output | Consumer |
| --- | --- | --- |
| Go2 camera adapter | `/go2/camera/image_raw` (`sensor_msgs/Image`, `bgr8`) | Perception |
| Go2 firmware | `/utlidar/cloud` (`sensor_msgs/PointCloud2`) | LiDAR adapter |
| LiDAR adapter | `/follower/cloud_base` (`PointCloud2`, `base_link`) | Perception |
| LiDAR adapter | `/follower/scan` (`LaserScan`, `base_link`) | Controller |
| Perception | `/follower/target` (`TargetEstimate`) | Controller |
| Controller | `/cmd_vel` (`geometry_msgs/Twist`) | Your Go2 motor bridge |
| Controller | `/follower/state` (`FollowerState`) | Operator |

`/follower/lock_target` and `/follower/reset_target` are `std_srvs/Trigger`. A raised wrist or a person held near image center for two seconds also locks. Lock is retained through brief occlusion using a torso color histogram and predicted image position. If no match is seen for five seconds, the tracker unlocks; commands decelerate toward zero while the target is absent. `/follower/enable` and `/follower/disable` gate motion; launch starts disabled. The offline launch publishes commands to `/follower/offline_cmd_vel` instead.

The sampled planner checks short forward arcs against LiDAR points and prefers progress toward the target. It reacts to each new scan; it does not predict other people's future motion or provide global route planning. Narrow corridors may require stopping and waiting for a clear path. The controller commands zero when the scan is stale, has too few valid returns, or contains an obstacle inside the safety radius or current braking distance. A missing pose model or failed camera/cloud association also prevents following.

## Prerequisites

- Go2 with usable SDK2 video service and `/utlidar/cloud`; verify both on your hardware. The official [Unitree ROS 2 guide](https://github.com/unitreerobotics/unitree_ros2) shows the LiDAR topic, and the [SDK2 Python guide](https://github.com/unitreerobotics/unitree_sdk2_python) documents the camera example.
- A running Go2 bridge that accepts `geometry_msgs/Twist` on `/cmd_vel`. Run **one** velocity command publisher at a time. This repo does not install or alter the bridge.
- ROS 2 Humble on Ubuntu 22.04, CycloneDDS, Python 3.10, OpenCV, NumPy, and `sensor_msgs_py`. The Dockerfile installs these and the public Unitree SDK2 Python package. No GPU framework is required; the ONNX model runs through OpenCV DNN. Measure actual frame rate on the Jetson before driving.
- A COCO 17-keypoint **YOLOv8 pose ONNX** model at `models/yolov8n-pose.onnx`. Model weights are not bundled. Export a model you are licensed to use; review its license independently. A differently shaped ONNX model needs a new decoder.

## Local build and tests

```bash
cd /path/to/go2_human_follower
python3 -m pytest tests -q
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch go2_follower_bringup sim_or_offline.launch.py camera_source:=webcam
```

For a bag, use `camera_source:=disabled` and replay `/go2/camera/image_raw` plus `/utlidar/cloud`. The offline launch never publishes on `/cmd_vel`. Inspect `/follower/target`, `/follower/scan`, `/follower/state`, and `/follower/offline_cmd_vel` before any live run. A webcam alone is insufficient for motion because the controller requires a fresh LiDAR scan.

## Jetson deployment

1. Pull **this** repository on the Jetson. Set `UNITREE_NET_IFACE` to the Ethernet interface that reaches the Go2 (for example `eth0`); keep the ROS domain equal to the motor bridge's domain. Confirm `ros2 topic info /utlidar/cloud -v` reports `sensor_msgs/msg/PointCloud2` with compatible QoS.
2. Place the licensed ONNX model in `models/`. Edit `src/go2_follower_bringup/config/follower_params.yaml` for your own robot, especially all camera and LiDAR calibration values. Keep `calibration_verified: false` until measured.
3. Build and start the follower container:

   ```bash
   export UNITREE_NET_IFACE=eth0
   docker compose build
   docker compose up
   ```

4. In another ROS 2 Humble terminal on the same domain, confirm camera frames, cloud, planar scan, and target distance while the robot is stationary. Once measured calibration is correct, set `calibration_verified: true`, rebuild/restart, and enable with `ros2 service call /follower/enable std_srvs/srv/Trigger '{}'`. Watch `/follower/state` and `/cmd_vel`. Disable with `ros2 service call /follower/disable std_srvs/srv/Trigger '{}'`.

The Dockerfile uses a public ROS 2 Humble arm64 base and installs public SDK2 Python from GitHub. It has **not** been built or tested on this machine's Jetson. Unitree's Python SDK pins CycloneDDS 0.10.2; if its ARM64 wheel is unavailable for the Jetson image, install a matching CycloneDDS build and set `CYCLONEDDS_HOME` as described in the [official SDK guide](https://github.com/unitreerobotics/unitree_sdk2_python#installation).

## Calibration checklist

The configured values are **placeholders**, not measured Go2 extrinsics:

- Confirm the raw cloud's frame with `ros2 topic echo --no-arr /utlidar/cloud` and measure `lidar_x_m`, `lidar_y_m`, `lidar_z_m`, and `lidar_yaw_rad` into `base_link`. The adapter applies a rigid transform but does not currently read TF.
- Measure camera position/yaw/pitch in `base_link` and calibrate horizontal/vertical FOV. Check projected points against a standing person at several known distances and across the image. An approximate FOV can associate a wall with the person.
- Verify `min_z_m`/`max_z_m` include body-height obstacles but reject floor/ceiling and self returns. Set `safety_bubble_m`, limits, and following distance based on measured braking distance and corridor width.
- Inspect target identity through crossing people, temporary occlusion, lighting changes, and similar clothing. Color histogram re-identification is lightweight and can fail when appearances match.
- Confirm that zero `/cmd_vel` stops your particular motor bridge promptly. An independent operator stop path must remain available for live tests.

## Public references

- [Unitree SDK2 Python camera API](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/go2/video/video_client.py)
- [Unitree ROS 2 Go2 LiDAR documentation](https://github.com/unitreerobotics/unitree_ros2)
- [ROS 2 Humble documentation](https://docs.ros.org/en/humble/)

No code from those repositories is copied into this project; they are external dependencies and interface references.
