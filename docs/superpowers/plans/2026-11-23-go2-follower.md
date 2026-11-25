# Go2 Human Follower Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan task by task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A standalone ROS 2 Humble human follower for Go2 with identity lock, LiDAR obstacle avoidance, and safe loss behavior.

**Architecture:** A camera adapter publishes ROS images; a LiDAR adapter transforms a native PointCloud2 into a base-frame cloud and a LaserScan. Perception detects people and associates the selected person's image box with 3D cloud points in base_link. A separate controller samples short velocity trajectories using the scan and is the sole live /cmd_vel publisher. Hardware and offline launches share the same perception and controller nodes.

**Tech Stack:** Python 3, ROS 2 Humble, OpenCV DNN with an external YOLOv8 pose ONNX model, NumPy, pytest, public Unitree Python SDK2 for the camera.

**Spec:** User request in this conversation, 2026-11-23.

## Global Constraints

- Repository path: `/home/vivek/go2_human_follower`.
- Use only public dependencies and interfaces; keep private source, images, models, configuration, and internal URLs out of the repository.
- Motion output is `geometry_msgs/Twist` on `/cmd_vel`.
- Native cloud input defaults to `/utlidar/cloud` and camera output to `/go2/camera/image_raw`.
- No commit or remote setup without user request.
- Live motion is disabled by default until an explicit enable service call.

## File tree and interfaces

- `src/go2_follower_interfaces`: `TargetEstimate.msg` (header in base_link, x/y, confidence, visible) and `FollowerState.msg` (state, detail).
- `src/go2_follower_perception`: `camera_adapter.py` (SDK/webcam/disabled modes -> Image), `lidar_adapter.py` (PointCloud2 -> base-frame PointCloud2 + LaserScan), `detector.py` (ONNX pose), `tracker.py` (appearance and motion association), `fusion.py` (camera box + 3D cloud association), `perception_node.py` (Image + base cloud -> TargetEstimate; Trigger lock/reset).
- `src/go2_follower_control`: `planner.py` (trajectory sampling), `controller.py` (loss and watchdog state machine), `control_node.py` (TargetEstimate + LaserScan -> `/cmd_vel`; Trigger enable/disable).
- `src/go2_follower_bringup`: `config/follower_params.yaml`; `launch/follower.launch.py` and `launch/sim_or_offline.launch.py`.
- Root: `Dockerfile`, `docker-compose.yml`, `README.md`, `tests/`.

## Review Focus

- Stale LiDAR: immediately command zero and reject motion.
- Lost target: ramp down, then wait and unlock after timeout without identity switch.
- Multiple people crossing: retain target by appearance and predicted location.
- Points behind/above robot: exclude from safety and planning slice.
- Model missing or native sensor topic absent: report fault and remain stopped.

### Task 1: Core tracking and fusion

- [x] Write failing tests for lock dwell, crossing association, loss timeout, and camera/LiDAR fusion.
- [x] Run focused pytest and confirm expected failures.
- [x] Implement `tracker.py`, `fusion.py`, and `detector.py` with pure functions/classes for testability.
- [x] Run the tests and confirm pass.

### Task 2: LiDAR safety and local motion

- [x] Write failing tests for cloud slicing, stale scan, bubble stop, trajectory obstacle rejection, and target-loss deceleration.
- [x] Run focused pytest and confirm expected failures.
- [x] Implement `lidar_adapter.py`, `planner.py`, and `controller.py`.
- [x] Run the tests and confirm pass.

### Task 3: ROS nodes and deployment

- [x] Implement camera, perception, and controller ROS nodes; messages; ament package metadata; launches; YAML.
- [x] Add Docker/Compose and README including sensor calibration, fail-safe limits, deployment, and offline mode.
- [x] Run full pytest, Python syntax, XML, YAML, Compose checks, and inspect the new repo for proprietary references.
- [ ] Run colcon build and live ROS/Jetson checks on a ROS 2 Humble Jetson; ROS is absent on this host.

## Practical limits

The onboard camera stream, exact LiDAR topic and TF, ONNX model performance, and robot stop behavior require verification on the target Jetson. The local tests exercise logic and package structure; they cannot certify collision avoidance on hardware.
