# AGENTS.md — Go2 Human Follower

This file is the current working guide for coding agents in this repository. Read it together with [README.md](README.md) and the current [architecture](docs/ARCHITECTURE.md) before changing code. The [implementation plan](docs/superpowers/plans/2026-11-23-go2-follower.md) records how the first version was built; check the current source before treating a plan or example value as fact.

## Project boundary

- This is a **personal, public, standalone** ROS 2 Humble project at `VivekSai07/go2_human_follower`. Keep its code and deployment self-contained. Use public dependencies and interfaces; do not import private source, images, models, configuration, or internal URLs from other projects.
- The only intended robot dependencies are public Unitree SDK2/`unitree_ros2` interfaces and standard open source ROS 2 libraries. The project publishes standard `geometry_msgs/Twist` to an already running Go2 motor bridge; it does not implement or bundle that bridge.
- Model weights, ROS bags, calibration measurements, and secrets are not in Git. `models/`, `bags/`, `*.onnx`, and `*.engine` are ignored. Obtain weights with a license appropriate for their intended use.
- Keep Git author identity personal. This checkout uses the repository-local GitHub noreply identity, not a corporate email. Do not add AI co-author trailers. Commit and push only when the user requests it.

## Current state — checked 2026-11-25

- Four ROS 2 packages and their launch/config files exist. The pure Python suite passed **16 tests**; Python syntax, package XML, YAML, and Compose configuration were parsed successfully. Re-run checks before claiming current results.
- **No ROS 2/`colcon` build or Docker image build was run in the development environment. No Go2/Jetson test, sensor calibration, collision-avoidance validation, or stop-latency measurement has been completed.** Do not describe the stack as hardware verified or production ready.
- The Go2 camera service, `/utlidar/cloud` availability/type/QoS, actual frame transforms, model frame rate, and `/cmd_vel` stopping behavior must be checked on the target Jetson.
- `calibration_verified` defaults to `false`; `/follower/enable` refuses live motion until an operator measures calibration, sets that parameter true, and provides a fresh LiDAR scan. The offline launch redirects velocity output away from `/cmd_vel`.

Next evidence to collect: (1) `colcon build` and Docker build on the intended Jetson; (2) actual camera and `/utlidar/cloud` frames, type, and QoS; (3) camera/LiDAR extrinsics and ONNX inference rate; (4) bag-based crossing/occlusion/obstacle tests; (5) controlled low-speed hardware tests with a human operator and an independent stop. Record measured results before updating the status above.

## Package and data flow

| Package | Main files | Role |
| --- | --- | --- |
| `go2_follower_interfaces` | `msg/TargetEstimate.msg`, `msg/FollowerState.msg` | ROS message contracts. `TargetEstimate` carries `x`, `y`, confidence, and visibility in `base_link`. |
| `go2_follower_perception` | `camera_adapter.py`, `lidar_adapter.py`, `detector.py`, `tracker.py`, `fusion.py`, `perception_node.py` | SDK/webcam image ingestion; native cloud transform and planar projection; ONNX pose detection; person lock and color/spatial re-identification; camera-box to 3D cloud association. |
| `go2_follower_control` | `planner.py`, `controller.py`, `control_node.py` | Short-horizon sampled local planner, watchdog, safety stops, and the sole follower `/cmd_vel` publisher. |
| `go2_follower_bringup` | `config/follower_params.yaml`, `launch/*.launch.py` | ROS parameters and hardware/offline launches. |

Actual runtime path:

```text
Unitree VideoClient or webcam -> /go2/camera/image_raw --+
                                                    perception -> /follower/target --+
/utlidar/cloud -> lidar_adapter -> /follower/cloud_base --+                        |
                             \-> /follower/scan ---------------------------- control -> /cmd_vel
```

- Camera adapter modes: `sdk`, `webcam`, `disabled`. `disabled` lets a ROS bag supply `/go2/camera/image_raw`. The SDK mode calls the public Unitree `VideoClient.GetImageSample()` API and publishes `bgr8` images.
- LiDAR adapter subscribes `/utlidar/cloud` as `sensor_msgs/PointCloud2`; it publishes a transformed `/follower/cloud_base` and a height-sliced 2D `/follower/scan`. Its transform is configured by `lidar_x_m`, `lidar_y_m`, `lidar_z_m`, `lidar_yaw_rad`; it does **not** read TF or model LiDAR roll/pitch.
- Perception loads an external **COCO 17-keypoint YOLOv8 pose ONNX** model through OpenCV DNN. It locks a centered person after `lock_dwell_s` (default 2 s), or on a raised wrist, and tracks using torso color histogram plus predicted image position. `/follower/lock_target` queues a lock of the next centered detection; `/follower/reset_target` clears it. These are `std_srvs/Trigger` services.
- Perception matches a detected person's box against 3D points in `/follower/cloud_base` using configured camera position, yaw, pitch, and field of view. It publishes `/follower/target` only with `visible=true` when a recent cloud supplies a match. This association is approximate and depends on measured extrinsics; it can confuse a foreground object with a person.
- Controller consumes `/follower/target` and `/follower/scan`, samples local trajectories around LiDAR obstacles, and publishes `/cmd_vel` plus `/follower/state`. `/follower/enable` and `/follower/disable` are `std_srvs/Trigger`. In `sim_or_offline.launch.py`, commands go to `/follower/offline_cmd_vel`.

## Safety behavior and limits

- Motion is disabled at launch. The controller sends zero velocity on stale LiDAR, too few valid returns, no forward returns, a point inside the configured `safety_bubble_m` (default 0.35 m), or an obstacle within its speed-dependent braking threshold. It decelerates toward zero while a target is absent or stale. The tracker unlocks after `loss_timeout_s` (default 5 s).
- The 0.35 m bubble and zero `/cmd_vel` are **software protective behavior, not a certified emergency stop**. Keep an independent operator stop path. Confirm the actual motor bridge stops promptly on zero velocity before live following.
- The planner is reactive. It has no global route plan, semantic obstacle prediction, or guarantee of safely passing people in a narrow corridor. Color-histogram re-identification can switch identities when people look alike. The current code has no synchronized image/cloud timestamps or TF-based calibration.
- The YAML camera and LiDAR extrinsics/FOV are placeholders. Never set `calibration_verified: true` based on the placeholder values. Do not raise speed or shrink safety margins without measured robot data.

## Build and verification

From the repository root:

```bash
python3 -m pytest tests -q
docker compose config --quiet
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch go2_follower_bringup sim_or_offline.launch.py camera_source:=disabled
```

`pytest` exercises the ROS-independent tracking, cloud fusion, LiDAR slicing, planner, and watchdog. It does **not** prove ROS package integration or hardware safety. If ROS 2 is unavailable, report the unrun build explicitly. Docker/Jetson instructions and calibration checklist are in [README.md](README.md). No licensed ONNX model is bundled, so perception cannot produce a target until one is installed.

## When changing the project

1. Check Git status and read the relevant current files before relying on notes or an old plan. Preserve unrelated user changes.
2. Keep sensor ingestion, perception, and motion control separate. Keep pure tracking/fusion/control logic testable without ROS imports; put ROS I/O in node modules.
3. Add or adjust focused tests for behavior changes, especially lock identity, stale inputs, obstacle proximity, braking, and target loss. Run the full pure Python suite and any ROS/Docker checks available in the environment; report exactly what ran.
4. Update `README.md`, [architecture](docs/ARCHITECTURE.md), this file, and `CLAUDE.md` when interfaces, safety behavior, deployment steps, or verified status change. Distinguish code behavior from hardware observations and assumptions.
5. Before any public push, inspect staged files for weights, bags, credentials, private URLs, and corporate content. Use a personal commit identity. Do not claim on-hardware success from local tests.
