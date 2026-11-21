# roborts_perception

Gimbal-top CSI camera (IMX219 / CAM0) perception and gesture/gimbal control.
The runtime is C++ (`rclcpp` + GStreamer C API + OpenCV + TensorRT); no PyTorch
or ultralytics import exists in any running node. The chessboard calibration
tool and the offline engine builder are Python.

See `doc/gimbal_perception_design.md` for the design.

## Nodes

| node | phase | role |
|---|---|---|
| `person_gesture_node` | P3/P4/P5 | in-process CSI capture, yolo11n-pose on TensorRT, optional hand-pose gestures |
| `gimbal_follow_node` | P6 | `/perception/persons` → absolute gimbal angles |
| `gesture_command_bridge` | P7 | gestures → `/cmd_vel_gesture`, follow toggle, target lock |
| `cmd_vel_mux` | P7 | safety > gesture > planner arbitration for `/cmd_vel_acc` |
| `pose_smoke_test` | tool | run an engine on a still image and print detections |
| `calibrate_csi` | tool | chessboard calibration (Python, offline) |
| `build_models` | tool | ONNX export + trtexec engine build (Python, offline) |

## Topics

| topic | type | producer |
|---|---|---|
| `/perception/persons` | `roborts_msgs/PersonArray` | `person_gesture_node` |
| `/perception/gestures` | `roborts_msgs/GestureArray` | `person_gesture_node` |
| `/perception/debug/image_raw`¹ | `sensor_msgs/Image` (bgr8) | `person_gesture_node` |
| `/perception/debug/camera_info`¹ | `sensor_msgs/CameraInfo` | `person_gesture_node` |
| `/cmd_gimbal_angle` | `roborts_msgs/GimbalAngle` | `gimbal_follow_node` |
| `/perception/lock_target` | `std_msgs/UInt32` | `gesture_command_bridge` |
| `/perception/follow_enable` | `std_msgs/Bool` | `gesture_command_bridge` |
| `/cmd_vel_gesture` | `geometry_msgs/Twist` | `gesture_command_bridge` |
| `/cmd_vel_acc` | `roborts_msgs/TwistAccel` | `cmd_vel_mux` |

¹ only with `publish_debug_image:=true` (best effort, depth 1).

## P3 — capture

The camera is opened **inside** `person_gesture_node` through a GStreamer
`appsink` (`nvarguscamerasrc → nvvidconv → BGRx`). No image topic is used
between capture and processing; capture and processing are separate threads and
the sink keeps one buffer (`drop=true`), so a slow consumer skips frames.

Static TF `gimbal → csi_camera_link → csi_camera_optical_frame` is published by
`roborts_bringup/launch/static_tf.launch.py` (mount offsets are launch args:
`csi_camera_x/y/z/roll/pitch/yaw`).

```bash
ros2 launch roborts_perception person_gesture.launch.py
ros2 launch roborts_perception person_gesture.launch.py publish_debug_image:=true
```

## P4 — person detection

`yolo11n-pose` → ONNX → TensorRT FP16 engine, decoded in C++ (letterbox, NMS,
17 COCO keypoints). Engines are **not committed**; build them on the robot:

```bash
ros2 run roborts_perception build_models --pose
colcon build --packages-select roborts_perception --symlink-install
# verify
ros2 run roborts_perception pose_smoke_test /tmp/people.jpg \
    $(ros2 pkg prefix roborts_perception)/share/roborts_perception/models/yolo11n-pose_fp16.engine 17 20
```

`Person` fields: `id` (greedy IoU tracker), bbox `center/size` in original
pixels, `score`, `distance` (metres, assumes `person_height_m`, default 1.7 m),
`yaw/pitch` in radians (yaw > 0 right, pitch > 0 down).

Parameters: `enable_person`, `person_engine_path`, `person_conf_threshold`,
`person_nms_iou`, `min_person_height_px`, `keypoint_conf_threshold`,
`min_valid_keypoints`, `person_height_m`. If the engine is missing or cannot be
deserialized the branch is disabled with an ERROR log; capture continues.

Measured on this Jetson: 8–19 ms per inference including H2D/D2H copies,
30 FPS end-to-end with capture.

## P5 — gestures

Enable with `enable_gesture:=true`. Requires a hand-pose engine with 21
MediaPipe-order landmarks:

```bash
ros2 run roborts_perception build_models --hand-weights /path/to/hand_best.pt
```

Community hand weights are **not** downloaded automatically (verify the
license and trust the checkpoint before exporting). For bring-up we used
`chrismuntean/YOLO11n-pose-hands` (`best.pt`, no license declared; the author
notes limited accuracy on non-open-hand gestures). It is validation-only and
must be replaced with a licensed checkpoint before delivery.

The node crops a hand ROI from each wrist (`hand_roi_scale`, default 0.28 ×
person box height), runs the engine, and classifies with rules (no training):

| id | name | rule |
|---|---|---|
| 1 | `fist` | index/middle/ring/pinky curled |
| 2 | `palm` | all five fingers extended |
| 3 | `victory` | index+middle extended, ring+pinky curled |
| 4 | `pointing_up` | index extended and up, other fingers curled |
| 5 | `thumbs_up` | thumb extended and up, other fingers curled |

A gesture is confirmed after `gesture_min_frames` (5) consecutive frames with
score ≥ `gesture_min_score` (0.6); it is edge-triggered with a
`gesture_cooldown_s` (1.5 s) cooldown and published while it is held.
`/perception/gestures` is published every processed frame (possibly empty).

## P6 — visual gimbal following

```bash
ros2 run roborts_perception gimbal_follow_node
# or together with P7:
ros2 launch roborts_perception gesture_control.launch.py
```

Control law: `target = current + gain * sign * bearing`, deadband 1°,
slew limit 60°/s, 0.5 s hold on loss, return to centre after 3 s. Yaw is
clamped to `[yaw_min_deg, yaw_max_deg]` unless `allow_chassis_follow:=true`.
Absolute `GimbalAngle` (`yaw_mode=false`, `pitch_mode=false`) is published; the
pitch is clamped to `[pitch_min_deg, pitch_max_deg]`.

**Bench test before enabling:** `yaw_sign`/`pitch_sign` default to `-1` for a
gimbal whose yaw increases counter-clockwise and pitch upward. Verify direction
on the robot first. In a closed-loop simulation the controller settled inside
the 1° deadband in ~0.5 s.

## P7 — gesture commands and velocity mux

```bash
ros2 launch roborts_perception gesture_control.launch.py
# enter gesture mode explicitly:
ros2 service call /gesture_command_bridge/enable std_srvs/srv/SetBool "{data: true}"
```

| gesture | action |
|---|---|
| `palm` | emergency stop, latched until the service re-enables |
| `thumbs_up` | forward 0.15 m/s |
| `fist` | backward 0.1 m/s |
| `victory` | toggle visual following |
| `pointing_up` | lock that person's `track_id` |

Speed is clamped to 0.2 m/s; 2 s without a hand stops the robot. The bridge is
silent while gesture mode is off, so `cmd_vel_mux` falls back to the planner.

`cmd_vel_mux` priority is safety > gesture > planner; every source expires
after 0.5 s and the output is zero when nothing is fresh. It publishes
`/cmd_vel_acc`, so when it is active the planner must not write that topic
directly. The navigation launch handles this:

```bash
ros2 launch roborts_vslam vslam_navigation.launch.py use_gesture_control:=true
```

which starts the P6/P7 nodes and remaps `local_planner` to
`/cmd_vel_acc_planner`.

## Visualization (on demand)

`person_gesture_node` publishes an annotated debug image (boxes, keypoints,
gesture names) when started with `publish_debug_image:=true`.  The
`perception_view` node adds a HUD on top: person ids with distance/yaw/pitch,
the locked target in red, gesture boxes, follow state and the current gimbal
command.  It republishes on `/perception/debug/overlay`.

```bash
# camera + perception + viewer + RViz
ros2 launch roborts_perception perception_view.launch.py

# local OpenCV window instead of RViz (needs DISPLAY)
ros2 launch roborts_perception perception_view.launch.py \
    use_rviz:=false show_window:=true

# gestures included
ros2 launch roborts_perception perception_view.launch.py enable_gesture:=true

# or view the same stream with rqt
ros2 run rqt_image_view rqt_image_view /perception/debug/overlay
```

| topic | content |
|---|---|
| `/perception/debug/image_raw` | raw debug image with perception overlays |
| `/perception/debug/overlay` | image + tracking / lock / gesture / gimbal HUD (RELIABLE depth 1 so RViz's Image display can subscribe) |

The viewer is debug-only; launch it instead of the normal perception launch
(it opens the camera).

## Calibration

`config/camera_csi.yaml` currently holds **nominal, uncalibrated** intrinsics.
Calibrate at the capture resolution (1080p):

```bash
ros2 run roborts_perception calibrate_csi --cols 9 --rows 6 \
    --square-size 0.025 --output camera_csi.yaml \
    --save-images /tmp/csi_calib --show
cp camera_csi.yaml roborts_perception/config/camera_csi.yaml
```

Acceptance: RMS reprojection error < 0.5 px. `--images DIR` calibrates from
saved images.

## Build

```bash
colcon build --packages-select roborts_msgs roborts_perception --symlink-install
```

Runtime dependencies: TensorRT 10.3 (`libnvinfer`), CUDA runtime, OpenCV,
GStreamer, yaml-cpp. Offline engine building additionally needs `torch` and
`ultralytics` (Python).
