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
| `gesture_command_bridge` | P7 | gesture tracking switch + motion commands → `/perception/{lock_target,follow_enable}`, `/cmd_vel_gesture` |
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

## Integrated P3–P7 launch

`perception_control.launch.py` starts the whole gimbal perception/control stack
as one unit:

| node | phase |
|---|---|
| `person_gesture_node` | P3 CSI capture, P4 persons, P5 gestures |
| `gimbal_follow_node` | P6 visual following |
| `gesture_command_bridge` | P7 open-hand↔fist tracking switch, thumbs_up motion |
| `cmd_vel_mux` | P7 safety > gesture > planner → `/cmd_vel_acc` |

Tracking is idle at start: show an open hand (a `palm`, or `pointing_up` /
`victory` if that is how the model labels it), then close it into a fist to
lock that person and start following; open the hand again to release and stop.

```bash
# camera + P4/P5 + P6/P7 (gestures enabled by default)
ros2 launch roborts_perception perception_control.launch.py

# with the perception HUD in RViz (turns on the debug image)
ros2 launch roborts_perception perception_control.launch.py use_viewer:=true

# headless / local window instead of RViz
ros2 launch roborts_perception perception_control.launch.py \
    use_viewer:=true use_rviz:=false show_window:=true

# allow thumbs_up forward motion
ros2 launch roborts_perception perception_control.launch.py gesture_enabled:=true
```

Requirements:
- By default this launch also starts `roborts_base` (`start_base:=true`), which
  publishes `base_link → gimbal` and consumes `/cmd_gimbal_angle` and
  `/cmd_vel_acc`. If a bringup already starts the base, pass
  `start_base:=false` or the serial port will be opened twice.
- The main bringup already publishes the CSI static TF. Set
  `publish_csi_tf:=true` only for a standalone run, never both.
- `cmd_vel_mux` writes `/cmd_vel_acc`, so a planner running at the same time
  must publish `/cmd_vel_acc_planner` instead.
  `vslam_navigation.launch.py use_gesture_control:=true` does that remap and
  includes this launch with `start_base:=false`, so navigation + gestures works
  from one top-level launch.

The individual launches remain available: `person_gesture.launch.py` (P3–P5)
and `gesture_control.launch.py` (P6/P7).

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
| 2 | `palm` | index/middle/ring/pinky extended; the thumb may be tucked |
| 3 | `victory` | index+middle extended, ring+pinky curled |
| 4 | `pointing_up` | index extended and up, other fingers curled |
| 5 | `thumbs_up` | thumb extended and up, other fingers curled |

A gesture is confirmed after `gesture_min_frames` (5) consecutive frames with
score ≥ `gesture_min_score` (0.6); it is edge-triggered with a
`gesture_cooldown_s` (0.8 s) cooldown and published while it is held.
`/perception/gestures` is published every processed frame (possibly empty).
The short cooldown keeps the open-hand → fist tracking edge responsive.

## P6 — visual gimbal following

```bash
ros2 run roborts_perception gimbal_follow_node
# or together with P7:
ros2 launch roborts_perception gesture_control.launch.py
```

Control law: `target = current + gain * sign * bearing`, deadband 0.8°,
slew limit 180°/s (50 Hz control loop), 0.5 s hold on loss, return to centre
after 3 s. `yaw_gain`/`pitch_gain` default 0.9. Yaw is
clamped to `[yaw_min_deg, yaw_max_deg]` unless `allow_chassis_follow:=true`.
Absolute `GimbalAngle` (`yaw_mode=false`, `pitch_mode=false`) is published; the
pitch is clamped to `[pitch_min_deg, pitch_max_deg]`.

Following starts **off**: `gesture_command_bridge` turns it on together with a
target lock on the open-hand → fist edge. Set `follow_enabled:=true` only if
the gimbal should follow the best-scoring person before any gesture.

**Bench test before enabling:** `yaw_sign`/`pitch_sign` default to `-1` for a
gimbal whose yaw increases counter-clockwise and pitch upward. Verify direction
on the robot first. In a closed-loop simulation the controller settled inside
the 1° deadband in ~0.5 s.

## P7 — gesture commands and velocity mux

```bash
# one launch for P3-P7 (camera + persons + gestures + follow + mux)
ros2 launch roborts_perception perception_control.launch.py

# optional: allow thumbs_up forward motion
ros2 service call /gesture_command_bridge/enable std_srvs/srv/SetBool "{data: true}"
```

For P6/P7 only (camera/perception already running elsewhere), use
`ros2 launch roborts_perception gesture_control.launch.py`.

| gesture | action |
|---|---|
| open hand → `fist` | lock that person's `track_id` and start gimbal following |
| `fist` → open hand | release the lock and stop following (also stops motion) |
| `thumbs_up` | forward 0.15 m/s (needs gesture mode via `~/enable`) |

"Open hand" means `palm`, but `pointing_up` and `victory` are also accepted:
with the current hand-pose model a loosely opened hand is often labelled as
one of those instead of `palm`. The tracking edge is per `track_id` and
accepted when the open hand and fist are seen within `transition_timeout_s`
(1.5 s); short detection gaps are tolerated. It works without entering gesture
mode because it only moves the gimbal. Any forward speed is clamped to
0.2 m/s and 2 s without a hand stops the robot. The bridge is silent while
gesture mode is off, so `cmd_vel_mux` falls back to the planner.

Tracking demo:

```bash
# one terminal: camera + P4/P5 + P6/P7, with the HUD in RViz
ros2 launch roborts_perception perception_control.launch.py use_viewer:=true

# in view: show an open hand, then close it into a fist -> lock + follow
#          open the hand again (fist -> open hand)         -> release + stop
```

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
