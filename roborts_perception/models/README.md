# Model artifacts

Engines are generated **on the target Jetson** with the TensorRT version used
at runtime; they are not committed (see `.gitignore`).

```bash
# P4 person pose engine (downloads yolo11n-pose.pt, exports ONNX, runs trtexec)
ros2 run roborts_perception build_models --pose

# P5 hand pose engine, from a hand-pose checkpoint you have verified/licensed
ros2 run roborts_perception build_models --hand-weights /path/to/best.pt

# Let the installed launch file see newly generated files
colcon build --packages-select roborts_perception --symlink-install
```

Expected files:

| file | used by |
|---|---|
| `yolo11n-pose_fp16.engine` | `person_gesture_node` (P4) |
| `hand_pose_fp16.engine` | `person_gesture_node` with `enable_gesture:=true` (P5) |

## Hand pose model (P5)

The hand engine is built from the community checkpoint named in the design doc:

- source: `chrismuntean/YOLO11n-pose-hands` → `runs/pose/train/weights/best.pt`
- sha256: `e60bb8d9a25d61c4b8238c84afe07453fc7daa8f0adfe98d14a58ff2f08c4af3`
- exported at `imgsz=320`, output `(1, 68, 2100)` (5 + 21×3)
- MediaPipe landmark order (0 wrist, 1–4 thumb, 5–8 index, 9–12 middle,
  13–16 ring, 17–20 pinky), matching `gesture_classifier.cpp`

**License/quality caveat:** the repository declares no license, and its author
notes limited accuracy on non-open-hand gestures (fist, thumbs-up, …). It is
used here for bring-up/validation only. Replace it with a licensed checkpoint
before delivery. The `.pt` is not committed (see `.gitignore`).

Training/export uses `torch` and `ultralytics`; the running node only uses
TensorRT (`libnvinfer` + CUDA runtime).
