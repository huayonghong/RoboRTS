# 云台 CSI 感知实现总结（P3–P7）

> **日期**：2026-11-22
> **分支**：`ros2`（基线 `8d81f3f` 之后）
> **设计文档**：`doc/gimbal_perception_design.md`
> **状态**：P3–P7 已实现，并提供 `perception_control.launch.py` 一体化启动；
> 本机（Jetson Orin Nano，L4T R36.4.3 / TensorRT 10.3）完成验证；
> P5 端到端手势（社区权重仅验证）、P6 云台符号与限位、P7 底盘联调仍需上车确认。
> **说明**：XVF3800 存在硬件故障，语音侧 P1/P2 暂缓；本轮先完成视觉侧。

---

## 0. 阶段一览

| 阶段 | 日期 | 功能 | 状态 | 本机验证 |
|---|---|---|---|---|
| P3 | 2026-11-21 | CSI 取流、相机标定、静态 TF | ✅ 已实现 | 1080p@30，30.0 FPS，0 跳帧 |
| P4 | 2026-11-21 | yolo11n-pose TensorRT 人体检测 | ✅ 已实现 | bus.jpg 检出 4 人；实机 30 FPS；推理 8–20 ms |
| P5 | 2026-11-21 | 手部 21 点规则手势 | ✅ 已跑通（社区权重，仅验证） | 规则用例通过；手部 engine 检测 score 0.899，推理 3.8–9.2 ms；实时 P4+P5 26–30 FPS；端到端手势事件待人在画面中确认 |
| P6 | 2026-11-21 | 目标锁定、单目测距、云台视觉跟随（默认关闭，手势边沿开启） | ✅ 已实现 | 闭环仿真 0.52 s 收敛，残差 0.44°/0.01° |
| P7 | 2026-11-21 | 手势跟踪开关、`cmd_vel_mux`、导航接入 | ✅ 已实现 | 张开手→fist 锁定/跟随、fist→张开手 结束；mux 优先级/超时/回落通过 |
| P3–P7 一体化 | 2026-11-22 | 单个 launch 启动取流/人体/手势/跟随/仲裁 | ✅ 已实现 | 四节点同启，CSI 1080p@30、engine 加载、`follow=off`、mux 正常 |
| 可视化 | 2026-11-21 | 按需查看视频、跟踪、手势、锁定、云台 HUD | ✅ 已实现 | 叠加图像与 RViz 配置验证通过 |

---

## 1. 实现记录（按阶段）

### P3 — CSI 取流 / 标定 / 静态 TF（2026-11-21）

- 新建 `roborts_perception`，包类型从 `ament_python` 改为 `ament_cmake`，
  节点用 **C++** 实现（`rclcpp` + GStreamer C API + OpenCV）。
- `src/csi_camera.cpp`：进程内 `nvarguscamerasrc → nvvidconv → BGRx → appsink`
  （`max-buffers=1 drop=true`），采集线程与处理线程解耦，只保留最新帧。
- `src/camera_model.cpp`：读取 `camera_info` 格式 YAML，像素→方位角（含去畸变）。
- `scripts/calibrate_csi.py`：棋盘格标定工具（离线 Python），输出 RMS/逐视图误差。
- `roborts_bringup/launch/static_tf.launch.py` 新增：
  `gimbal → csi_camera_link → csi_camera_optical_frame`（安装偏移为 launch 参数）。
- 调试话题（默认关闭）：`/perception/debug/image_raw`、`/perception/debug/camera_info`。

### P4 — 人体检测（2026-11-21）

- `src/trt_engine.cpp`：TensorRT 10 C++ API 封装（反序列化、显存、`enqueueV3`）。
- `src/pose_detector.cpp`：letterbox、YOLO11-pose 解码、NMS、关键点还原，全部 C++ 自实现。
- `src/person_tracker.cpp`：IoU 贪心跟踪，输出稳定 `id`。
- `src/pose_smoke_test.cpp`：静态图 engine 验证工具。
- `scripts/build_models.py`：官方 `yolo11n-pose.pt` → ONNX → `trtexec --fp16`。
- 发布 `/perception/persons`（`PersonArray`），字段：`id`、bbox、`score`、
  `distance`（假设身高 1.7 m）、`yaw/pitch`（弧度，yaw 右正、pitch 下正）。
- 已在本机构建 `models/yolo11n-pose_fp16.engine`（输入 `images` 1×3×640×640，
  输出 `output0` 1×56×8400，FP32 I/O，FP16 网络）。
- engine 缺失/反序列化失败时打印重建命令并关闭 P4 分支，**不回退到 PyTorch**。

### P5 — 手势识别（2026-11-21）

- `src/gesture_classifier.cpp`：基于 21 个手部关键点（MediaPipe 顺序）的规则分类：
  `fist / palm / victory / pointing_up / thumbs_up`。
- 由双腕关键点生成手部 ROI，运行手部 pose engine；连续 5 帧、score ≥0.6 确认，
  边沿触发，0.8 s 冷却，手势保持期间持续发布。
- 发布 `/perception/gestures`（`GestureArray`），`enable_gesture` 默认 `false`。
- **未随仓库提供手部权重**（社区权重许可证/可信度需自行确认）；缺失时仅关闭手势分支，
  P3/P4 正常运行。构建方式：
  `ros2 run roborts_perception build_models --hand-weights /path/to/best.pt`。
- 2026-11-21 验证用模型：`chrismuntean/YOLO11n-pose-hands` 的 `best.pt`
  （sha256 `e60bb8d9…c4af3`，**仓库未声明许可证**，作者注明对张开手效果较好、
  fist/thumbs_up 等非张开手势有限），导出 `imgsz=320`，ONNX 输出
  `(1, 68, 2100)`，构建 `hand_pose_fp16.engine`。仅用于验证，交付前必须替换为
  许可证明确的 checkpoint。
- 实测（2026-11-21）：手部裁剪图 detection score **0.899**，推理 **3.8–9.2 ms**；
  实时 P4+P5 处理 **26–30 FPS**（hand 8–19 ms，含双腕 ROI）。
- 调试增强：debug 图绘制所有手部框与 21 个关键点；腕部 ROI 改为沿肘→腕方向
  前移，尺寸 `0.45 × 人体框高`、上限 800 px（适配近景大手）。
- 2026-11-22 手掌识别放宽：`palm` 只要求 index/middle/ring/pinky 四指伸展
  （拇指可收），并使用 1.10 的伸展比例判定，避免张开手掌因拇指或透视被误判为
  `pointing_up`；合成关键点用例 palm（拇指伸出/收拢/1.15 倍透视）、fist、
  victory、pointing_up、thumbs_up 全部通过。
- 端到端状态：2026-11-21 启动成功但画面内无完整人体/手；2026-11-22 实机已检测到
  人体（id 4）和 `fist`/`pointing_up`，但当时未出现 `palm`，因此没有触发
  `palm→fist`。放宽 palm 规则后需再复测一次。

### P6 — 云台视觉跟随（2026-11-21）

- 新增 `src/gimbal_follow_node.cpp`：
  - 消费 `/perception/persons`，支持 `/perception/lock_target` 与 `/perception/follow_enable`；
  - **跟随默认关闭**，由 `gesture_command_bridge` 在“张开手 → fist”边沿同时下发锁定目标与跟随使能；
  - 绝对角控制：`target = current + gain · sign · bearing`；
  - 增益 0.9、0.8° 死区、180°/s 限速、50 Hz 控制/指令频率、0.5 s 丢失保持、3 s 回中；
  - pitch 软限位；yaw 可配置“禁止底盘跟随”并 clamp；
  - 启动时调用 `/set_gimbal_mode(CODE_CONTROL)`（可关闭）。
- `yaw_sign` / `pitch_sign` 默认 `-1`，需台架确认方向。
- 闭环仿真（模拟云台按 120°/s 响应）：0.52 s 进入 1° 死区，
  稳态残差 yaw 0.44°、pitch 0.01°。
- 2026-11-22 提速：`yaw_gain/pitch_gain` 0.7→0.9、`max_rate_deg_s` 60→180、
  `deadband_deg` 1.0→0.8、`publish_rate` 30→50 Hz，均作为 launch 参数暴露。
  直接闭环阶跃测试（0.5 rad ≈ 28.6°）：**0.40 s 进入 1.15° 窗口**，
  最大超调约 2.2°，随后稳定在 -0.50 rad。

#### 底盘是否跟随（2026-11-22 确认）

- **默认不跟随**：`allow_chassis_follow=false`，yaw 被软件限位在
  `[-90°, +90°]`，目标超出时只打印 warning，底盘保持不动；pitch 只动云台。
- `gimbal_follow_node` **不发布 `/cmd_vel`**，所以底盘不会前后移动去追人；
  底盘前进只来自 `thumbs_up` 手势（且需手势模式）。
- 设置 `allow_chassis_follow:=true` 后：不再 clamp yaw；当目标超过云台相对机身的
  机械行程时，由**下位机固件**驱动底盘跟随转向。注意：
  - 只在云台转到底时跟随，不是连续主动跟随；
  - 只转向，不前进/后退；
  - 跟随速度/加速度由固件决定，上位机不控制。
- 导航中保持 `allow_chassis_follow:=false`，避免底盘转动改变 `base_link`，
  影响 odom、RTAB-Map、costmap 与 TEB。
- 当前**没有**通过 `/cmd_vel_acc` 主动转向跟随的实现；如需要，需要在
  `gimbal_follow_node` 中发布角速度并接入 `cmd_vel_mux`。
- 启动示例：

  ```bash
  # 默认：仅云台上下左右跟随
  ros2 launch roborts_perception perception_control.launch.py

  # 允许云台到行程极限后由固件带动底盘转向
  ros2 launch roborts_perception perception_control.launch.py \
      allow_chassis_follow:=true
  ```

### P7 — 手势指令 / 速度仲裁（2026-11-21）

- 新增 `src/gesture_command_bridge.cpp`：
  - **跟踪开关（按 `track_id` 边沿）**：张开手 → `fist` 锁定该人的 `track_id`
    （`/perception/lock_target`）并开启跟随（`/perception/follow_enable=true`）；
    `fist` → 张开手先关闭跟随、再释放锁定，结束跟踪；
  - “张开手”以 `palm` 为准，同时接受 `pointing_up` / `victory`：实机复测中
    模型把张开的手识别为 `pointing_up`，`palm` 始终为 0，强制要求 `palm`
    会导致开关不可用；
  - 两个手势须属于同一 `track_id`、在 `transition_timeout_s`（默认 1.5 s）内出现，
    中间允许短暂丢失（手势确认/冷却造成的空档）；
  - 该开关只动云台，不需要手势模式；`fist` 不再后退、`palm` 不再急停；
  - `thumbs_up` = 前进 0.15 m/s，仍需通过 `~/enable`（`std_srvs/SetBool`）显式进入手势模式；
  - 速度上限 0.2 m/s；2 s 未见手自动停车；
  - 未进入手势模式时不发布 `/cmd_vel_gesture`，mux 自动回落到规划器。
- 新增 `src/cmd_vel_mux.cpp`：优先级 **安全 > 手势 > 规划器**，
  每路 0.5 s 超时，输出 `/cmd_vel_acc`（`TwistAccel`，本仓库导航栈实际使用该话题）。
- `roborts_vslam/launch/vslam_navigation.launch.py` 新增
  `use_gesture_control:=true`：启动完整的 P3–P7 一体化 launch，并把
  `local_planner` 重映射到 `/cmd_vel_acc_planner`，由 mux 统一输出。
- 新增 `launch/gesture_control.launch.py` 统一启动 P6/P7（P3–P7 一体化使用
  `perception_control.launch.py`，见下文）。

### 可视化（2026-11-21，按需调试）

- 新增 `scripts/perception_view.py`（安装为 `perception_view`）：
  - 订阅 `/perception/debug/image_raw`、`/perception/persons`、`/perception/gestures`、
    `/perception/lock_target`、`/perception/follow_enable`、`/cmd_gimbal_angle`；
  - 叠加：人物 id/距离/方位/俯仰、锁定目标红框、手势框与名称、跟随状态、云台角度、FPS；
  - 发布 `/perception/debug/overlay`（RViz / `rqt_image_view`），
    可选 `show_window:=true` 打开本地 OpenCV 窗口。
- 新增 `launch/perception_view.launch.py`：按需启动相机 + 感知 + viewer，
  默认附带 RViz（`config/perception.rviz`），也可 `use_rviz:=false show_window:=true`。
- 已在 1080p 实机流与 1280×720 合成流上验证 HUD/锁定/手势/距离标签渲染正确。
- 注意：viewer 仅用于调试，启动它会打开相机；不要与常规感知节点同时运行。

启动方式（2026-11-21 22:10 实机验证）：

```bash
# 默认：相机 + 感知 + viewer + RViz（Image 面板显示 /perception/debug/overlay）
ros2 launch roborts_perception perception_view.launch.py

# 本机 OpenCV 窗口（需要 DISPLAY）
ros2 launch roborts_perception perception_view.launch.py \
    use_rviz:=false show_window:=true

# 或任意时刻用 rqt 查看叠加流
ros2 run rqt_image_view rqt_image_view /perception/debug/overlay
```

显示内容：实时视频；人物绿框 + `#id 距离 yaw pitch` + 关键点；锁定目标红色粗框
`LOCK`；手势青色框 + 名称/置信度/跟踪 id；HUD 中的 FPS、人数、锁定 id、
`follow: ON/OFF`、云台 yaw/pitch、当前手势。`enable_gesture:=true` 且手部
engine 存在时才会出现手势数据。

实测说明：1080p 下 viewer 绘制/发布约 5–8 FPS（Python 调试节点，不在性能路径）；
overlay 输出使用 **RELIABLE depth 1**，否则 RViz 的 RELIABLE 订阅会因 QoS 不兼容
收不到图像（已修复并确认无 “incompatible QoS” 日志）。

### 一体化启动（P3–P7，2026-11-22）

**新增 launch**：`roborts_perception/launch/perception_control.launch.py`。
一次启动视觉侧完整控制链路：

| 节点 | 阶段 | 作用 |
|---|---|---|
| `person_gesture_node` | P3/P4/P5 | CSI 取流、人体检测、手势识别 |
| `gimbal_follow_node` | P6 | 云台视觉跟随 |
| `gesture_command_bridge` | P7 | 张开手↔`fist` 锁定跟踪、`thumbs_up` 前进 |
| `cmd_vel_mux` | P7 | 安全 > 手势 > 规划器 → `/cmd_vel_acc` |

默认参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `start_base` | `true` | 一并启动 `roborts_base`（已有 bringup 启动底盘时设为 `false`） |
| `enable_person` | `true` | P4 人体检测 |
| `enable_gesture` | `true` | P5 手势；P6/P7 跟踪开关依赖它 |
| `follow_enabled` | `false` | 启动时不跟随，等待“张开手→fist” |
| `gesture_enabled` | `false` | `thumbs_up` 前进需显式开启 |
| `use_mux` | `true` | 输出 `/cmd_vel_acc` |
| `use_viewer` | `false` | 开启调试图像 + `perception_view` |
| `use_rviz` | `true` | 仅在 `use_viewer:=true` 时生效 |
| `publish_csi_tf` | `false` | 主 bringup 已发布时保持 `false` |

启动命令：

```bash
# 基本运行：相机 + 人体 + 手势 + 云台跟随 + 速度仲裁
ros2 launch roborts_perception perception_control.launch.py

# 带感知 HUD / RViz（自动打开 publish_debug_image）
ros2 launch roborts_perception perception_control.launch.py use_viewer:=true

# 无 RViz，使用本地 OpenCV 窗口（需要 DISPLAY）
ros2 launch roborts_perception perception_control.launch.py \
    use_viewer:=true use_rviz:=false show_window:=true

# 允许 thumbs_up 前进运动
ros2 launch roborts_perception perception_control.launch.py gesture_enabled:=true

# 独立运行且主 bringup 未发布 CSI TF 时
ros2 launch roborts_perception perception_control.launch.py publish_csi_tf:=true
```

前置条件：

- 默认 `start_base:=true`，本 launch 会一并启动 `roborts_base`：它发布
  `base_link → gimbal`，提供 `/set_gimbal_mode`，并消费
  `/cmd_gimbal_angle` 与 `/cmd_vel_acc`。
  如果已有 bringup 启动底盘，必须传 `start_base:=false`，避免串口被打开两次。
- 主 bringup（如 `roborts.launch.py` / `vslam.launch.py`）已经发布
  `gimbal → csi_camera_link → csi_camera_optical_frame`；
  只有没有主 bringup 的独立运行才设 `publish_csi_tf:=true`，不要同时发布。
- `cmd_vel_mux` 会写 `/cmd_vel_acc`；同时运行规划器时必须让 planner 发布
  `/cmd_vel_acc_planner`，否则两条控制源会争用同一话题。

导航集成：

- `vslam_navigation.launch.py use_gesture_control:=true` 现在直接包含
  `perception_control.launch.py`（传 `start_base:=false`，因为导航 launch
  自己启动 `roborts_base`），同时仍把 `local_planner` 重映射到
  `/cmd_vel_acc_planner`。因此导航 + 手势跟踪只需一个顶层 launch：

  ```bash
  ros2 launch roborts_vslam vslam_navigation.launch.py use_gesture_control:=true
  ```

- 原有的 `person_gesture.launch.py`（P3–P5）和
  `gesture_control.launch.py`（P6/P7）保留，便于分阶段调试。

验证（2026-11-22，本机）：

| 项 | 结果 |
|---|---|
| 构建 | `colcon build --packages-select roborts_perception roborts_vslam` 通过；`roborts_msgs` + `roborts_base` 已重建以清除旧 `GimbalLimit` 符号 |
| 一体化启动 | 5 节点同时启动（含 `roborts_base`），CSI 1080p@30，person/hand engine 加载，`follow=off`，跟踪待机，mux 正常 |
| 云台链路 | `roborts_base` 连接固件 3.0.0.1；`/set_gimbal_mode` 被调用；`base_link → gimbal` TF 为 `(0, 0, 0.15)` |
| 手掌识别 | 放宽后合成用例 7/7 通过（palm 拇指伸出/收拢/1.15 倍透视、fist、victory、pointing_up、thumbs_up） |
| `use_viewer:=true` | 自动 `debug_image=true`，`perception_view` 启动 |
| `publish_csi_tf:=true` | `gimbal → csi_camera_link`、`csi_camera_link → optical` 两个广播器启动 |
| 跟踪边沿 | 张开手→fist 锁定/跟随、fist→张开手 释放/停止，合成手势测试 PASS（含 `pointing_up→fist`） |
| 直接闭环 | 合成目标方位 0.2 rad：`pointing_up→fist` → `lock=42`/`follow=true`；云台下发 10 条角度指令，指令收敛到 -0.200 rad，实际 `base_link→gimbal` yaw 从 +0.007 变到 -0.2007 rad（约 -12°）；`fist→pointing_up` → `follow=false`/`lock=0`；RESULT PASS |
| 云台门控 | 未使能跟随时 0 条指令；锁定+使能后开始下发；关闭后停止 |
| 实机手势 | 2026-11-22 检测到人（id 4）和 `fist`/`pointing_up`，未出现 `palm`；放宽规则后待复测 |
| 实机手势（最终） | 张开手被识别为 `palm` → `OPEN -> FIST: locked person 10`，`follow=true`，`/cmd_gimbal_angle` 持续下发且云台 yaw 实际变化；再次张开手 → `FIST -> OPEN`，`follow=false`，跟踪停止 |

### 消息与接口（2026-11-21）

- 新增消息：`Person.msg`、`PersonArray.msg`、`Gesture.msg`、`GestureArray.msg`。
- 新增可执行：`person_gesture_node`、`gimbal_follow_node`、`gesture_command_bridge`、
  `cmd_vel_mux`、`pose_smoke_test`、`perception_view`。
- 新增 launch：`perception_control.launch.py`（P3–P7 一体化）、
  `person_gesture.launch.py`（P3–P5）、`gesture_control.launch.py`（P6/P7）、
  `perception_view.launch.py`（HUD）。
- 离线工具：`calibrate_csi`、`build_models`。

---

## 2. 话题与参数

| 话题 | 类型 | 生产者 | 阶段 |
|---|---|---|---|
| `/perception/persons` | `roborts_msgs/PersonArray` | `person_gesture_node` | P4 |
| `/perception/gestures` | `roborts_msgs/GestureArray` | `person_gesture_node` | P5 |
| `/perception/lock_target` | `std_msgs/UInt32` | `gesture_command_bridge` | P7 |
| `/perception/follow_enable` | `std_msgs/Bool` | `gesture_command_bridge` | P7 |
| `/cmd_gimbal_angle` | `roborts_msgs/GimbalAngle` | `gimbal_follow_node` | P6 |
| `/cmd_vel_gesture` | `geometry_msgs/Twist` | `gesture_command_bridge` | P7 |
| `/cmd_vel_acc` | `roborts_msgs/TwistAccel` | `cmd_vel_mux` | P7 |
| `/perception/debug/image_raw`¹ | `sensor_msgs/Image` | `person_gesture_node` | P3 |
| `/perception/debug/camera_info`¹ | `sensor_msgs/CameraInfo` | `person_gesture_node` | P3 |
| `/perception/debug/overlay`¹ | `sensor_msgs/Image` | `perception_view` | 可视化 |

¹ 仅 `publish_debug_image:=true` 时发布。

主要参数：`enable_person`、`enable_gesture`、`person_engine_path`、
`hand_engine_path`、`person_conf_threshold`、`person_nms_iou`、
`min_person_height_px`、`person_height_m`、`hand_roi_scale`、
`gesture_min_frames`、`gesture_min_score`、`gesture_cooldown_s`、
`transition_timeout_s`、`yaw_gain`、`pitch_gain`、`yaw_sign`、`pitch_sign`、
`deadband_deg`、`max_rate_deg_s`、`publish_rate`、`allow_chassis_follow`、`yaw/pitch 限位`。

---

## 3. 实测数据（2026-11-21）

| 项 | 结果 |
|---|---|
| CSI 1080p@30 | driver 30.0–30.1 FPS，processed 30.0 FPS，0 跳帧 |
| P4 推理（含 H2D/D2H） | 8.0 ms best，12–20 ms 典型，首帧 ~96 ms |
| P4 端到端 | 30 FPS；房间内无目标时为 0 人，不阻塞取流 |
| P4 静态验证 | `bus.jpg` 检出 4 人，score 0.88/0.88/0.88/0.36，框与关键点正确 |
| 节点资源（P4） | RSS ≈ 359 MB，约 1 个 CPU 核 |
| P5 规则分类器 | fist/palm/victory/pointing_up/thumbs_up 5/5 通过（合成关键点） |
| P5 手部 engine | 320 输入、21 关键点；实拍手部裁剪图 detection **0.899**，推理 **3.8–9.2 ms** |
| P5 实时链路 | `enable_gesture:=true` 启动成功，hand detector 加载；P4+P5 处理 26–30 FPS |
| P5 端到端手势 | 待人在 1.5–3 m 内展示手势后确认（当时画面内无完整人体/手） |
| P5 缺失 engine | 明确 ERROR，手势分支关闭，P3/P4 继续 30 FPS |
| P6 闭环仿真 | 0.52 s 收敛，残差 0.44°/0.01°（目标 <3°，无振荡） |
| P7 优先级 | 手势 0.15 覆盖 planner 0.05 |
| P7 超时回落 | 关闭手势 0.5 s 后 mux 回落到 planner 0.05 |
| P7 跟踪边沿 | 张开手→fist → `lock_target=<id>` + `follow_enable=true`；fist→张开手 → `follow_enable=false` + `lock_target=0`（张开手接受 palm/victory/pointing_up） |
| P7 事件 | `thumbs_up` → 前进 0.15；`victory`/`pointing_up` 不再参与跟踪开关 |
| TensorRT engine 构建 | 673.98 s（约 11 min，Orin Nano） |

---

## 4. 与设计文档的差异

1. **运行时语言**：节点由设计中的 Python 改为 C++，TensorRT 使用 C++ API
   （`libnvinfer` + CUDA runtime），避免 1080p 取流/推理链路上的 GIL 开销。
2. **P5 模型**：手部权重不随仓库提供。验证时使用社区
   `chrismuntean/YOLO11n-pose-hands` 权重（仓库无许可证声明，作者注明非张开手势
   效果有限），仅用于验证；交付前必须替换为许可证明确、可信的 checkpoint。
3. **P7 速度话题**：本仓库导航栈由 `local_planner` 直接发布 `/cmd_vel_acc`，
   因此 mux 仲裁 `TwistAccel` 并输出 `/cmd_vel_acc`；导航 launch 通过
   `use_gesture_control` 将 planner 重映射到 `/cmd_vel_acc_planner`。
4. **相机内参**：`config/camera_csi.yaml` 仍为标称值（`calibrated: false`），
   节点启动会告警；P4/P6 的方位角在真实标定前仅具参考意义。
5. **标定工具**：保留 Python（离线使用，不影响运行性能）。

---

## 5. 上车前待办

1. 真实棋盘格标定（1080p），RMS < 0.5 px，覆盖 `config/camera_csi.yaml`。
2. P4 在 3/5/8 m、实际光照下评估召回率（≥90%）；不足时按设计文档微调。
3. P5 当前用社区 `chrismuntean/YOLO11n-pose-hands` 权重（**无许可证声明**）跑通；
   交付前必须替换为许可证明确的 checkpoint，并验证 5 类手势准确率。
4. P6 台架确认 `yaw_sign`/`pitch_sign`、yaw 相对机身行程、pitch 上下限；
   导航中保持 `allow_chassis_follow:=false`。如需底盘跟随，另需确认
   `allow_chassis_follow:=true` 时固件在 yaw 到限位后的跟随速度与幅度。
5. P7 上车验证：张开手→fist 在 1.5–3 m 内可靠触发锁定与跟随、fist→张开手
   可靠结束；底盘对 `TwistAccel.accel` 的响应、超时停车，
   以及 `use_gesture_control:=true` 时导航与手势接管的切换。
6. 在另一台 Jetson 上需按相同 TensorRT 版本重新构建 engine。

---

## 6. 构建与运行

```bash
colcon build --packages-select roborts_msgs roborts_perception roborts_vslam \
    --symlink-install

# 一体化 P3–P7（相机 + 人体 + 手势 + 云台跟随 + 速度仲裁）
ros2 launch roborts_perception perception_control.launch.py
# 带 HUD/RViz（自动打开调试图像）
ros2 launch roborts_perception perception_control.launch.py use_viewer:=true
# 允许 thumbs_up 前进运动
ros2 launch roborts_perception perception_control.launch.py gesture_enabled:=true

# 单独调试：
# 取流 + P4（+ 可选 P5）
ros2 launch roborts_perception person_gesture.launch.py
ros2 launch roborts_perception person_gesture.launch.py publish_debug_image:=true
ros2 launch roborts_perception person_gesture.launch.py enable_gesture:=true

# P6/P7
ros2 launch roborts_perception gesture_control.launch.py
# 可选：允许 thumbs_up 前进运动（跟踪开关不需要）
ros2 service call /gesture_command_bridge/enable std_srvs/srv/SetBool "{data: true}"

# 导航 + 手势控制（自动包含 P3–P7；planner 经 mux）
ros2 launch roborts_vslam vslam_navigation.launch.py use_gesture_control:=true

# 模型
ros2 run roborts_perception build_models --pose
ros2 run roborts_perception build_models --hand-weights /path/to/best.pt
ros2 run roborts_perception pose_smoke_test IMAGE ENGINE 17 20

# 按需可视化（会打开相机）
ros2 launch roborts_perception perception_view.launch.py
ros2 launch roborts_perception perception_view.launch.py \
    use_rviz:=false show_window:=true
ros2 run rqt_image_view rqt_image_view /perception/debug/overlay
```

---

## 7. 环境与模型说明

- TensorRT：10.3，`trtexec` 位于 `/usr/src/tensorrt/bin/trtexec`。
- engine 与 GPU / TensorRT 版本绑定，必须在目标 Jetson 上生成。
- `roborts_perception/models/` 下 `*.pt/*.onnx/*.engine` 已加入 `.gitignore`，
  不随仓库提交；`models/README.md` 说明构建步骤。
- 离线 ultralytics 导出需固定 Python 环境：
  `numpy==1.26.4`、`onnx==1.16.2`、`ml_dtypes==0.4.0`
  （最新 onnx 会拉入 numpy 2.x，破坏 ROS/cv_bridge）。

---

## 变更记录

| 日期 | 内容 |
|---|---|
| 2026-11-21 | P3：`roborts_perception`（C++ 取流）、标定工具、CSI 静态 TF |
| 2026-11-21 | P4：TensorRT 人体检测、`Person/PersonArray`、engine 构建与验证 |
| 2026-11-21 | P5：手部 21 点规则手势、`Gesture/GestureArray`、去抖与冷却 |
| 2026-11-21 | P6：`gimbal_follow_node`、锁定/测距/跟随、死区与限位 |
| 2026-11-21 | P7：`gesture_command_bridge`、`cmd_vel_mux`、导航 `use_gesture_control` 接入 |
| 2026-11-21 | 可视化：`perception_view`（HUD/锁定/手势/云台）、`perception_view.launch.py`、`perception.rviz`；overlay 输出改为 RELIABLE 以兼容 RViz，实机启动验证通过 |
| 2026-11-21 | P5 模型：社区 `chrismuntean/YOLO11n-pose-hands` → ONNX 320 `(1,68,2100)` → `hand_pose_fp16.engine`；手部裁剪检测 score 0.899，推理 3.8–9.2 ms |
| 2026-11-21 | P5 调试：debug 图叠加手部框/21 关键点；腕部 ROI 改为肘–腕方向前置、0.45×人体框高、最大 800 px |
| 2026-11-21 | P5 实机：`enable_gesture:=true` 启动成功、hand detector 加载；画面内暂无完整人体/手，手势事件待确认 |
| 2026-11-22 | P6/P7 交互调整：跟随默认关闭；`palm → fist` 锁定并开始云台跟踪，`fist → palm` 释放并结束跟踪（按 `track_id` 边沿、1.5 s 容差）；`fist`/`palm` 不再承担后退/急停；手势冷却 1.5 → 0.8 s |
| 2026-11-22 | 一体化启动：新增 `perception_control.launch.py`，一次启动 P3–P7；`vslam_navigation.launch.py use_gesture_control:=true` 改为包含该 launch；可选 `use_viewer`、`publish_csi_tf` |
| 2026-11-22 | 实机排查“云台不跟踪”：① palm 未识别 → 放宽四指/1.10 比例规则；② `roborts_base` 未运行且旧二进制引用已删除的 `GimbalLimit` → 重建 `roborts_msgs`/`roborts_base`；`perception_control` 新增 `start_base`（默认 true），导航包含时传 `false` |
| 2026-11-22 | 复测发现张开手始终被识别为 `pointing_up`、`palm` 为 0：跟踪开关改为“张开手（`palm`/`victory`/`pointing_up`）↔ `fist`”，并增加“open hand seen”提示日志；合成测试新增 `pointing_up→fist` 用例并通过 |
| 2026-11-22 | 直接闭环测试通过：合成方位 0.2 rad，`pointing_up→fist` 锁定 id 42 并跟随，云台实际 yaw 从 +0.007 转到 -0.2007 rad（约 -12°），`fist→pointing_up` 正确停止 |
| 2026-11-22 | 实机手势闭环通过：张开手（`palm`）→ `fist` 锁定 person 10，`follow=true`，云台跟随；`fist` → 张开手触发 `FIST -> OPEN`，`follow=false`，跟踪停止 |
| 2026-11-22 | 跟随提速：`yaw_gain/pitch_gain` 0.9、`deadband_deg` 0.8、`max_rate_deg_s` 180、`publish_rate` 50；0.5 rad 阶跃 **0.40 s** 进入 1.15° 窗口，超调约 2.2°；参数已加入 launch |
| 2026-11-22 | 确认底盘跟随：默认 `allow_chassis_follow=false`，仅云台上下左右跟随、底盘不动；设为 `true` 时仅在云台到机械行程后由固件带动底盘转向（不前进/后退）；导航中保持 false |
