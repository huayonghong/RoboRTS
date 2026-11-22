# 云台 CSI 相机感知设计文档

> **范围**：云台顶部 CSI 相机（IMX219/CAM0）+ XVF3800 四麦阵列的语音检测/声源定位、人体检测、手势识别与云台跟随。
>
> **节点划分**：语音检测（VAD）+ 声源定位（DOA）合为一个节点 `voice_perception_node`；
> CSI 取流 + 人体检测 + 手势识别合为一个节点 `person_gesture_node`（图像不经 ROS 话题中转）；
> 云台指向仍为独立节点 `gimbal_aim_node`。
>
> **当前实施优先级**：视觉侧 P3–P7 已实现（P5 手部权重需自备）；XVF3800 存在硬件故障，
> 语音侧 P1/P2 待硬件恢复后推进。
>
> **代码基线**：`ros2` 分支，`8d81f3f` 之后。
> **环境实测日期**：2026-11-15。
> **状态**：设计定稿；P3–P7 已实现（2026-11-21）。

---

## 0. 结论速览

| 项 | 结论 |
|---|---|
| 声源定位硬件 | ReSpeaker XVF3800（4 麦），装在**云台顶部**，随云台 yaw 一起转动 |
| DOA 读取方式 | pyusb 读 `DOA_VALUE`（resid 20 / cmdid 18），或 `xvf_host AEC_AZIMUTH_VALUES` |
| 语音节点 | `voice_perception_node` 单节点完成 VAD（语音检测）+ DOA（声源定位）+ 滤波 + 事件发布 |
| 视觉节点 | `person_gesture_node` 单节点完成 CSI 取流（P3）+ 人体检测（P4）+ 手势识别（P5），共享同一帧与预处理 |
| 云台指向节点 | `gimbal_aim_node` 独立于语音感知节点，只消费 `/voice/doa`，可单独禁用 |
| 人体发现方式 | **不盲目扫云台**：语音 DOA 粗定向 → 云台转到声源方位 → 视觉确认 |
| 运行时推理 | **只用 TensorRT**：FP16 engine + TensorRT C++ API（`libnvinfer` + CUDA runtime）；PyTorch/ultralytics 仅用于离线导出与精度评估，不进入运行时 |
| 人体检测模型 | 官方预训练 `yolo11n-pose`，离线导出 ONNX → TRT engine，先用泛化能力；实测失败再微调 |
| 手势方案 | 后续阶段：手部 21 关键点（同样导出 TRT engine）+ 规则分类；当前不训练 |
| CSI 取流 | 在 `person_gesture_node` 内用 C++ GStreamer appsink（本机 `cv2` 无 GStreamer），`nvarguscamerasrc`；**不发布 ROS 图像话题**，调试时可选开启 |
| 云台控制 | 只用 `cmd_gimbal_angle`（`GimbalRate` 在 `roborts_base` 中无订阅者） |
| 云台限位 | **yaw** 越限后固件驱动**底盘跟随**（全向可达，但机器人整体转向）；**pitch** 越限后云台**停止**（不可达）；SDK 未上报限位值，需软限位参数或扩展协议 |
| 当前前置条件 | XVF3800 **尚未插入**；pyusb 已就绪；udev 规则只覆盖 PID `001e`，`001a` 需确认/补充 |

---

## 1. 目标与范围

### 1.1 总目标

1. 听到人说话后，云台自动转向声源方位。
2. 视场内检测并锁定人体，云台跟随人体（或手部）。
3. 识别手势，转换为机器人指令。
4. 全链路与现有 D455 VSLAM / RPLidar 导航共存。

### 1.2 当前阶段（P1/P2）

- 打通 XVF3800 的 DOA/VAD 数据；
- 云台按声源方位自动指向；
- 限位、失锁、噪声等安全保护。

> 其余模块（CSI 相机、人体检测、手势、底盘指令）在本文档中给出完整设计，
> 但**不作为当前编码范围**，按第 7 节顺序推进。

### 1.3 非目标

- 不做声纹识别、不做语音内容识别（ASR）；本文所说的“语音识别”仅指语音活动检测（VAD）。
- 不修改现有 RTAB-Map、RPLidar、costmap、规划器逻辑。
- 不替换 D455 的 VSLAM 职责。

---

## 2. 硬件与环境基线

### 2.1 计算平台与软件（实测）

| 项 | 值 |
|---|---|
| 板卡 | Jetson Orin Nano Super（`3767-300-0005`），MAXN_SUPER |
| L4T | R36.4.3（JetPack 6.2） |
| OS / ROS | Ubuntu 22.04 / ROS2 Humble |
| CUDA / TensorRT | CUDA 12.5 / TensorRT 10.3 |
| Python | 3.10.12；运行时依赖 `tensorrt`、`cuda-python`、`cv2 4.13.0` |
| 离线工具链 | `torch 2.5.0`（NVIDIA 版）、`ultralytics 8.4.51` **仅用于模型导出与精度评估**，不进入运行时 |
| 内存 / CPU | 7.6 GB 统一内存 / 6 核 |
| TensorRT 工具 | `trtexec`、TensorRT Python API、`cuda-python` 可用；无 DeepStream |
| 下位机 | STM32 VCP → `/dev/ttyACM0` |

### 2.2 传感器拓扑

| 设备 | 接口 | 设备节点 | 用途 |
|---|---|---|---|
| IMX219 CSI 相机 | CAM0 | `/dev/video0`（RG10 Bayer） | 云台视觉（人体/手势） |
| RealSense D455 | USB 3.0 | `/dev/video1–6`、`/dev/media1–2` | 现有 VSLAM |
| RPLidar | 串口 | `/dev/rplidar` | 现有 costmap |
| XVF3800 | USB | VID `0x2886` / PID `0x001A` / interface 3 | 声源定位（**当前未连接**） |

- CSI 相机与 D455 是两条独立通路，互不占用。
- **不能**用 `roborts_camera` 的 UVC 路径读 CSI：该路径是 `cv::VideoCapture` + UVC，
  且本机 Python `cv2` **无 GStreamer 支持**；`/dev/video0` 只暴露 RG10 原始 Bayer，需 Argus 做 ISP。

### 2.3 实测性能记录

| 测试 | 条件 | 结果 |
|---|---|---|
| CSI 取流 | GStreamer appsink，1280×720@30 | **30.3 FPS**，稳定 |
| `yolo11n-pose` | **TensorRT FP16 engine**，640 | P4 实测回填（设计目标 10–15 ms） |
| `yolo11n-pose`（选型基线） | PyTorch FP16，640（**仅离线对比，不用于运行时**） | **27.5 FPS / 36.4 ms** |
| 当前内存占用 | 空闲 | 2.4 GB / 7.6 GB |

> 运行时统一使用 TensorRT FP16 engine，**不存在 PyTorch 推理路径**；
> PyTorch 数据只是选型阶段（说明为何不用 PyTorch）的离线基线。
> TRT 实测性能在 P4 回填本节。

---

## 3. 总体架构

### 3.1 数据流

```
┌─────────────────────────────── 云台顶部 ───────────────────────────────┐
│  XVF3800 4-mic                                     IMX219 CSI (CAM0)  │
│      │ USB control (pyusb)                              │ GStreamer   │
└──────┼──────────────────────────────────────────────────┼──────────────┘
       ▼                                                  ▼
 voice_perception_node                        person_gesture_node（单节点）
   (语音检测 VAD + 声源定位 DOA)                 ├─ CSI 取流（进程内，无 ROS 图像话题）
   /voice/doa                                    ├─ P4: yolo11n-pose 人体检测
   /voice/speech_event                           ├─ P5: 手部 21 关键点 + 规则手势
       │                                         └─ /perception/{persons,gestures}
       ▼
 gimbal_aim_node
   (云台指向，独立节点)
       │ /cmd_gimbal_angle
       ▼
 roborts_base (现有)  ──► 下位机
       │
       └─ TF: base_link → gimbal（已有动态 TF）

                        ┌─────────── 后续阶段 ───────────┐
                        │ gimbal_follow_node             │
                        │ gesture_command_bridge         │
                        │ cmd_vel_mux → /cmd_vel         │
                        └────────────────────────────────┘
```

#### 3.1.1 节点划分

| 节点 | 包 | 职责 | 阶段 |
|---|---|---|---|
| `voice_perception_node` | `roborts_voice` | 语音活动检测（VAD）、语音事件、DOA 读取与滤波，发布 `/voice/*` | P1 |
| `gimbal_aim_node` | `roborts_voice` | 云台指向控制、状态机、限位与安全 | P2 |
| `person_gesture_node` | `roborts_perception` | CSI 取流（节点内）+ 人体检测（P4）+ 手势识别（P5），发布 `/perception/*` | P3/P4/P5 |

- 语音侧按**“感知 / 控制”**拆分：检测与定位在同一个节点，云台动作在另一个节点，
  便于单独旁路自动指向而不影响 DOA 发布。
- 视觉侧按**“取流—检测—手势同一进程”**合并：CSI 帧从 appsink 直接进入推理链，
  手势 ROI 由同一帧的人体关键点派生，避免 1080p 图像经 DDS 中转（见 4.2）。
- 不再有独立的 `csi_camera_node`；需要看图/标定时用 `publish_debug_image` 参数临时开启调试话题。
- 视觉 P3–P7 作为**一个控制整体**启动：
  `roborts_perception/launch/perception_control.launch.py` 同时启动
  `person_gesture_node`（P3/P4/P5）、`gimbal_follow_node`（P6）、
  `gesture_command_bridge` 与 `cmd_vel_mux`（P7）；
  `vslam_navigation.launch.py use_gesture_control:=true` 直接包含该 launch，
  并把 `local_planner` 重映射到 `/cmd_vel_acc_planner`。
  单个 launch（`person_gesture.launch.py`、`gesture_control.launch.py`）保留用于分阶段调试。

### 3.2 坐标系与 TF

```
base_link
 ├─ base_laser_link              (已有，静态)
 ├─ camera_link                 (已有，D455)
 └─ gimbal                       (已有，动态；roborts_base 发布)
      └─ csi_camera_link         (新增，静态：相机挂载偏移)
           └─ csi_camera_optical_frame   (新增，静态：REP-103 旋转)
```

- `base_link → gimbal`：`roborts_base/gimbal/gimbal.cpp` 依据 `cmd_gimbal_info`
  的 pitch/yaw 编码角发布，平移固定为 `(0, 0, 0.15)`。
- 底盘跟随时云台相对机身的 yaw 保持在行程内，`base_link → gimbal` 不会越界；
  机器人整体转向体现在 `odom → base_link` 上。
- XVF3800 随云台转动，DOA 参考系与 `gimbal` 固连，零位偏置需标定。

---

## 4. 模块设计

### 4.1 XVF3800 语音感知节点（P1 核心：语音检测 + 声源定位）

**包**：新建 `roborts_voice`（`ament_python`）
**可执行**：`voice_perception_node`

**职责**：单节点完成 XVF3800 的语音活动检测（VAD / 语音事件）与声源方位（DOA）识别，
包括设备读取、环形滤波、事件判定和话题发布。云台指向控制由独立节点 `gimbal_aim_node`
（见 4.1.5）承担，保持感知与控制解耦。

**为何合为一个节点**：

- DOA 与 VAD 来自**同一次** USB 读取：`DOA_VALUE` 一次返回 `[azimuth, vad]`，
  拆成两个节点会重复访问设备，并引入时间戳对齐问题；
- VAD 门控、连续帧确认、云台运动抑制都是 DOA 滤波流程的一部分，同进程共享状态最简单；
- 语音事件与方位使用同一采样节拍和同一时钟，`speech_event` 与 `doa` 天然同源。

#### 4.1.1 设备访问

- 使用 `pyusb`（已安装 1.3.1）直接做 vendor control transfer，不依赖 `xvf_host`。
- 设备枚举：VID `0x2886`，PID `0x001A`，interface 3。
- 现有 udev 规则 `/etc/udev/rules.d/99-respeaker.rules` 内容为
  `SUBSYSTEM=="usb", ATTR{idVendor}=="2886", ATTR{idProduct}=="001e", MODE="0666"`，
  **只覆盖 PID `001e`，不一定匹配 XVF3800 的 `001a`**。插上设备后用 `lsusb` 确认 PID；
  若为 `001a`，需补一条
  `SUBSYSTEM=="usb", ATTR{idVendor}=="2886", ATTR{idProduct}=="001a", MODE="0666", GROUP="plugdev"`。
- 读命令参数：

  | 参数 | 值 | 说明 |
  |---|---|---|
  | `resid` | 20 | DOA 资源 |
  | `cmdid` | 18 | `DOA_VALUE` |
  | `length` | 4 | 返回 2 个 `uint16`：`[azimuth, vad]` |
  | 传输 | `ctrl_transfer(0xC0, 0x80\|cmdid, resid, length)` | vendor IN |

  > 不同固件资源号可能不同（部分版本用 19）。`resid` 必须做成参数，
  > 启动时先用 `VERSION` 读命令验证通信。

- 设备未连接时：节点告警并 1 s 重试，不崩溃、不影响其余 ROS 节点。

#### 4.1.2 数据模型与发布

| 话题 | 类型 | 频率 | 说明 |
|---|---|---|---|
| `/voice/doa` | `roborts_msgs/VoiceDoa` | 10–20 Hz | 原始角、滤波角、VAD、beam 能量 |
| `/voice/speech_event` | `std_msgs/Bool`（或自定义） | 事件 | VAD 上升沿/下降沿 |

`VoiceDoa.msg` 字段建议：

```
std_msgs/Header header
float32 azimuth_raw_deg      # 相对麦克风阵列零位，0–360
float32 azimuth_filtered_deg # 中值+环形滤波后
float32 azimuth_gimbal_deg   # 已扣除阵列安装偏置，相对云台零位
float32 azimuth_base_deg     # 叠加当前云台 yaw，相对底盘
bool    speech               # VAD
float32 beam_energy          # AEC_SPENERGY 或等效
uint8   beam_id              # 选中的 beam（3 = 自动选择束）
```

- `/voice/doa` 与 `/voice/speech_event` 由同一个 `voice_perception_node` 发布；
  两者取自同一次 USB 读取，`header.stamp` 一致，下游无需再做时间对齐。
- 角度统一使用 **度**，内部计算转弧度。
- 角度是多值的，滤波必须用**环形中值/环形平均**，不能直接线性平均。

#### 4.1.3 安装与坐标变换

XVF3800 装在**云台顶部**，随云台 yaw 一起转动：

```
声源相对底盘的方位 = 云台当前 yaw + (DOA − 阵列零位偏置)
```

- 云台 yaw 从 `base_link → gimbal` TF 读取（与 DOA 同时间戳对齐）。
- 该公式得到的是**相对底盘**的方位；若需要世界坐标系方位（如与地图目标比对），
  还需叠加 `odom → base_link` 的 yaw——底盘跟随转向时这一项会变化。
- 数组零位偏置 `mic_yaw_offset_deg`：安装时用已知方位的声源标定。
- **自反馈特性**：云台转向声源时，DOA 会同步变小，因此控制律
  `target_yaw = current_yaw + K · (DOA − offset)`（K<1）天然收敛，
  不会持续追着旧的绝对角跑。
- 俯仰：DOA 只有水平方位，**不提供俯仰**。P1 阶段俯仰保持不变，
  进入视觉阶段后再由人框/头部中心决定。
- 阵列随云台 pitch 倾斜时，DOA 平面会倾斜，±20° 内误差约数度；
  如需高精度，按 `atan(tan(doa) · cos(pitch))` 做一阶补偿（P2 可选）。

#### 4.1.4 VAD 与滤波

1. 仅在 VAD 为真时更新方位角；VAD 为假时保持上一有效值并发布 `speech=false`。
2. 5 点环形中值滤波，抑制混响/多人说话造成的跳变。
3. 电机噪声可能触发 VAD 误判：云台运动期间（角速度 > 5°/s）暂停更新 DOA。
4. 连续 3 帧 VAD 为真才产生 `speech_event`，避免瞬时噪声触发云台。

#### 4.1.5 云台指向控制（独立控制节点）

**包**：`roborts_voice`
**可执行**：`gimbal_aim_node`

- 不并入 `voice_perception_node`：控制节点只消费 `/voice/doa`，
  可单独禁用/重启，不影响语音感知与 DOA 发布。
- 前置：调用 `set_gimbal_mode(CODE_CONTROL)`（枚举值 0）。
  `GYRO_CONTROL`(1) 为增稳模式，可能不响应角度指令。
- 控制律（绝对角模式）：

  ```
  target   = current_yaw + K · (azimuth_gimbal)     K = 0.4–0.6
  target   = clamp(target, yaw_min, yaw_max)        # 仅“禁止底盘跟随”模式
  cmd.yaw  = slew_limit(target, current_yaw, max_rate = 60°/s)
  cmd.pitch = current_pitch                          # 保持不变
  yaw_mode = false (absolute), pitch_mode = false (absolute)
  ```

  > 注意：`GimbalAngle` 的 yaw/pitch 两个模式位独立。P1 采用“绝对模式 +
  > 保持当前俯仰”，避免相对模式语义在固件侧不确定导致意外动作。

- **yaw 越限由底盘跟随补足**：云台相对机身的 yaw 行程有限，到达左右极限后固件会让底盘跟随，
  因此“允许底盘跟随”模式下不对 yaw 做 clamp，0–360° 均可对准；代价是机器人整体转向（见 4.1.6）。
- 底盘跟随期间 DOA 闭环的等效增益会变大（底盘也在转），需相应降低 K、保留死区并限制命令速率。
- 发布频率 30–50 Hz；死区 1°；误差 <1.5° 时不再下发（避免抖动）。
- 超时：连续 0.5 s 无新 DOA 则停止更新目标，保持当前角度。

#### 4.1.6 限位与安全

RoboMaster 机器人由**云台 + 底盘**两部分组成，上位机只负责下发云台角度；
两个轴的越限行为不同：

| 轴 | 到达行程极限后的行为 | 对控制的含义 |
|---|---|---|
| yaw（左右） | 云台转到极限后，**底盘跟随转动**，目标方位仍可对准 | 0–360° 全向可达；超出云台行程时**机器人整体转向** |
| pitch（上下） | 云台**停止**，不会继续运行 | 目标俯仰超出行程即**不可达**，必须 clamp 并上报 |

> 该跟随/停止行为由下位机固件实现，上位机不直接控制跟随速度。
> 当前 `cmd_gimbal_info`（`roborts_base/roborts_sdk/protocol/protocol_define.h`）
> 只包含 `mode / pitch_ecd / yaw_ecd / pitch_gyro / yaw_gyro / yaw_rate / pitch_rate`，
> **不含限位字段**，软限位数值仍需人工配置或由固件组提供。

- 软限位与落地方式：
  - `pitch`：目标角先 clamp 到 `[pitch_min, pitch_max]`；到达限位后不继续加速顶死，
    只上报 warning；视觉阶段人框超出可达俯仰时，云台停在上限，不硬转。
  - `yaw`：按运行模式二选一：
    - **允许底盘跟随**（机器人静止时的语音/视觉指向）：不对 yaw 做 clamp，
      全范围对准，接受底盘随动带来的机体转向；
    - **禁止底盘跟随**（导航/建图/遥控进行中）：命令 yaw 限制在云台相对机身的可转范围内
      （参数 `yaw_min/max`），超出部分标记“不可达”，**不让底盘转向**。
  - 限位数值来源：1) 下位机固件上报 → 扩展 `protocol_define.h` 与 `gimbal.cpp` 后读取；
    2) 暂用参数软限位，MCU 硬限位兜底。
- 底盘跟随的副作用：
  - 自动指向触发底盘转动时，`base_link` 的 yaw 会改变，进而影响 odom、RTAB-Map、
    costmap 与 TEB 的当前位姿；**导航过程中默认关闭自动指向，或只允许云台内行程**。
  - 跟随期间云台相对机身的 yaw 会回到行程内，`base_link → gimbal` TF 保持在限位内，
    机体转向体现在 `odom → base_link` 上。
  - 云台与底盘可能形成“云台转→底盘跟→DOA 变”的耦合，需降低 K、保留死区、限制命令速率，
    避免来回摆动。
- 安全规则：
  - 所有目标角先按当前模式 clamp，再下发；
  - 限位/不可达时上报 warning，不反复顶死；
  - 声源方位超出允许范围（禁止跟随模式）时，报“不可达”，不硬转；
  - 遥控器占用云台时禁止自动指向（需提供仲裁开关）。
- yaw 误差做 ±π 归一化；编码器在限位处可能跳变，优先用 gyro angle 做连续性判断。

#### 4.1.7 状态机

```
IDLE ──speech_event──► AIMING ──到位±3°──► HOLD(1~2s)
  ▲                       │                  │
  │                       └── 新 DOA ──► 更新目标
  └────────超时/VEAD 结束────────────────────┘
```

- `IDLE`：无语音，云台保持。
- `AIMING`：按滤波 DOA 转向；yaw 越限时由底盘跟随补足（禁止跟随模式则标记不可达），
  pitch 受上下限约束。
- `HOLD`：语音结束后保持 1–2 s，避免说话停顿导致云台来回摆。
- 后续接入视觉后，`HOLD` 改为“等待视觉确认”，再进入 `LOCK`。

#### 4.1.8 P1/P2 验收标准

| 指标 | 目标 |
|---|---|
| DOA 读取稳定性 | 连续 30 min 无断连，≥10 Hz |
| 静态指向误差 | 0°/90°/180°/270° 声源，误差 ≤5°（标定后） |
| 动态响应 | VAD 触发 → 云台到位 ≤800 ms，超调 ≤3° |
| yaw 越限 | 底盘平顺跟随，指向可达；导航模式下不触发底盘转向 |
| pitch 越限 | 云台停在上下限，不顶死、不振荡 |
| 静音时 | 云台无动作 |
| 电机噪声 | 云台转动期间不触发 DOA 更新 |

### 4.2 视觉感知节点（P3 取流 / P4 人体 / P5 手势）

**包**：新建 `roborts_perception`（`ament_cmake`，节点用 C++ 实现，避免 Python GIL
在 1080p 取流/推理链路上的开销；棋盘格标定工具为 Python，仅离线使用）
**可执行**：`person_gesture_node`

**职责**：单节点完成 CSI 取流 → 预处理 → 人体检测 → 手势识别，结果发布到 `/perception/*`。
CSI 图像**不经过 ROS 话题中转**，从 GStreamer appsink 直接进入节点内推理链。

**为何把取流放进节点**：

- 原方案由独立的 `csi_camera_node` 发布 `/csi_camera/image_raw`、视觉节点再订阅：
  每帧都要经历 appsink → `cv2` → 序列化 → DDS → 反序列化 → `cv2`，
  1080p BGR 约 6 MB/帧，30 FPS 下约 190 MB/s 的纯拷贝与 DDS 带宽开销；
- 同进程内可直接传递帧缓冲引用，配合 `appsink max-buffers=1 drop=true`
  天然实现“只处理最新帧”，不会因下游慢而积压旧帧；
- 少一个节点、少一组图像话题，也少一次 QoS/带宽配置出错的机会；
- 手势 ROI 由同一帧的人体关键点派生，取流、检测、手势在同一进程内完成。

#### 4.2.1 CSI 取流（P3，节点内）

- 管线（在节点进程内启动，不发布图像话题）：

  ```
  nvarguscamerasrc sensor-id=0 !
  video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1 !
  nvvidconv ! video/x-raw,format=BGRx !
  appsink max-buffers=1 drop=true sync=false
  ```

- 取流线程与推理线程解耦：appsink 回调只做 `cv2.cvtColor(COLOR_BGRA2BGR)`
  （或直接保留 BGRx 视图）并替换“最新帧”引用；推理线程按自身节奏取最新帧，
  处理不过来时跳帧，不排队。
- 调试/标定：参数 `publish_debug_image`（默认 `false`），开启后才发布
  `/perception/debug/image_raw`（`sensor_msgs/Image`，bgr8）与
  `/perception/debug/camera_info`（`best_effort`、`depth=1`），仅供 RViz 和标定使用。
- 1080p 是 16:9 裁切模式，**视场与 720p 不同，必须按 1080p 标定**。
- 标定：OpenCV 棋盘格脚本，输出 `K`、`D` 到 `roborts_perception/config/camera_csi.yaml`，
  节点启动时加载，内部用于像素→方位角投影；验收重投影误差 <0.5 px。
- 静态 TF 加到 `roborts_bringup/launch/static_tf.launch.py`：
  `gimbal → csi_camera_link → csi_camera_optical_frame`。

#### 4.2.2 人体检测（P4，TensorRT 运行 yolo11n-pose）

**策略：官方预训练权重泛化，不训练；运行时只用 TensorRT，不用 PyTorch。**

- 模型：`yolo11n-pose`（17 个 COCO 人体关键点）。
- 离线导出（开发阶段执行一次）：
  `.pt → ONNX（opset 17，imgsz=640，静态 shape）→ trtexec --fp16 --saveEngine=yolo11n-pose_fp16.engine`。
  engine 必须**在目标 Jetson、相同 TensorRT 版本**上生成；`torch`/`ultralytics` 只在这一步使用。
- 运行时推理：
  - 用 TensorRT C++ API（`libnvinfer`）反序列化 engine，CUDA runtime 管理显存与拷贝；
  - 固定输入 `1×3×640×640` FP16，输出按 YOLO11-pose 布局自行解码（框 + 17×3 关键点）；
  - letterbox、归一化、NMS、关键点坐标还原全部 C++ 自实现，**运行时不链接 torch/ultralytics**；
  - `cudaMemcpyAsync` + `enqueueV3` 单流同步执行（实测 8–19 ms/帧，含拷贝）。
- engine 路径参数化（`person_engine_path`）：文件缺失或反序列化失败时**直接报错并打印重建命令**，
  不静默回退到 PyTorch。
- 输出：人体框、17 关键点（含双肩 5/6、双肘 7/8、双腕 9/10）。
- 用途：
  - 人体检测与锁定；
  - 由双肩确定上半身 ROI；
  - 由双腕定位手部 ROI（供 4.2.3 手势分支使用）。
- 过滤：人体框高度 < 阈值（如 40 px @640）或关键点置信度低的目标丢弃。
- **训练兜底条件**：在目标场景（3/5/8 m、实际光照、云台视角）实测，
  人体召回率 <90% 或腕点置信度不足以定位手部时，再采集 500–1000 张
  CSI 实拍帧人工标注，微调 `yolo11n-pose`（PC/云端，50–80 epoch）；
  微调后仍走“ONNX → TRT engine”导出流程，运行时不变。

#### 4.2.3 手势识别（P5 之后）

- 在 `person_gesture_node` 内作为人体检测之后的分支运行，由 `enable_gesture` 参数控制启停。
- 手部 21 关键点：社区预训练权重（`chrismuntean/YOLO11n-pose-hands` 的 `best.pt`，
  或 Kaggle `mumarfiaz/hand-keypoints-detection-v1`）。Ultralytics 官方不提供手部权重，
  使用前确认许可证。
- 运行时与人体检测一致：离线把权重导出为 ONNX → TRT FP16 engine，节点内用 TensorRT 执行；
  两张手部 ROI 可拼成 batch=2 一次推理，同样不依赖 torch/ultralytics。
- 输入：由双腕关键点扩展出的手部 ROI，放大到 256–320。
- 分类：**规则法，不训练**——根据 21 点的相对几何判断手指伸直/弯曲、
  拇指外展、手掌朝向，输出 `fist / palm / victory / pointing_up / thumbs_up`。
  `palm` 只要求四指（食指/中指/无名指/小指）伸展，拇指可收，并使用较宽松的
  伸展比例，避免张开手掌被误判为 `pointing_up`。
- 去抖：同一手势连续 5 帧、置信度 ≥0.6 才确认；边沿触发；冷却 0.8 s
  （较短冷却保证“张开手 → 拳头”跟踪边沿及时出现）。
- 运动模糊：手势判定期间云台减速到 ≤15°/s 或暂停跟随。
- 兜底：规则法在 3/5/8 m 实测准确率 <85% 时，再收集数据训练分类器或手部检测器。

### 4.3 云台视觉跟随（后续）

- 锁定目标后，用像素误差做视觉伺服：

  ```
  yaw_err   = atan((u − cx) / fx)
  pitch_err = atan((v − cy) / fy)
  ```

- 绝对角模式 + PID；死区 0.8°，最大角速度 180°/s，50 Hz 控制频率；
  丢失 0.5 s 保持，3 s 回中。
- 多目标时优先选方位最接近最近一次 DOA 的人。

### 4.4 手势指令与底盘仲裁（P7）

- 跟踪开关（按 `track_id` 边沿触发，同一人的两个手势须在 1.5 s 内出现，
  中间允许短暂丢失）：
  - 张开手 → `fist`：锁定该人 `track_id` 并开启视觉跟随（云台锁定跟踪）；
  - `fist` → 张开手：释放锁定并关闭视觉跟随（结束跟踪）。
  - “张开手”以 `palm` 为准，同时接受 `pointing_up` / `victory`：当前手部模型
    常把松散张开的手识别成这两类，若强制要求 `palm` 会导致开关不可用。
- 运动指令：`thumbs_up` = 前进 0.15 m/s；跟踪开关本身不驱动底盘。
- 跟踪开关只动云台，因此不依赖手势控制模式；运动指令需通过
  `/gesture_command_bridge/enable` 显式进入手势模式后才生效。
- 当前 `cmd_vel` 已有两个发布者（`vel_converter`、`chassis_executor`），
  必须新增 `cmd_vel_mux`，优先级：安全 > 手势 > 规划器，0.5 s 超时归零。
- 本仓库实际导航栈由 `local_planner` 直接发布 `/cmd_vel_acc`（TwistAccel，不是 `/cmd_vel`），
  因此 P7 的 mux 以 TwistAccel 仲裁并输出 `/cmd_vel_acc`；`vslam_navigation.launch.py`
  在 `use_gesture_control:=true` 时把 planner 重映射到 `/cmd_vel_acc_planner`。
- 速度上限 0.2 m/s；2 s 未见手自动停车；需显式进入手势控制模式。

---

## 5. 接口定义

### 5.1 新增消息（`roborts_msgs`）

| 消息 | 字段 |
|---|---|
| `VoiceDoa.msg` | 见 4.1.2 |
| `Person.msg`（P4） | `id, bbox(center,size), score, distance, yaw, pitch` |
| `PersonArray.msg` | `header, Person[]` |
| `Gesture.msg`（P5） | `track_id, gesture_id, name, score, bbox` |
| `GestureArray.msg` | `header, Gesture[]` |

> 不依赖未安装的 `vision_msgs`。

### 5.2 话题汇总

| 话题 | 类型 | 生产者 | 阶段 |
|---|---|---|---|
| `/voice/doa` | `roborts_msgs/VoiceDoa` | `voice_perception_node` | P1 |
| `/voice/speech_event` | `std_msgs/Bool` | `voice_perception_node` | P1 |
| `/cmd_gimbal_angle` | `roborts_msgs/GimbalAngle` | `gimbal_aim_node` | P1 |
| `/perception/persons` | `roborts_msgs/PersonArray` | `person_gesture_node` | P4 |
| `/perception/gestures` | `roborts_msgs/GestureArray` | `person_gesture_node` | P5 |
| `/perception/lock_target` | `std_msgs/UInt32` | `gesture_command_bridge` | P7 |
| `/perception/follow_enable` | `std_msgs/Bool` | `gesture_command_bridge` | P7 |
| `/perception/debug/image_raw`（可选） | `sensor_msgs/Image` | `person_gesture_node` | P3 |
| `/perception/debug/camera_info`（可选） | `sensor_msgs/CameraInfo` | `person_gesture_node` | P3 |
| `/cmd_vel_gesture` → mux | `geometry_msgs/Twist` | `gesture_command_bridge` | P7 |

### 5.3 服务

| 服务 | 提供者 | 用途 |
|---|---|---|
| `/set_gimbal_mode` | `roborts_base`（已有） | 切换到 `CODE_CONTROL` |
| `/voice_aim/enable` | `gimbal_aim_node`（新增） | 自动指向使能/旁路 |

---

## 6. 性能预算

| 环节 | 实测/预估 | 说明 |
|---|---|---|
| XVF3800 DOA 轮询 | <1% CPU | USB control transfer，10–20 Hz |
| 云台指令发布 | 可忽略 | 30–50 Hz 小消息 |
| CSI 1080p30 取流 + 转 BGR（节点内，无 DDS） | 约 1 核 15–20% | 720p 实测 30 FPS，1080p 待复测；无图像话题中转开销 |
| `yolo11n-pose` **TensorRT FP16** @640 | **实测 8–19 ms**（含 H2D/D2H） | 运行时唯一推理路径；含解码/NMS；30 FPS 端到端 |
| 手部 21 点 **TensorRT FP16**（2 ROI，batch=2） | 目标 4–8 ms（待实测） | 仅手势阶段 |
| engine 构建（离线一次性） | **实测 ~11 min**（Orin Nano） | 目标机 + 相同 TRT 版本，不占运行时 |
| 内存（TensorRT-only 感知） | 目标 <600 MB | 不常驻 torch/ultralytics |
| 端到端（语音→云台到位） | 目标 <800 ms | P2 验收 |

现有 VSLAM 占用约 2.4 GB 内存，感知链路需要控制在 1 GB 以内。

`person_gesture_node` 内取流、人体检测与手势识别共享同一进程和同一帧，
不产生 `sensor_msgs/Image` 的序列化/反序列化与 DDS 拷贝；P3/P4/P5 共用一个进程，
内存按“单进程双模型”预算。

运行时**禁止 import torch/ultralytics**；代码中加启动自检，检测到即视为配置错误并告警，
确保推理只能走 TensorRT。

---

## 7. 实施阶段

| 阶段 | 内容 | 工期 | 验收 |
|---|---|---|---|
| **P1** | `roborts_voice` 包、`voice_perception_node`（USB 读取、VAD + DOA、滤波、话题） | 1–1.5 天 | DOA 稳定 ≥10 Hz，30 min 无断连 |
| **P2** | 独立节点 `gimbal_aim_node`、状态机、限位、安全 | 1 天 | 声源指向误差 ≤5°，到位 ≤800 ms；yaw 跟随/禁止跟随两模式验证；pitch 越限停止 |
| P3 | `person_gesture_node` 内 CSI 取流、标定、静态 TF | 0.5 天 | 节点内取流稳定 30 FPS，重投影 <0.5 px（debug 话题可选） |
| P4 | `person_gesture_node` 人体检测分支：`yolo11n-pose` TRT engine 构建 + TensorRT 推理接入、发布 `/perception/persons`、泛化评估 | 1 天 | 3/5/8 m 召回率 ≥90%；TRT 推理 ≥30 FPS |
| P5 | 同一节点启用手势分支（`enable_gesture`）：手部 21 点 TRT engine、规则手势、去抖 | 1 天 | 5 类手势准确率 ≥85% |
| P6 | 目标锁定、单目测距、云台视觉跟随 | 1 天 | 稳态误差 <3°，无振荡 |
| P7 | 手势指令、`cmd_vel_mux`、联调 | 1 天 | 张开手→fist 锁定/跟随可靠，fist→张开手停止；超时停车 |

**P3–P7 已实现**（P5 手部权重需自备）。P6/P7 仍需台架确认 `yaw_sign`/`pitch_sign`、
软限位与底盘联调；P4 的“训练兜底”只在泛化实测不达标时触发。

---

## 8. 风险与对策

| 风险 | 影响 | 对策 |
|---|---|---|
| XVF3800 固件 DOA 资源号不同 | 读不到数据 | `resid` 参数化；先用 `VERSION` 验证；保留 `xvf_host` 对照 |
| USB 权限/设备未插 | 节点起不来 | 现有 udev 规则只匹配 `001e`，需按实际 PID 补充；节点重试不崩溃；启动日志明确提示 |
| 4 麦阵列 DOA 精度有限（约 ±10–15°） | 指向偏差 | 仅作粗定向，视觉阶段精修；静态标定零位 |
| 阵列随云台转动形成反馈 | 可能来回摆 | 控制律用 `current_yaw + K·DOA`，K<1，加死区与滤波 |
| 电机噪声触发 VAD | 云台自激 | 运动期间暂停 DOA 更新；VAD 连续 3 帧确认 |
| 俯仰倾斜使 DOA 平面倾斜 | 数度误差 | ±20° 内可接受；需要时做 `atan(tan·cos)` 补偿 |
| 相对/绝对角语义在固件侧不确定 | 云台意外动作 | P1 只用绝对角 + 保持当前俯仰；台架先验证 |
| 自动指向触发底盘跟随 | 机体转向改变 odom/VSLAM/costmap 位姿，干扰导航 | 导航中禁用自动指向或只允许云台内行程；静止时才允许底盘跟随；回中缓速 |
| yaw 跟随与 DOA 闭环耦合 | “云台转→底盘跟”来回摆 | 降低 K、保留死区、限制命令速率；跟随期间用 gyro 角判断收敛 |
| pitch 限位信息未上报 | 可能顶死或指令不可达 | 参数软限位（pitch_min/max）+ MCU 硬限位；请固件组确认限位值 |
| 遥控器与自动指向冲突 | 抢控制权 | 增加使能开关/仲裁，测试时断开遥控 |
| 取流与推理同进程，推理阻塞取流回调 | 掉帧/延迟增大 | 取流线程与推理线程分离；appsink `drop=true` 只保留最新帧，推理慢时跳帧不积压 |
| TensorRT engine 与 GPU / TRT 版本绑定 | 升级 L4T 后 engine 加载失败或行为异常 | 在目标 Jetson 上生成并记录 TRT 版本；升级后重建；启动时校验失败直接报错，不回退 |
| PyTorch → TRT FP16 数值差异 | 精度下降、关键点漂移 | 导出后对比 ONNX 与 TRT 输出（关键点误差/检测一致性）；必要时敏感层回退 FP32 或改用 FP32 engine |
| 自写 TRT 前后处理出错 | 框/关键点错位 | 与 PyTorch 参考输出离线逐帧对比；letterbox、NMS、坐标还原参数统一并做回归测试 |
| 1080p 与 720p 视场不同 | 标定失效 | 必须按实际分辨率标定 |
| 8 m 处手部过小 | 手势不可用 | 姿态腕点 + 自适应 ROI；必要时限制在 6 m 或换长焦 |

---

## 9. 待确认事项

1. **底盘跟随的实现细节**：yaw 到达极限后由固件自动驱动底盘跟随，还是需要上位机发底盘角速度？
   跟随速度/加速度是否可配？能否关闭（只让云台停）？
2. **Pitch 限位值**：上下限角度；是否可通过协议读取。yaw 的相对机身行程值同样需要。
3. **下位机限位信息的形式**：协议字段（命令集/ID/结构体）还是固件内部值？
   若上报，需要字段定义以扩展 `protocol_define.h` 与 `gimbal.cpp`。
4. **云台角度语义**：绝对角零点相对底盘还是云台安装零位？相对模式固件是否真正支持？
5. **XVF3800 阵列零位**：阵列正面相对云台前向的安装偏角；固件版本。
6. **控制权仲裁**：自动指向期间遥控器是否禁用？由谁切换？导航中是否禁止底盘跟随？
7. **手势集合与指令映射**是否按 4.4 节执行。
8. **是否接受**手势阶段使用社区手部关键点权重（需确认许可证）。

---

## 附录 A：标定与验证步骤

### A.1 XVF3800 零位与精度

1. 插上设备，`lsusb` 应出现 `2886:001a`；确认 `/etc/udev/rules.d/99-respeaker.rules` 生效。
2. 用 `xvf_host VERSION` 验证通信；`xvf_host AEC_AZIMUTH_VALUES` 观察 4 个 beam。
3. 将声源放在云台正前方 1 m，记录 `azimuth_raw_deg`，写入 `mic_yaw_offset_deg`。
4. 依次放在 90°/180°/270°，记录误差；超过 5° 时检查固件与波束选择。
5. 播放粉噪代替人声，验证 VAD 与电机噪声抑制。

### A.2 云台指向台架测试

1. 关闭遥控器或切到自动模式，`set_gimbal_mode(CODE_CONTROL)`。
2. 先固定小角度目标（±10°），确认方向符号正确、无反向。
3. yaw 逐步到云台相对机身的行程极限（如 ±60°/±90° 直至已知极限），观察云台到位、
   底盘是否平顺跟随、是否出现“云台-底盘”来回摆；同时确认导航模式下不触发跟随。
4. pitch 打到上下极限，确认云台停止、不顶死、不振荡。
5. 记录到位时间、超调、静止抖动，以及底盘跟随时的机体转向幅度。

### A.3 CSI 相机标定

1. 以 `publish_debug_image:=true` 启动 `person_gesture_node`（或直接用附录 B 的 `gst-launch-1.0`），
   `1920×1080@30` 采集棋盘格 20–30 张，覆盖画面四角。
2. OpenCV 标定，输出 `K`、`D`；重投影误差 <0.5 px。
3. 写入 `roborts_perception/config/camera_csi.yaml`，节点启动时加载。

---

## 附录 B：关键命令与实测记录

```bash
# CSI 相机快速验证（3 帧）
gst-launch-1.0 -e nvarguscamerasrc sensor-id=0 num-buffers=3 ! \
  'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' ! \
  nvvidconv ! videoconvert ! jpegenc ! multifilesink location=/tmp/csi_%d.jpg

# 设备检查
lsusb | grep -i 2886
arecord -l

# TensorRT engine 构建（离线一次性，必须在目标 Jetson + 相同 TRT 版本上执行）
trtexec --onnx=yolo11n-pose.onnx --fp16 --saveEngine=yolo11n-pose_fp16.engine
# 静态 ONNX（imgsz=640）无需 shape 参数；若导出为动态 shape，再补
# --minShapes/--optShapes/--maxShapes（输入名以实际 ONNX 为准，通常为 images）

# TensorRT 基准（运行时路径，无需 PyTorch）
trtexec --loadEngine=yolo11n-pose_fp16.engine --fp16 --iterations=200

# 现有云台相关代码
roborts_base/gimbal/gimbal.cpp
roborts_base/roborts_sdk/protocol/protocol_define.h
roborts_msgs/msg/GimbalAngle.msg
```

### 变更记录

| 日期 | 内容 |
|---|---|
| 2026-11-15 | 初稿：确定“先声源定位”的实施顺序；确认 yolo11n-pose 先泛化后训练；XVF3800 装云台顶部 |
| 2026-11-15 | 节点合并：语音检测（VAD）+ 声源定位（DOA）合为 `voice_perception_node`；人体检测 + 手势识别合为 `person_gesture_node`；云台指向仍为独立 `gimbal_aim_node` |
| 2026-11-15 | CSI 取流并入 `person_gesture_node`：取消 `csi_camera_node` 与 `/csi_camera/*` 图像话题中转，避免 1080p 帧经 DDS 反复拷贝；调试用 `publish_debug_image` 可选开启 |
| 2026-11-15 | 补充云台越限行为：yaw 到极限后由底盘跟随补足（全向可达，但机体转向）；pitch 到极限后云台停止（不可达）。新增跟随/禁止跟随两种模式及导航仲裁 |
| 2026-11-15 | 推理统一改为 TensorRT FP16 engine（TensorRT Python API + cuda-python），运行时不使用 PyTorch；PyTorch/ultralytics 仅用于离线导出与精度评估，不提供回退路径 |
| 2026-11-21 | P3 实现：新增 `roborts_perception`（C++ `person_gesture_node` 进程内 CSI 取流、标定工具、调试话题）与 `gimbal → csi_camera_link → csi_camera_optical_frame` 静态 TF；XVF3800 硬件故障，语音侧 P1/P2 暂缓 |
| 2026-11-21 | P4–P7 实现：TensorRT C++ 推理（yolo11n-pose，实测 8–19 ms/帧，30 FPS）、`Person/Gesture` 消息、手部 21 点规则手势（`enable_gesture`）、`gimbal_follow_node`（P6，仿真收敛 <1°）、`gesture_command_bridge` + `cmd_vel_mux`（P7，优先级/超时/急停已验证）；导航 launch 增加 `use_gesture_control`（planner 重映射到 `/cmd_vel_acc_planner`）。手部权重未随仓库提供，需自备许可证明确的 checkpoint |
| 2026-11-22 | P6/P7 交互调整：跟随默认关闭；`palm → fist` 锁定该人并开启云台跟踪，`fist → palm` 释放并结束跟踪（按 `track_id` 边沿、1.5 s 容差）；`fist` 不再后退、`palm` 不再急停；`thumbs_up` 前进仍需手势模式；手势冷却缩短到 0.8 s |
| 2026-11-22 | 一体化启动：新增 `perception_control.launch.py` 一次启动 P3–P7；导航 `use_gesture_control:=true` 改为包含该 launch；可选 `use_viewer`（HUD/RViz）与 `publish_csi_tf`（独立运行且主 bringup 未发布 CSI TF 时） |
| 2026-11-22 | 实机复测：张开手被模型识别为 `pointing_up`，`palm` 始终为 0；跟踪开关改为“张开手（`palm`/`victory`/`pointing_up`）↔ `fist`”；`perception_control` 默认启动 `roborts_base`（`start_base:=true`），导航包含时传 `false`；重建 `roborts_msgs`/`roborts_base` 修复旧 `GimbalLimit` 符号 |
| 2026-11-22 | 跟随提速：增益 0.9、死区 0.8°、限速 180°/s、50 Hz；0.5 rad 阶跃 0.40 s 进入 1.15° 窗口（超调约 2.2°） |
