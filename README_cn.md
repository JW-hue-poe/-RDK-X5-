# rescue_robot 中文文档（README_cn）

> 基于 **RDK X5** 单目视觉的塌方废墟救援寻路机器人（ROS2 / TogetheROS.Bot）

---

## 1. 项目简介

`rescue_robot` 是一个运行在 D-Robotics **RDK X5** 上的 ROS2 感知软件包。它采用**纯单目视觉感知方案**——不依赖轮式里程计、IMU、激光雷达或超声波，仅通过一枚单目摄像头完成环境感知，为上层的决策/底盘控制节点提供：

- **人体检测**：基于 YOLOv8（可跑 BPU 加速）识别幸存者；
- **单目测距**：根据人体像素高度估算其与机器人的距离；
- **单目深度估计**：基于 Depth Anything V2，估计场景深度；
- **可通行区域分析**：在画面底部 ROI 内判断障碍距离与可通行偏移，用于避障；
- **云台自动跟踪**：通过 PCA9685 + SG90 舵机驱动 Pan 单轴云台，自动将最大人体框居中。

本包只负责"感知 + 视野控制"，运动控制、路径规划等需由上层节点订阅相应话题自行实现。

---

## 2. 系统架构

感知流水线（ROS2 话题通信）：

```
[相机 MIPI GS130W / USB / 仿真]
        │  /mipi_cam/image_raw (sensor_msgs/Image, NV12 或 bgr8)
        ▼
   camera_node  ──▶ /rescue_robot/image_raw (bgr8)  +  /rescue_robot/camera_info (CameraInfo)
        │
        ├────────────▶ body_detector_node (YOLOv8 / BPU)  ──▶ /rescue_robot/body_detections (Detection2DArray)
        │                                                     └─▶ /rescue_robot/nearest_human_distance (Float32)
        ├────────────▶ depth_estimator_node (Depth Anything V2) ─▶ /rescue_robot/depth_image (Image, JET)
        │                                                     ├─▶ /rescue_robot/passable_mask (Image, mono8)
        │                                                     ├─▶ /rescue_robot/nearest_obstacle_distance (Float32)
        │                                                     └─▶ /rescue_robot/passable_offset (Int32)
        └────────────▶ visualization_node (叠加显示 + 状态日志)
   body_detector_node ─▶ /rescue_robot/body_detections ─▶ camera_servo_node (PCA9685+SG90 云台自动跟踪)
                                                          └─▶ /rescue_robot/camera_pan_angle (Float64)
```

---

## 3. 目录结构

```
rescue_robot/
├── README.md                  # 英文/原版说明
├── 项目代码说明.html           # 带样式的 HTML 代码说明
├── package.xml                # ROS2 包描述
├── setup.py / setup.cfg       # 构建与入口节点注册
├── requirements.txt           # Python 依赖
├── config/                    # 相机/算法/模型转换参数
│   ├── camera_params.yaml
│   ├── robot_params.yaml
│   ├── depth_anything_v2_bpu.yaml
│   ├── depth_vits_392x518_v124.yaml
│   └── yolov8n_bpu.yaml
├── launch/                    # 启动文件
│   ├── rescue_robot.launch.py
│   ├── rescue_robot_monocular.launch.py
│   └── mipi_cam_gs130w.launch.py
├── rescue_robot/              # ROS2 节点源码
│   ├── camera_node.py
│   ├── body_detector_node.py
│   ├── depth_estimator_node.py
│   ├── visualization_node.py
│   ├── camera_servo_node.py
│   ├── bpu_model.py
│   ├── bpu_yolov8_detector.py
│   ├── utils.py
│   └── __init__.py
├── scripts/                   # 模型导出 / 转换 / 部署 / 标定 / 仿真脚本
├── docs/                      # 设计文档与部署指南
│   ├── 基础设计文档.md
│   ├── RDK部署指南.md
│   └── SCP手动部署指南.md
├── urdf/                      # 机器人 URDF 描述
│   └── rescue_robot.urdf
├── test/                      # 单元测试与冒烟测试
├── resource/                  # 资源占位
└── patch_inspect*.py          # Python inspect 兼容补丁
```

---

## 4. 硬件平台

| 硬件 | 说明 |
|------|------|
| 主控 | D-Robotics **RDK X5**（亦兼容 X3），运行 **TogetheROS.Bot (TROS) / ROS2 Humble** |
| 相机 | **RDK GS130W 单目 MIPI 模组**（sensor `sc132gs`，`device_mode=single`，支持 `nv12`/`bgr8`）；USB 摄像头为备选 |
| 云台舵机 | **SG90** + **PCA9685** I2C PWM 驱动（`i2c_bus=5`，`i2c_address=0x40`，50Hz，脉宽 500~2500µs），Pan 单轴 |
| BPU 模型 | YOLOv8n（人体检测）；Depth Anything V2 Small（单目深度） |

> GS130W 模组标定参数存于 EEPROM，开启 `mipi_gdc_enable` 后由 GDC 硬件完成去畸变/行对齐，故 `camera_params.yaml` 中畸变系数默认填 0。

---

## 5. 软件依赖

**ROS2 依赖**（`package.xml`）：`rclpy`、`std_msgs`、`sensor_msgs`、`cv_bridge`、`image_transport`、`vision_msgs`，`exec_depend` 含 `robot_state_publisher`、`hobot_mipi_cam`。

**Python 依赖**（`requirements.txt`）：
```
opencv-python>=4.8.0
numpy>=1.24.0
PyYAML>=6.0
# 可选（PC 端测试 / ONNX 推理）：onnxruntime>=1.15.0
```

**系统包**（RDK 上）：
```bash
apt install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-vision-msgs tros-hobot-mipi-cam
```

---

## 6. 核心节点说明

所有节点源码位于 `rescue_robot/rescue_robot/`，由 `setup.py` 注册为 `console_scripts` 可执行入口。

### 6.1 `camera_node` — 图像采集
- **类**：`CameraNode`；**发布**：`/rescue_robot/image_raw`(bgr8)、`/rescue_robot/camera_info`
- 支持三种模式：**MIPI 代理**（订阅 `/mipi_cam/image_raw`）、**USB 直连**（`usb_device`）、**仿真**（绘制绿色人体矩形）。NV12→BGR 转换；USB 断线每 2s 自动重连。
- 关键参数：`camera_type`、`simulate`、`width/height`(640/480)、`fps`(30)、`fx/fy/cx/cy`、`k1~k3/p1/p2`。

### 6.2 `body_detector_node` — 人体检测与测距
- **类**：`BodyDetectorNode`；**订阅**：`/rescue_robot/image_raw`；**发布**：`/rescue_robot/body_detections`(Detection2DArray)、`/rescue_robot/nearest_human_distance`(Float32)
- 模型初始化优先级：**BPU** → OpenCV DNN 加载同名 `.onnx` → 测试模式（中心模拟框）。
- 后处理支持 ONNX `[1,84,8400]` 与 BPU 单输出 / 6 头输出（strides `[8,16,32]`，`reg_max=16`，DFL 解码）。
- 基于 IOU 的简单跟踪，稳定 `track_id`；测距调用 `estimate_human_distance`；无订阅者时跳过推理省算力。
- 关键参数：`model_path`、`use_bpu`、`conf_threshold`(0.5)、`nms_threshold`(0.45)、`average_human_height`(1.7)、`camera_height`(0.45)、`camera_pitch`(10.0)。

### 6.3 `depth_estimator_node` — 单目深度估计与可通行区域
- **类**：`DepthEstimatorNode`；**发布**：`/rescue_robot/depth_image`、`/rescue_robot/passable_mask`、`/rescue_robot/nearest_obstacle_distance`、`/rescue_robot/passable_offset`
- 可订阅外部深度话题（`depth_topic`，支持 `16UC1`/`32FC1`）；否则对 `/rescue_robot/image_raw` 跑 Depth Anything V2。
- ROI 取图像底部 `[roi_top, roi_bottom]`；可通行掩码 = 深度 > 安全距离，经形态学开/闭运算去噪；最近障碍距离取 ROI 内 5% 分位；偏移调用 `compute_passable_offset`。
- 关键参数：`use_bpu`、`input_width/height`(518/392)、`safe_distance`(0.8)、`danger_distance`(0.4)、`roi_top`(0.45)、`roi_bottom`(0.85)、`passable_width`(120)。

### 6.4 `visualization_node` — 可视化与日志
- **类**：`VisualizationNode`；**发布**：`/rescue_robot/overlay_image`、`/rescue_robot/status_log`(JSON)
- 叠加检测框 + 距离 + 状态文本 + 右下角深度小窗；按 `RobotState` 统计避障次数/累计人体数；检测到人体时按 `save_path` 自动截图（最小间隔 0.5s）；定时发布 JSON 状态日志。

### 6.5 `camera_servo_node` — 舵机云台控制
- **类**：`CameraServoNode` + `PCA9685Controller`（I2C/SMBus2 PWM 封装）；**发布**：`/rescue_robot/camera_pan_angle`(Float64)
- 订阅手动遥控 `/rescue_robot/cmd_camera_pan` 与自动跟踪 `/rescue_robot/body_detections`（选面积最大人体框居中）。
- 无硬件时进入模拟模式只打日志；以 `max_speed`(45°/s) 限角速度平滑逼近。

### 6.6 工具与 BPU 封装
- `utils.py`：`RobotState` 枚举（EXPLORING/AVOIDING/HUMAN_DETECTED/HUMAN_PAUSED/U_TURN/STOPPED）、`clamp`、`compute_iou`、`estimate_human_distance`、`compute_passable_offset`、`normalize_depth_for_display`。
- `bpu_model.py`：`BPUModel` 通用封装（依赖 TROS `hobot_dnn.pyeasy_dnn`），导入前设置 `LD_LIBRARY_PATH`/`PATH`，支持 NV12 模型、letterbox、BGR→NV12。
- `bpu_yolov8_detector.py`：兼容别名（`BPUYolov8Detector = BPUModel`）。

---

## 7. 核心话题表

| 话题 | 类型 | 方向 |
|------|------|------|
| `/rescue_robot/image_raw` | `sensor_msgs/Image`(bgr8) | camera → 检测/深度/可视化 |
| `/rescue_robot/camera_info` | `sensor_msgs/CameraInfo` | camera → 外部 |
| `/rescue_robot/body_detections` | `vision_msgs/Detection2DArray` | 检测 → 云台 |
| `/rescue_robot/nearest_human_distance` | `std_msgs/Float32` | 检测 → 外部 |
| `/rescue_robot/depth_image` | `sensor_msgs/Image`(JET) | 深度 → 可视化 |
| `/rescue_robot/passable_mask` | `sensor_msgs/Image`(mono8) | 深度 → 外部 |
| `/rescue_robot/nearest_obstacle_distance` | `std_msgs/Float32` | 深度 → 外部 |
| `/rescue_robot/passable_offset` | `std_msgs/Int32` | 深度 → 外部 |
| `/rescue_robot/cmd_camera_pan` | `std_msgs/Float64` | 外部 → 云台 |
| `/rescue_robot/camera_pan_angle` | `std_msgs/Float64` | 云台 → 外部 |
| `/rescue_robot/overlay_image` | `sensor_msgs/Image` | 可视化 → 外部 |
| `/rescue_robot/status_log` | `std_msgs/String`(JSON) | 可视化 → 外部 |

---

## 8. 配置参数

主要配置文件：`config/camera_params.yaml`（相机内参/类型）、`config/robot_params.yaml`（算法/模型总配置，按节点名分节）。

典型参数（`robot_params.yaml`）：
- `body_detector_node`：`model_path`、`use_bpu`、`class_id`(0)、`conf_threshold`、`nms_threshold`、`input_width/height`、`letterbox`、`input_format`(rgb)、`max_track_miss`(30)
- `distance_estimation`：`average_human_height`(1.7)、`camera_height`(0.45)、`camera_pitch`(10.0)
- `depth_estimator_node.depth`：`input_width/height`、`min_depth`(0.1)、`max_depth`(10.0)、`scale`、`offset`、`use_bpu`、`model_path`、`depth_topic`
- `obstacle_avoidance`：`safe_distance`(0.8)、`danger_distance`(0.4)、`passable_width`(120)、`roi_top`(0.45)、`roi_bottom`(0.85)
- `camera_servo_node`：servo/auto_track 全参数（`pan_channel`(0)、`i2c_bus`(5)、`i2c_address`(0x40)、`pan_min/max_angle`、`max_speed`、`kp` 等）
- `visualization_node`：`show_ui`、`publish_overlay`、`log_interval`、`save_path`、`safe_distance`

> ⚠️ **参数不一致提醒**：`robot_params.yaml` 中人体模型为 `best_256x256.onnx`（`use_bpu=false`），而 `docs/基础设计文档.md` 写的是 `best_detect_bayese_640x640_nv12.bin`，README 又提到 `depth_anything_v2_small.bin`。深度输入尺寸也存在 392×518 与 518×518 的差异。请以实际部署的模型为准，统一修改。

---

## 9. 构建与运行

**构建（colcon）**：
```bash
cd ~/ros2_ws
source /opt/tros/setup.bash
colcon build --packages-select rescue_robot --symlink-install
source install/setup.bash
```

**启动**：
```bash
ros2 launch rescue_robot rescue_robot.launch.py             # 纯视觉节点（默认）
ros2 launch rescue_robot mipi_cam_gs130w.launch.py         # GS130W 相机驱动（需先启）
ros2 launch rescue_robot rescue_robot_monocular.launch.py  # 相机+视觉一体
```
> ⚠️ `rescue_robot.launch.py` 内部已启动 `camera_servo_node`，而 `rescue_robot_monocular.launch.py` 又额外启动同名节点，**会重复启动同一节点名**，实际运行可能冲突。如需一体启动，建议仅保留一个 `camera_servo_node`。

单节点运行：
```bash
ros2 run rescue_robot camera_node
ros2 run rescue_robot body_detector_node
ros2 run rescue_robot depth_estimator_node
ros2 run rescue_robot visualization_node
ros2 run rescue_robot camera_servo_node
```

---

## 10. 部署到 RDK X5

提供三种方式（详见 `docs/`）：

1. **PowerShell 一键**：`scripts/deploy_to_rdk.ps1`（先改 `RDK_IP`）。
2. **Bash 部署**：`scripts/deploy_to_rdk.sh`（默认 `RDK_USER=sunrise`、`RDK_IP=192.168.31.174`、`RDK_PORT=22`、`RDK_WORKSPACE=/home/sunrise/ros2_ws`）。
3. **手动 SCP**：打包 → `scp` 到 `/tmp/` → `ssh` 内解压到 `src/` → `colcon build`（见 `docs/SCP手动部署指南.md`）。

`scripts/deploy_models.sh` 会在 RDK 上建立 `/opt/rescue_robot/models`，并拷贝 YOLOv8 / 深度 bin 与 TROS 自带的 `mono2d_body_detection.bin`。

---

## 11. 模型转换（可选 BPU 加速）

YOLOv8：
```bash
python3 scripts/export_yolov8_onnx.py          # PC 端导出 ONNX（需 ultralytics）
python3 scripts/generate_calib_data.py         # 生成标定图片
bash scripts/convert_yolov8_bpu.sh yolov8n     # 需 D-Robotics hb_mapper / Docker
```

深度模型：
```bash
python3 scripts/export_depth_anything_v2_onnx.py --model-dir /opt/rescue_robot/models
# 用 config/depth_anything_v2_bpu.yaml 或 depth_vits_392x518_v124.yaml 转 BPU
```

转换后把 `.bin` 放到 `/opt/rescue_robot/models/`，并在 `robot_params.yaml` 设 `model_path` 与 `use_bpu: true`。

> ⚠️ `config/yolov8n_bpu.yaml` 的 `march: bernoulli2` 对应 **X3**；本项目目标板是 **RDK X5**，应改为 `bayes` / `bayes-e`。

---

## 12. 相机标定

`scripts/camera_calibration.py` 支持棋盘格标定（`-collect` 采集、`calibrate`、`save_yaml`、`update_config_yaml` 写回 `config/camera_params.yaml`）。GS130W 因 EEPROM 已标定，一般无需此脚本。

---

## 13. PC 离线仿真

```bash
python3 scripts/offline_simulation.py
```
不依赖 ROS2/BPU，`OfflineCamera` / `OfflineBodyDetector` / `OfflineDepthEstimator` / `OfflineNavigator` 四类复现检测 + 深度 + 决策 + 可视化窗口。`scripts/test_yolov8_onnx.py` 可在 PC 上直接测 YOLOv8 ONNX（输出 `yolov8_result.jpg`）。

---

## 14. 测试

```bash
pytest test/test_utils.py            # 工具函数单元测试（clamp/IOU/测距/偏移等）
pytest test/test_nodes_smoke.py      # 节点导入冒烟测试（camera 初始化需 ROS2 环境，已 skip）
```

---

## 15. 补丁脚本说明

`patch_inspect*.py`（patch_inspect / patch_inspect2 / patch_inspect3）用于**就地修补 Python 标准库 `inspect.py`**，修复某依赖（疑似 `hobot_dnn` / `pyeasy_dnn` 在导入时触发 inspect 反射）与当前 Python 解释器 `inspect` 不兼容导致的崩溃。三者采用不同改写策略，目的相同。

> ⚠️ `patch_bpu_revert.py` 当前为 **0 字节空文件**，疑似"还原补丁"占位/未实现。如需还原 inspect，请手动恢复被改动的 `inspect.py`，或补全该脚本。

---

## 16. 文档索引

| 文档 | 内容 |
|------|------|
| `README.md` | 项目概况、目录、依赖、部署、运行、核心话题、关键参数、相机标定、模型部署 |
| `项目代码说明.html` | 带样式的 HTML 版代码说明 |
| `docs/基础设计文档.md` | 最详尽：系统架构图、节点职责表、算法公式、话题/参数表、模型格式 |
| `docs/RDK部署指南.md` | 三种部署方式、自定义 YOLOv8 BPU 转换、FAQ |
| `docs/SCP手动部署指南.md` | 纯 tar+scp+colcon 手动步骤与 Windows 命令参考 |

---

## 17. 已知问题 / 注意事项

1. `rescue_robot_monocular.launch.py` 会重复启动 `camera_servo_node`，建议修正。
2. 模型名/尺寸/`march` 在多处文档与配置间不一致，部署前统一。
3. `patch_bpu_revert.py` 为空，补丁还原流程未实现。
4. 纯视觉方案不依赖里程计，定位/回环需上层自行处理。
