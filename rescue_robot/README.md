# 基于 RDK 单目视觉的塌方废墟救援寻路机器人 - 纯视觉感知方案

本 ROS2 Python 包为**纯视觉感知方案**，仅包含视觉节点，不依赖轮式里程计、IMU、激光雷达或超声波。包括：
- 图像采集（USB / MIPI 相机）
- 人体检测 + 单目测距
- 单目深度估计 + 可通行区域分析
- 实时可视化与日志

决策规划、底盘控制、里程计等代码未包含，可按 `/rescue_robot/body_detections`、
`/rescue_robot/nearest_obstacle_distance` 等话题自行接入上层节点。

## 项目结构

```
rescue_robot/
├── rescue_robot/                  # 节点源码
│   ├── camera_node.py             # 相机采集（USB / MIPI 转发）
│   ├── body_detector_node.py      # 人体检测 + 单目测距
│   ├── depth_estimator_node.py    # 单目深度估计 + 可通行区域分析
│   └── visualization_node.py      # 实时可视化 + 日志记录
├── config/
│   ├── camera_params.yaml         # 相机标定参数
│   └── robot_params.yaml          # 视觉算法参数
├── launch/
│   ├── rescue_robot.launch.py              # 启动纯视觉节点
│   └── mipi_cam_gs130w.launch.py           # RDK GS130w 摄像头驱动
├── scripts/
│   ├── camera_calibration.py      # 棋盘格相机标定
│   ├── deploy_models.sh           # BPU 模型部署脚本
│   ├── deploy_to_rdk.sh           # RDK 部署脚本（Bash，含SCP）
│   ├── deploy_to_rdk.ps1          # RDK 部署脚本（PowerShell，含SCP）
│   └── offline_simulation.py      # PC 离线仿真
├── docs/
│   ├── RDK部署指南.md             # RDK 部署详细说明
│   └── SCP手动部署指南.md         # 纯 SCP 手动部署步骤
├── package.xml
├── setup.py
├── setup.cfg
├── requirements.txt
└── README.md
```

## 依赖

- RDK X3 / X5 或 PC（离线仿真）
- TogetheROS.Bot (ROS2 Humble/Foxy) — 板端运行
- Python 3.8+
- OpenCV 4.x
- cv_bridge
- vision_msgs

## 安装与编译

详细 RDK 部署步骤见 [docs/RDK部署指南.md](docs/RDK部署指南.md)。

### 快速部署（PowerShell，推荐）

```powershell
cd rescue_robot
.\scripts\deploy_to_rdk.ps1
```

### 手动 SCP 部署

如果你只想用 `scp` 手动上传，详见 [docs/SCP手动部署指南.md](docs/SCP手动部署指南.md)。

核心命令：

```powershell
# 1. 打包
cd C:\Users\jingwai\WorkBuddy\2026-07-05-00-10-44
tar --exclude='.git' --exclude='__pycache__' --exclude='*.pyc' -czf rescue_robot.tar.gz rescue_robot

# 2. SCP 上传到 RDK（替换 IP；RDK X5 用户为 sunrise，X3 为 root）
scp -P 22 rescue_robot.tar.gz sunrise@192.168.31.174:/tmp/

# 3. 在 RDK 上解压编译
ssh sunrise@192.168.31.174 "bash -c 'mkdir -p /home/sunrise/ros2_ws/src && rm -rf /home/sunrise/ros2_ws/src/rescue_robot && tar -xzf /tmp/rescue_robot.tar.gz -C /home/sunrise/ros2_ws/src/ && cd /home/sunrise/ros2_ws && source /opt/tros/setup.bash && colcon build --packages-select rescue_robot --symlink-install'"
```

## 运行

### RDK GS130w MIPI 摄像头使用说明

本项目默认使用 **RDK GS130w 单目 MIPI 摄像头**。配置如下：

1. 确认 `config/camera_params.yaml` 中：
   ```yaml
   camera:
     camera_type: mipi
     mipi_topic: /mipi_cam/image_raw
     width: 640
     height: 480
     fps: 30
   ```

2. 启动 TROS 官方 `mipi_cam` 节点（需先安装）：
   ```bash
   sudo apt install tros-hobot-mipi-cam
   ros2 launch rescue_robot mipi_cam_gs130w.launch.py
   # 或使用官方 launch：
   # ros2 launch mipi_cam mipi_cam.launch.py
   ```

3. 启动视觉节点：
   ```bash
   ros2 launch rescue_robot rescue_robot.launch.py
   ```

### 一键启动视觉节点

```bash
ros2 launch rescue_robot rescue_robot.launch.py
```

### 单独启动节点

```bash
ros2 run rescue_robot camera_node
ros2 run rescue_robot body_detector_node
ros2 run rescue_robot depth_estimator_node
ros2 run rescue_robot visualization_node
```

### PC 离线仿真（无需 RDK/ROS2）

```bash
python3 scripts/offline_simulation.py
```

## 核心话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/rescue_robot/image_raw` | sensor_msgs/Image | 原始相机图像 |
| `/rescue_robot/camera_info` | sensor_msgs/CameraInfo | 相机内参 |
| `/rescue_robot/body_detections` | vision_msgs/Detection2DArray | 人体检测结果 |
| `/rescue_robot/nearest_human_distance` | std_msgs/Float32 | 最近人体距离 |
| `/rescue_robot/depth_image` | sensor_msgs/Image | 深度热力图 |
| `/rescue_robot/passable_mask` | sensor_msgs/Image | 可通行区域掩码 |
| `/rescue_robot/nearest_obstacle_distance` | std_msgs/Float32 | 最近障碍距离 |
| `/rescue_robot/passable_offset` | std_msgs/Int32 | 可通行区域中心偏移 |
| `/rescue_robot/overlay_image` | sensor_msgs/Image | 叠加可视化图像 |
| `/rescue_robot/status_log` | std_msgs/String | 搜救状态日志 |

## 关键参数配置

编辑 `config/robot_params.yaml`：

- `depth.model_path`: 深度模型路径
- `body_detection.model_path`: 人体检测模型路径
- `obstacle_avoidance.safe_distance`: 安全距离阈值
- `obstacle_avoidance.danger_distance`: 危险距离阈值

编辑 `config/camera_params.yaml`：

- `camera.fx/fy/cx/cy`: 相机内参
- `camera.k1~k5`: 畸变系数
- `distance_estimation.*`: 人体测距参数

## 相机标定

```bash
python3 scripts/camera_calibration.py --collect --device 0 --square_size 0.025
```

生成的 `camera_params.yaml` 可直接替换 `config/camera_params.yaml`。

## 模型部署

### 人体检测模型

推荐使用 TROS 官方 `mono2d_body_detection` 包：

```bash
ros2 launch mono2d_body_detection mono2d_body_detection.launch.py
```

并在 `body_detector_node` 中订阅 `/hobot_mono2d_body_detection` 输出。

### 单目深度模型

使用 Depth Anything V2 Small 转换后的 BPU 模型：

```bash
# 模型路径
/opt/rescue_robot/models/depth_anything_v2_small.bin
```

纯视觉方案下，`depth_estimator_node` 直接订阅图像进行单目深度推理，
**不依赖外部双目深度节点**。

## 测试模式

若未提供模型，节点会自动进入测试模式：

- 生成模拟人体检测框
- 生成模拟深度图
- 验证视觉链路

## 与上层决策/底盘控制代码对接

本包为纯视觉感知层，不直接控制电机/舵机，也不发布里程计。
你只需让上层节点订阅视觉节点发布的话题即可。

### 推荐对接方式

直接订阅以下话题实现决策：

| 话题 | 用途 |
|------|------|
| `/rescue_robot/body_detections` | 人体位置与距离 |
| `/rescue_robot/nearest_human_distance` | 最近人体距离 |
| `/rescue_robot/nearest_obstacle_distance` | 最近障碍距离 |
| `/rescue_robot/passable_offset` | 可通行区域中心偏移 |

示例：

```python
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float32, Int32

# 订阅视觉话题，自行实现决策与控制
self.det_sub = self.create_subscription(Detection2DArray, '/rescue_robot/body_detections', self.det_callback, 10)
self.obs_sub = self.create_subscription(Float32, '/rescue_robot/nearest_obstacle_distance', self.obs_callback, 10)
self.offset_sub = self.create_subscription(Int32, '/rescue_robot/passable_offset', self.offset_callback, 10)
```

## 扩展建议

如需接入导航和底盘，可订阅以下话题实现决策：

- `/rescue_robot/body_detections`：人体位置与距离
- `/rescue_robot/nearest_obstacle_distance`：最近障碍距离
- `/rescue_robot/passable_offset`：可通行方向
