# RDK 部署指南

本指南说明如何将 `rescue_robot` 项目从 Windows 电脑导入到 RDK X3/X5 开发板并运行。

> **用户说明**：RDK X3/X3 Module 默认用户为 `root`，RDK X5 默认用户为 `sunrise`。本指南已按 RDK X5（`sunrise` / `/home/sunrise/ros2_ws`）更新，若使用 RDK X3 请将命令中的 `sunrise` 替换为 `root`，`/home/sunrise` 替换为 `/root`。

## 前提条件

1. RDK 已开机并接入网络，IP 可访问
2. 电脑与 RDK 处于同一局域网，或电脑可直接连接 RDK
3. 已在 RDK 上安装 TogetheROS.Bot（TROS）
4. 电脑上已安装 Git for Windows（自带 `ssh`、`scp`）

## 方式一：使用 PowerShell 一键部署脚本（推荐）

### 1. 修改脚本中的 RDK IP

编辑 `scripts/deploy_to_rdk.ps1`，将默认 IP 改为你的 RDK 实际 IP：

```powershell
[string]$RDK_IP = "192.168.31.174"
```

### 2. 执行部署

在 PowerShell 中进入项目目录：

```powershell
cd C:\Users\jingwai\WorkBuddy\2026-07-05-00-10-44\rescue_robot
.\scripts\deploy_to_rdk.ps1
```

脚本会自动完成：打包项目、上传到 RDK、解压、编译。

### 3. 在 RDK 上启动

部署完成后，SSH 登录 RDK 并启动：

```bash
ssh sunrise@192.168.31.174
source /home/sunrise/ros2_ws/install/setup.bash
ros2 launch rescue_robot rescue_robot.launch.py
```

如需同时启动 GS130w 摄像头驱动和视觉节点：

```bash
# 终端1：启动摄像头
ros2 launch rescue_robot mipi_cam_gs130w.launch.py

# 终端2：启动视觉节点
ros2 launch rescue_robot rescue_robot.launch.py
```

## 方式二：使用 Bash 脚本（WSL / Git Bash）

如果你使用 WSL 或 Git Bash：

```bash
cd C:/Users/jingwai/WorkBuddy/2026-07-05-00-10-44/rescue_robot
bash scripts/deploy_to_rdk.sh
```

同样需要在脚本中修改 `RDK_IP`。

## 方式三：手动 SCP 部署

如果你只想手动用 `scp` 上传，请参考 [`SCP手动部署指南.md`](./SCP手动部署指南.md)。

## 自定义 YOLOv8 BPU 模型转换（可选）

如果你希望使用自己导出的 YOLOv8 ONNX 模型并启用 BPU 加速，需要在 Linux/WSL 环境中使用 D-Robotics `hb_mapper` 工具链将其转换为 `.bin`。

### 1. 准备环境

- 安装 D-Robotics 工具链（参考官方文档，或使用 Docker）：
  ```bash
  docker pull drobotics/horizon_xj3_open_explorer
  ```
- 已导出 ONNX：`python3 scripts/export_yolov8_onnx.py yolov8n`
- 准备标定图片：`calibration_data/` 目录下放置 100~200 张 640x640 的 COCO 图片

### 2. 修改 BPU 配置

编辑 [config/yolov8n_bpu.yaml](../config/yolov8n_bpu.yaml)，根据 RDK 型号设置正确的 `march`：

- RDK X3 / X3 Module：`bernoulli2`
- RDK X5 / Ultra：请参考官方文档（通常为 `bayes`）

### 3. 执行转换

```bash
bash scripts/convert_yolov8_bpu.sh yolov8n
```

转换成功后会在 `bpu_output/` 目录生成 `yolov8n.bin`（或 `.hbm`）。

### 4. 上传到 RDK 并切换配置

```bash
scp bpu_output/yolov8n.bin sunrise@<RDK_IP>:/opt/rescue_robot/models/
```

然后修改 `config/robot_params.yaml`：

```yaml
body_detection:
  model_path: /opt/rescue_robot/models/yolov8n.bin
  use_bpu: true
```

> **注意**：`body_detector_node.py` 已通过 `rescue_robot/bpu_yolov8_detector.py` 接入 D-Robotics BPU 运行时。转换后的 `yolov8n.bin` 只要输出形状与 ONNX 一致（`(1, 84, 8400)`），即可直接启用 BPU 推理。

## 常见问题

### 1. 无法 SSH 到 RDK

- 确认 RDK 与电脑在同一网络
- 确认 RDK 已开启 SSH 服务：`systemctl status ssh`
- 尝试使用 RDK 的默认 IP 或查看路由器后台

### 2. colcon build 报错缺少依赖

在 RDK 上安装 ROS2 依赖：

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-vision-msgs
```

### 3. 摄像头启动失败

- 确认 GS130w 已正确连接到 RDK 的 MIPI 接口
- 确认已安装 `tros-hobot-mipi-cam`：
  ```bash
  sudo apt install tros-hobot-mipi-cam
  ```
- 检查摄像头节点输出：
  ```bash
  ros2 topic list
  ros2 topic hz /mipi_cam/image_raw
  ```

### 4. 模型文件不存在

在 RDK 上执行模型部署脚本：

```bash
cd /home/sunrise/ros2_ws/src/rescue_robot
bash scripts/deploy_models.sh
```

或手动下载/转换模型到 `/opt/rescue_robot/models/`。

## 推荐目录结构（RDK 端）

```
/home/sunrise/ros2_ws/
├── src/
│   └── rescue_robot/       # 本项目
├── build/
├── install/
└── log/
```
