# 手动 SCP 部署指南

如果你只想通过 `scp` 把项目传到 RDK，然后在 RDK 上手动编译，可按以下步骤操作。

> **用户说明**：RDK X3/X3 Module 默认用户为 `root`，RDK X5 默认用户为 `sunrise`。以下示例按 RDK X5 编写，若使用 RDK X3 请将 `sunrise` 替换为 `root`，`/home/sunrise` 替换为 `/root`。

## 1. 在 Windows 上打包项目

```powershell
cd C:\Users\jingwai\WorkBuddy\2026-07-05-00-10-44
tar --exclude='.git' --exclude='__pycache__' --exclude='*.pyc' -czf rescue_robot.tar.gz rescue_robot
```

> 也可使用 `Compress-Archive` 打成 zip，但 RDK 上需先安装 `unzip`。

## 2. 使用 SCP 上传到 RDK

```powershell
scp -P 22 rescue_robot.tar.gz sunrise@192.168.31.174:/tmp/
```

如果你的 RDK 使用其他端口、用户名或 IP，请相应修改：

```powershell
scp -P <端口> rescue_robot.tar.gz <用户名>@<RDK_IP>:/tmp/
```

## 3. 在 RDK 上解压并编译

SSH 登录 RDK：

```bash
ssh sunrise@192.168.31.174
```

在 RDK 上执行：

```bash
mkdir -p /home/sunrise/ros2_ws/src
rm -rf /home/sunrise/ros2_ws/src/rescue_robot
tar -xzf /tmp/rescue_robot.tar.gz -C /home/sunrise/ros2_ws/src/
cd /home/sunrise/ros2_ws
source /opt/tros/setup.bash
colcon build --packages-select rescue_robot --symlink-install
```

## 4. 启动

```bash
source /home/sunrise/ros2_ws/install/setup.bash
ros2 launch rescue_robot rescue_robot.launch.py
```

---

## 更简单的 Windows 命令参考

```powershell
# 1. 打包并上传（请替换 IP）
cd C:\Users\jingwai\WorkBuddy\2026-07-05-00-10-44
tar --exclude='.git' --exclude='__pycache__' --exclude='*.pyc' -czf rescue_robot.tar.gz rescue_robot
scp -P 22 rescue_robot.tar.gz sunrise@192.168.31.174:/tmp/

# 2. 在 RDK 上解压编译（ssh 执行）
ssh sunrise@192.168.31.174 "bash -c 'mkdir -p /home/sunrise/ros2_ws/src && rm -rf /home/sunrise/ros2_ws/src/rescue_robot && tar -xzf /tmp/rescue_robot.tar.gz -C /home/sunrise/ros2_ws/src/ && cd /home/sunrise/ros2_ws && source /opt/tros/setup.bash && colcon build --packages-select rescue_robot --symlink-install'"
```

> 注意：Windows 上执行 `ssh` 命令需要确保已安装 OpenSSH 客户端（Windows 10/11 默认自带）。

