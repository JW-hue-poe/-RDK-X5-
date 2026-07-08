# qs 中文文档（README_cn）

> 基于 **RDK X5** 的地震坍塌救援机器人控制系统（四轮驱动 + 四关节姿态 + ASR 语音 + DeepSeek 决策 + LoRa 通信）

---

## 1. 项目简介

`qs` 是一个运行在 D-Robotics **RDK X5** 上的救援机器人"大脑"控制程序。硬件由 **4 个轮式电机（移动）+ 4 个关节舵机（姿态：正身 / 附身 / 趴下）+ 抓取舵机 + ASRPRO 语音模块 + 串口 HMI 触摸屏 + LoRa 数传 + PCA9685 PWM 扩展板**组成。

系统采用分层架构：感知 → 决策 → 执行 → 通信 的闭环控制。语音/HMI 作为外部触发输入，DeepSeek 大模型根据感知数据给出高层动作决策，再由 `BodyControl` 驱动底层硬件执行，并通过 LoRa 向指挥中心回传状态。

> 代码、注释与 `TECHNICAL_DOC.md` 均围绕"救援机器人"展开；其硬件形态（四轮 + 关节舵机 + 可切换正身/附身/趴下姿态）等价于一个可变形轮椅/辅助机器人底盘。

---

## 2. 软件架构

```
摄像头(模拟)/感知 ──▶ _perceive() ──▶ _decide() [DeepSeek] ──▶ _execute() [BodyControl] ──▶ _communicate() [LoRa]
                                                      │
                        语音/HMI 线程 ──▶ execute_action() ──▶ _execute()
```

- **入口**：`main.py` 的 `RescueRobot` 类，负责初始化各模块并进入 `main_loop`（每 0.2s 一次）。
- **分层**：
  - `Driver/`：硬件驱动（PCA9685、电机、舵机、LoRa、HMI、语音）
  - `Control/`：姿态控制（`BodyControl`）
  - `App/`：AI 决策（`DeepSeek`）
  - `test/`：测试文件

`RescueRobot.ACTION_MAP` 将高层动作关键词映射到 `BodyControl` 方法：
`forward→upright_forward`、`backward→upright_backward`、`turn_left→upright_left_turn`、`turn_right→upright_right_turn`、`stop→stop`、`lie_down→lie_down_forward`、`lean→lean_forward`、`grab→grab`。

---

## 3. 硬件清单与接线

| 硬件 | 接口 / 引脚 | 说明 |
|------|------|------|
| RDK X5（主控） | — | 全局 |
| PCA9685 PWM 板 | I2C 总线 5，地址 `0x40`，OE→GND | 16 路 PWM 核心驱动 |
| 4× 轮式电机 | 方向：PCA CH0–7；速度：sysfs PWM0/PWM1（物理脚 32/31） | 四轮差速 |
| 4× 关节舵机 | PCA CH8–11（左前/右前/左后/右后） | 姿态切换 |
| 抓取舵机（2 路） | PCA CH14/15 | 夹爪 |
| 360° 旋转夹爪 | 预留 `Driver/1.py`，CH15 | 见"已知问题" |
| ASRPRO 语音模块 | UART `/dev/ttyS1`，115200 | 语音指令输入 |
| HMI 串口触摸屏 | UART `/dev/ttyUSB0`，115200，GBK | 触摸控制与状态显示 |
| LoRa 数传模块 | UART，9600，8N1 | 与指挥中心通信 |
| 摄像头 | **未实装**；`Control/camera.py` 为空，`main.py` 用 `CameraSimulator` 模拟 | 待实现 |

---

## 4. 模块说明

### 4.1 `main.py` — 入口与编排
- `class CameraSimulator`：FPS 模拟器，生成 `terrain/obstacles/slope/survivors` 等感知字典；`get_real_image()` 预留真实摄像头接口（当前 `NotImplementedError`）。
- `class RescueRobot`：系统主类。
  - `initialize()` 依次初始化：PCA9685 → BodyControl → DeepSeek → LoRa → 语音(ASRVoiceSerial) → HMI(RobotHMI) → 摄像头模拟。任一模块初始化失败会降级为"模拟模式"（置 `None` + ⚠️ 警告），便于无硬件调试。
  - `main_loop()`：每 0.2s 调用 `_perceive()`（感知）→ `_decide()`（DeepSeek 决策，失败回退 `_local_decision()` 纯规则）→ `_execute()`（经 ACTION_MAP 执行）→ `_communicate()`（LoRa 发送状态 JSON）。
  - 监听线程：`_voice_listener` / `_hmi_listener` 守护线程将语音/HMI 输入送入 `execute_action()`。

### 4.2 `App/ds_api.py` — DeepSeek 决策
- `class DeepSeek`，基于 `openai.OpenAI` 客户端，端点 `https://api.deepseek.com/v1`，模型 `deepseek-chat`。
- `get_reply(user_text, custom_system_prompt=None)`：核心接口，返回 `choices[0].message.content.strip()`；异常时返回 `"大模型调用失败：..."`。
- `set_default_prompt(new_prompt)`：修改全局系统提示词（系统提示词设定为"地震救援机器人决策助手"，输出限定 `forward/backward/turn_left/turn_right/stop/lie_down/lean/grab`）。
- ⚠️ 该文件 `__main__` 段引用了不存在的 `DeepSeekClient`，直接运行会报错；应仅作为模块 import。

### 4.3 `Control/arm_wheel.py` — 姿态控制
- `class BodyControl`，组合 `PCA9685` + `ServoController` + `CarMotor`。
- 关节舵机通道：`LF_JOINT=8, RF_JOINT=9, LB_JOINT=10, RB_JOINT=11`；抓取通道 `GRAB_CH_A=14, GRAB_CH_B=15`。
- 姿态角度表 `POSTURE_ANGLES`：`upright={20,20,20,30}`、`lean_forward={0,0,60,60}`、`lie_down={30,30,150,150}`。
- `_set_posture(name, transition_time=0.5)`：线性插值平滑过渡。
- 运动方法：`upright_forward/backward`、`lean_forward/backward`、`lie_down_forward/backward`、`upright_left_turn/upright_right_turn`、`grab`、`stop`、`reset`、`get_current_status`。
- `--test` 触发 `run_unit_tests()`（Mock 校验参数范围与占空比输出）。

### 4.4 `Control/camera.py`
- **空文件（占位）**。真实摄像头感知由 `main.py::CameraSimulator` 承担；真实接口在 `CameraSimulator.get_real_image()` 预留。

### 4.5 `Driver/pca9685.py` — PCA9685 PWM 底层驱动
- `class PCA9685`，依赖 `smbus2`，I2C 总线 5，地址 `0x40`，12 位（0~4095）。
- 接口：`set_pwm_freq`(24~1526Hz)、`set_duty_cycle`、`set_raw_pwm`、`get_channel_raw`、`single_channel_zero`、`all_channel_zero`、`close`；支持上下文管理器。
- 工具：`check_i2c_permission`、`validate_i2c_dev`。

### 4.6 `Driver/servo.py` — 舵机控制
- `SingleServo`（角度 0–180，脉冲 0.5–2.5ms）+ `ServoController`（50Hz，通道 8–11 输出）。

### 4.7 `Driver/motor.py` — 四轮电机
- `class CarMotor`。方向通道 PCA CH0–7；**速度**经 sysfs 硬件 PWM（`/sys/class/pwm/pwmchip0`，PWM0/PWM1，1kHz，65% 占空比）。
- 方法：`car_stop/car_forward/car_backward/car_left_group_run/car_right_group_run`、`cleanup`（unexport）。
- 依赖 `/sys/class/pwm/pwmchip0` 存在，通常需要 root 权限。

### 4.8 `Driver/lora.py` — LoRa 数传
- `class LoraDevice`，9600/8N1/timeout=0.3。后台接收线程 `_recv_loop()`；`send(text)` UTF-8 发送；`list_all_com()` / `select_serial_port()` 枚举并交互选择端口；`__main__` 为收发 demo。

### 4.9 `Driver/hmi.py` — 串口 HMI 触摸屏
- `class RobotHMI`，默认 `/dev/ttyUSB0`，115200，GBK。指令层 `send_raw_cmd/set_text/switch_page`；页面刷新 `update_main_page/update_gimbal_page/update_arm_page/update_leg_page/update_system_info/update_comm_status`；`parse_recv_line` 支持 `<BTN>/<RESCUE>/<MSG>`；`register_action(name, func)` + `listen_loop()` 阻塞监听并自动执行已注册动作。

### 4.10 `Driver/voice.py` — ASRPRO 语音模块（main.py 实际使用的 ASR 通道）
- `class ASRVoiceSerial`，默认 `/dev/ttyS1`，115200。
- 协议：`<READY>` / `<ASR>id,text`；`ACTION_MAP`(snid→动作名)、`PLAY_MAP`(snid→音频编号)；`play_audio(id)` 发 `<PLAY>{id}\n`。
- `listen(callback)`：循环读串口，命中 asr 时回调 `(snid, text, action_name)` 并自动播报对应音频。含 `MockSerial` 与 `test_with_mock_serial()`，便于无硬件测试。

### 4.11 `Driver/1.py` — 360° 旋转夹爪（`Gripper360`，占位测试模块）
- 独立测试模块，基于脉宽控制 `stop/open/close`。⚠️ 调用了 `pca.set_pwm_us(...)`，而当前 `pca9685.py` 无此方法，运行会 `AttributeError`。需补 `set_pwm_us` 或改用 `set_raw_pwm`。

### 4.12 `rdkx5_asr_uart.py` — 独立 ASR 中控 demo（"小娅机器人"）
- **不被 main.py 引用**，是早期/并行开发的独立版 ASR 演示：串口 `/dev/ttyS1`，115200，`ASR_COMMANDS` 字典、`XiaoyaASRController` 中控（先播报再执行动作）。协议与 `Driver/voice.py` 等价。

---

## 5. 主流程（main.py）

```
initialize()
  ├─ PCA9685(bus_num=5, addr=0x40)        # 失败→模拟模式
  ├─ BodyControl(bus_num=5, addr=0x40)
  ├─ DeepSeek(api_key, base_url, sys_prompt)
  ├─ LoraDevice(port)                     # 未配置则交互选端口
  ├─ ASRVoiceSerial(/dev/ttyS1, 115200)   # play_audio(20101) 开机播报
  ├─ RobotHMI(/dev/ttyUSB0, 115200)       # init_all_ui + register_action ×16
  └─ CameraSimulator(fps=10)
start() → main_loop()  # 0.2s 周期
  ├─ _perceive()      # 模拟帧 scene_analysis + survivors
  ├─ _decide()        # DeepSeek 决策 / 回退 _local_decision 规则
  ├─ _execute()       # ACTION_MAP → BodyControl 方法 + 刷新 HMI
  └─ _communicate()   # 状态 JSON 经 LoRa 发送
voice/hmi 监听线程 → execute_action() → _execute()
```

配置默认值在 `main()` 的 `config` 字典：`pca_bus=5`、`pca_addr=0x40`、`lora_port=None`、`voice_port="/dev/ttyS1"`、`hmi_port="/dev/ttyUSB0"`、`camera_fps=10`、`ds_api_key`、`ds_base_url="https://api.deepseek.com/v1"`。

---

## 6. ASR 语音交互

两套实现：
1. **`Driver/voice.py`（main.py 集成）**：`ASRVoiceSerial`，协议 `<ASR>id,text`，命中后自动 `play_by_snid` 播报对应音频并回调执行动作。
2. **`rdkx5_asr_uart.py`（独立 demo）**：`XiaoyaASRController`，先播报 `<PLAY>id` 再延迟执行动作桩（打印 TODO）。

ASRPRO 端需 `Serial.begin(115200)` 且以 `\n` 结尾发送文本（`readStringUntil('\n')`）。

---

## 7. DeepSeek 决策

- 模块：`App/ds_api.py: DeepSeek`，OpenAI 兼容端点 `https://api.deepseek.com/v1`，模型 `deepseek-chat`。
- 主循环 `_decide()` 调用 `get_reply()`，再从回复中匹配动作关键词；大模型不可用时回退 `_local_decision()` 纯规则：不可通行→`turn_left`、障碍→`turn_right`、窄道/坡→`lean`、楼梯→`lie_down`、否则 `forward`。
- 系统提示词约束输出为 8 个固定动作之一。

> ⚠️ **安全提醒（重要）**：DeepSeek API Key 目前以**明文硬编码**形式同时出现在 `main.py` 的 `config` 与 `ds_api.py` 的构造参数默认值中。请立即将其移出源码，改为从环境变量读取（如 `os.getenv("DEEPSEEK_API_KEY")`）或放入不被提交的配置文件 / 密钥管理服务，**切勿将含密钥的代码提交到仓库**。

---

## 8. 安装与运行

**依赖安装**（仓库暂无 `requirements.txt`，建议补充）：
```bash
pip install openai smbus2 pyserial
```

**硬件前置检查**：
```bash
i2cdetect -y 5          # 应出现 0x40 (PCA9685)
ls /sys/class/pwm/      # 应有 pwmchip0（电机速度 PWM）
ls /dev/ttyS1 /dev/ttyUSB0   # 语音 / HMI 串口存在
```
I2C 需用户在 `i2c` 组或 `sudo`；PWM/GPIO 通常需 root。

**运行主系统**：
```bash
# 在 RDK X5 上，已接好 PCA9685、ASRPRO、HMI、LoRa
cd /path/to/qs
python3 main.py          # Ctrl+C 优雅停止并清理资源
```

**运行独立 ASR demo**：
```bash
python3 rdkx5_asr_uart.py   # 需 ASRPRO 接 ttyS1
```

**单模块自测（各自 `__main__`）**：
```bash
python3 Driver/pca9685.py [--debug]
python3 Driver/motor.py [--debug]
python3 Driver/voice.py            # 串口失败自动转 MockSerial 测试
python3 Control/arm_wheel.py [--debug] | --test
python3 Driver/lora.py             # 交互式收发
python3 Driver/1.py                # ⚠️ 需先补 PCA9685.set_pwm_us
```

---

## 9. 测试说明

| 文件 | 状态 | 说明 |
|------|------|------|
| `test/cmara_wheel_test.py` | 空（占位） | 摄像头+轮子测试，未实现 |
| `test/ds_stage.py` | 空（占位） | DeepSeek 测试，未实现 |
| `test/hmi_all_test.py` | 空（占位） | HMI 全功能测试，未实现 |
| `test/motor_test.py` | 历史遗留、不可运行 | 引用了不存在的 `MotorController/MotorPWMError/MotorGPIOError`；但其中的编码器 GPIO 引脚表（BCM 17/27/22/23/5/6/26/16）有参考价值 |
| `test/voice_all_test.py` | 空（占位） | 语音全功能测试，未实现 |

> 实际可用的测试入口分散在各模块自身的 `__main__` 与 `Driver/voice.py::test_with_mock_serial`、`Control/arm_wheel.py::run_unit_tests()`。

---

## 10. 已知问题 / TODO

1. **API Key 硬编码**（高优先级）：见第 7 节，务必移出源码。
2. **`Driver/1.py` 不兼容**：调用了 `pca.set_pwm_us(...)`，需补该方法或改用 `set_raw_pwm`。
3. **`test/motor_test.py` 不可运行**：引用的符号已不存在，且电机速度实际由 sysfs PWM 实现，与其中描述不符。
4. **`ds_api.py::__main__` 引用 `DeepSeekClient`**：仅应作为模块 import。
5. **摄像头未实装**：`Control/camera.py` 为空，`main.py` 仅用 `CameraSimulator` 模拟；真实人物识别/路况识别待实现。
6. **TECHNICAL_DOC.md 与代码偏差**：文档称 `main.py` 为"待实现"（实际已完成）；文档未涵盖 `voice.py/hmi.py/camera.py/rdkx5_asr_uart.py/1.py` 及 `test/`；文档电机速度写"0–100% 占空比"，实际为 sysfs 固定 65% PWM，PCA9685 仅负责方向。

---

## 11. 文档索引

| 文档 | 内容 |
|------|------|
| `TECHNICAL_DOC.md` | v1.0 技术文档：系统概述、模块接口表、核心业务流程、LoRa JSON 格式、未完成功能、硬件连接表（部分过时） |

> 本 `README_cn.md` 以**代码为准**编写；涉及与 `TECHNICAL_DOC.md` 冲突处以代码为准，并已在第 10 节标注偏差。
