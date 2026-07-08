#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera_servo_node.py
SG90 舵机云台控制节点（Pan 单轴）
- 支持 I2C PCA9685 硬件驱动与模拟模式
- 手动遥控：订阅 /rescue_robot/cmd_camera_pan
- 自动跟踪：订阅 /rescue_robot/body_detections，将最大人体框居中
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from vision_msgs.msg import Detection2DArray
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
import math
import time
from typing import Optional

from rescue_robot.utils import clamp


# PCA9685 寄存器定义
PCA9685_MODE1 = 0x00
PCA9685_PRESCALE = 0xFE
PCA9685_LED0_ON_L = 0x06
PCA9685_LED0_OFF_L = 0x08
MODE1_RESTART = 0x80
MODE1_SLEEP = 0x10
MODE1_AI = 0x20
MODE1_ALLCALL = 0x01


class PCA9685Controller:
    """PCA9685 I2C PWM 驱动器封装"""

    def __init__(self, bus: int, address: int, frequency: int = 50, logger=None):
        self.address = address
        self.logger = logger
        try:
            import smbus2
            self.bus = smbus2.SMBus(bus)
        except ImportError:
            raise RuntimeError('未安装 smbus2，请执行: pip install smbus2')
        except Exception as e:
            raise RuntimeError(f'无法打开 I2C 总线 {bus}: {e}')

        self._init_chip(frequency)

    def _init_chip(self, frequency: int):
        """初始化 PCA9685：复位、设置频率、退出睡眠"""
        prescale = int(round(25000000.0 / (4096.0 * frequency)) - 1)
        prescale = clamp(prescale, 3, 255)

        old_mode = self.bus.read_byte_data(self.address, PCA9685_MODE1)
        # 进入睡眠模式才能设置频率
        self.bus.write_byte_data(self.address, PCA9685_MODE1, (old_mode & 0x7F) | MODE1_SLEEP)
        self.bus.write_byte_data(self.address, PCA9685_PRESCALE, prescale)
        self.bus.write_byte_data(self.address, PCA9685_MODE1, old_mode)
        time.sleep(0.005)
        # 启动：自动递增 + 正常模式
        self.bus.write_byte_data(self.address, PCA9685_MODE1, old_mode | MODE1_AI | MODE1_ALLCALL)
        time.sleep(0.005)

        if self.logger:
            self.logger.info(f'PCA9685 初始化完成: 总线地址=0x{self.address:02X}, 频率={frequency}Hz, prescale={prescale}')

    def set_pwm(self, channel: int, on: int, off: int):
        """设置指定通道的 PWM 占空比"""
        reg_base = PCA9685_LED0_ON_L + 4 * channel
        data = [
            on & 0xFF,
            (on >> 8) & 0x0F,
            off & 0xFF,
            (off >> 8) & 0x0F,
        ]
        for i, byte in enumerate(data):
            self.bus.write_byte_data(self.address, reg_base + i, byte)

    def set_servo_angle(self, channel: int, pulse_us: int):
        """将脉宽（微秒）转换为 PWM 值并输出"""
        # 50Hz 下，4096 对应 20000us
        off = int(pulse_us * 4096 / 20000)
        off = clamp(off, 0, 4095)
        self.set_pwm(channel, 0, off)


class CameraServoNode(Node):
    def __init__(self):
        super().__init__('camera_servo_node')

        self._declare_params()
        self._load_params()

        self.controller: Optional[PCA9685Controller] = None
        if not self.simulation:
            try:
                self.controller = PCA9685Controller(
                    self.i2c_bus, self.i2c_address, self.frequency, self.get_logger()
                )
            except Exception as e:
                self.get_logger().error(f'PCA9685 初始化失败，进入模拟模式: {e}')
                self.simulation = True

        self.current_pan = self.pan_initial_angle
        self.target_pan = self.pan_initial_angle
        self.manual_override_until = 0.0
        self._last_track_time = 0.0

        # 手动遥控
        self.cmd_sub = self.create_subscription(
            Float64, '/rescue_robot/cmd_camera_pan', self.cmd_pan_callback, 10
        )
        # 自动跟踪输入
        self.det_sub = self.create_subscription(
            Detection2DArray, '/rescue_robot/body_detections', self.det_callback, 10
        )
        # 当前角度状态
        self.state_pub = self.create_publisher(Float64, '/rescue_robot/camera_pan_angle', 10)

        # 控制循环 20Hz
        self.timer = self.create_timer(0.05, self.control_loop)

        # 初始位置
        self._write_servo(self.current_pan)

        self.add_on_set_parameters_callback(self._on_parameter_change)
        self.get_logger().info(
            f'舵机节点已启动: simulation={self.simulation}, '
            f'pan_initial={self.pan_initial_angle}, max_speed={self.max_speed}deg/s'
        )

    def _declare_params(self):
        self.declare_parameter('servo.pan_channel', 0)
        self.declare_parameter('servo.i2c_bus', 5)
        self.declare_parameter('servo.i2c_address', 0x40)
        self.declare_parameter('servo.frequency', 50)
        self.declare_parameter('servo.pan_min_angle', -90.0)
        self.declare_parameter('servo.pan_max_angle', 90.0)
        self.declare_parameter('servo.pan_initial_angle', 0.0)
        self.declare_parameter('servo.simulation', False)
        self.declare_parameter('servo.max_speed', 45.0)
        self.declare_parameter('servo.pulse_min_us', 500)
        self.declare_parameter('servo.pulse_max_us', 2500)

        self.declare_parameter('auto_track.enabled', True)
        self.declare_parameter('auto_track.image_width', 640)
        self.declare_parameter('auto_track.hfov_deg', 60.0)
        self.declare_parameter('auto_track.kp', 0.8)
        self.declare_parameter('auto_track.manual_override_sec', 2.0)

    def _load_params(self):
        self.pan_channel = self.get_parameter('servo.pan_channel').value
        self.i2c_bus = self.get_parameter('servo.i2c_bus').value
        self.i2c_address = self.get_parameter('servo.i2c_address').value
        self.frequency = self.get_parameter('servo.frequency').value
        self.pan_min_angle = self.get_parameter('servo.pan_min_angle').value
        self.pan_max_angle = self.get_parameter('servo.pan_max_angle').value
        self.pan_initial_angle = self.get_parameter('servo.pan_initial_angle').value
        self.simulation = self.get_parameter('servo.simulation').value
        self.max_speed = self.get_parameter('servo.max_speed').value
        self.pulse_min_us = self.get_parameter('servo.pulse_min_us').value
        self.pulse_max_us = self.get_parameter('servo.pulse_max_us').value

        self.auto_track_enabled = self.get_parameter('auto_track.enabled').value
        self.image_width = self.get_parameter('auto_track.image_width').value
        self.hfov_deg = self.get_parameter('auto_track.hfov_deg').value
        self.kp = self.get_parameter('auto_track.kp').value
        self.manual_override_sec = self.get_parameter('auto_track.manual_override_sec').value

    def _angle_to_pulse_us(self, angle_deg: float) -> int:
        """将角度（-90~+90）映射为脉宽微秒"""
        ratio = (angle_deg - self.pan_min_angle) / (self.pan_max_angle - self.pan_min_angle)
        ratio = clamp(ratio, 0.0, 1.0)
        pulse = self.pulse_min_us + ratio * (self.pulse_max_us - self.pulse_min_us)
        return int(pulse)

    def _write_servo(self, angle_deg: float):
        """向舵机写入目标角度"""
        angle_deg = clamp(angle_deg, self.pan_min_angle, self.pan_max_angle)
        pulse = self._angle_to_pulse_us(angle_deg)
        if self.controller is not None:
            try:
                self.controller.set_servo_angle(self.pan_channel, pulse)
            except Exception as e:
                self.get_logger().error(f'舵机写入失败: {e}', throttle_duration_sec=5.0)
        elif self.simulation:
            self.get_logger().debug(f'[模拟] 舵机角度: {angle_deg:.1f}°, PWM脉宽: {pulse}us')

    def cmd_pan_callback(self, msg: Float64):
        """手动遥控：直接设置目标角度，并激活手动覆盖期"""
        self.target_pan = clamp(msg.data, self.pan_min_angle, self.pan_max_angle)
        now = self.get_clock().now().nanoseconds / 1e9
        self.manual_override_until = now + self.manual_override_sec
        self.get_logger().info(f'手动指令: pan={self.target_pan:.1f}°, 覆盖期={self.manual_override_sec}s')

    def det_callback(self, msg: Detection2DArray):
        """自动跟踪：选择最大的人体框，计算水平偏移并生成目标角度"""
        if not self.auto_track_enabled:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now < self.manual_override_until:
            return
        if not msg.detections:
            return

        # 选择面积最大的检测框
        best = max(msg.detections, key=lambda d: d.bbox.size_x * d.bbox.size_y)
        cx = best.bbox.center.position.x

        # 计算像素偏移（正 = 偏右，负 = 偏左）
        offset_x = cx - self.image_width / 2.0

        # 像素偏移 -> 角度增量
        angle_per_pixel = self.hfov_deg / self.image_width
        delta = -offset_x * angle_per_pixel * self.kp  # 负号：画面右偏需要左转

        new_target = clamp(self.current_pan + delta, self.pan_min_angle, self.pan_max_angle)

        if abs(new_target - self.target_pan) > 0.5:
            self.target_pan = new_target
            self._last_track_time = now
            self.get_logger().debug(
                f'自动跟踪: offset_x={offset_x:.1f}px, delta={delta:.1f}°, target={self.target_pan:.1f}°'
            )

    def control_loop(self):
        """控制循环：以限制角速度逐步逼近目标角度"""
        if abs(self.target_pan - self.current_pan) < 0.1:
            return

        dt = 0.05  # 定时器周期
        max_delta = self.max_speed * dt

        error = self.target_pan - self.current_pan
        step = clamp(error, -max_delta, max_delta)
        self.current_pan += step

        self._write_servo(self.current_pan)

        # 发布当前角度
        self.state_pub.publish(Float64(data=self.current_pan))

    def _on_parameter_change(self, params):
        unsupported = []
        for param in params:
            name = param.name
            value = param.value
            if name == 'servo.pan_min_angle':
                self.pan_min_angle = float(value)
            elif name == 'servo.pan_max_angle':
                self.pan_max_angle = float(value)
            elif name == 'servo.max_speed':
                self.max_speed = float(value)
            elif name == 'servo.pulse_min_us':
                self.pulse_min_us = int(value)
            elif name == 'servo.pulse_max_us':
                self.pulse_max_us = int(value)
            elif name == 'auto_track.enabled':
                self.auto_track_enabled = bool(value)
            elif name == 'auto_track.kp':
                self.kp = float(value)
            elif name == 'auto_track.manual_override_sec':
                self.manual_override_sec = float(value)
            else:
                unsupported.append(name)
        if unsupported:
            return SetParametersResult(successful=False, reason=f'不支持的参数: {", ".join(unsupported)}')
        return SetParametersResult(successful=True)

    def destroy_node(self):
        if self.timer is not None:
            self.timer.cancel()
        # 归位
        if self.controller is not None:
            try:
                self._write_servo(self.pan_initial_angle)
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
