#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import threading

current_dir = os.path.dirname(os.path.abspath(__file__))
driver_dir = os.path.join(os.path.dirname(current_dir), 'Driver')
if driver_dir not in sys.path:
    sys.path.insert(0, driver_dir)

from pca9685 import PCA9685, PCA9685InitError, PCA9685CommError
from servo import ServoController, SingleServo, ServoError
from motor import CarMotor


class BodyControlError(Exception):
    pass


class BodyControlInitError(BodyControlError):
    pass


class BodyControl:
    """
    机器人姿态控制类，集成舵机控制和电机控制功能
    
    硬件配置：
    - 舵机：4个关节舵机，使用PCA9685通道8~11
    - 电机：4个轮式电机，使用PCA9685通道0~7
    
    关节定义：
    - 左前关节 (LF_JOINT)：通道8
    - 右前关节 (RF_JOINT)：通道9
    - 左后关节 (LB_JOINT)：通道10
    - 右后关节 (RB_JOINT)：通道11
    """
    
    # ===================== 舵机通道配置 =====================
    LF_JOINT = 8    # 左前关节
    RF_JOINT = 9    # 右前关节
    LB_JOINT = 10   # 左后关节
    RB_JOINT = 11   # 右后关节
    
    ALL_JOINTS = [LF_JOINT, RF_JOINT, LB_JOINT, RB_JOINT]
    
    # ===================== 机械臂抓取通道配置 =====================
    GRAB_CH_A = 14   # 抓取通道A
    GRAB_CH_B = 15   # 抓取通道B
    
    # ===================== 转向参数配置 =====================
    BASE_TURN_TIME = 2.0  # 基础转向时间（90度）
    
    # ===================== 姿态角度配置 =====================
    POSTURE_ANGLES = {
        'upright': {
            'name': '正身姿态',
            'description': '所有关节保持90度，机器人直立',
            'angles': {
                LF_JOINT: 20,
                RF_JOINT: 20,
                LB_JOINT: 20,
                RB_JOINT: 30
            }
        },
        'lean_forward': {
            'name': '附身姿态',
            'description': '前后关节前倾，降低重心',
            'angles': {
                LF_JOINT: 0,
                RF_JOINT: 0,
                LB_JOINT: 60,
                RB_JOINT: 60
            }
        },
        'lie_down': {
            'name': '趴下姿态',
            'description': '所有关节平放，贴近地面',
            'angles': {
                LF_JOINT: 30,
                RF_JOINT: 30,
                LB_JOINT: 150,
                RB_JOINT: 150
            }
        }
    }
    
    # ===================== 速度配置 =====================
    DEFAULT_SPEED = 0.65
    MIN_SPEED = 0.2
    MAX_SPEED = 1.0
    
    def __init__(self, bus_num=5, addr=0x40, debug=False):
        """
        初始化姿态控制器
        
        :param bus_num: I2C总线编号，默认5
        :param addr: PCA9685设备地址，默认0x40
        :param debug: 是否开启调试模式
        :raises BodyControlInitError: 初始化失败时抛出
        """
        self.bus_num = bus_num
        self.addr = addr
        self.debug = debug
        
        self._pca = None
        self._servo_controller = None
        self._motor_controller = None
        self._servos = {}
        self._current_posture = None
        self._current_direction = None
        self._is_running = False
        
        self._init_hardware()
    
    def _init_hardware(self):
        """初始化PCA9685、舵机控制器和电机控制器"""
        try:
            print("[BodyControl] 初始化PCA9685...")
            self._pca = PCA9685(bus_num=self.bus_num, addr=self.addr, debug=self.debug)
            self._pca.set_pwm_freq(50)
            
            print("[BodyControl] 初始化舵机控制器...")
            self._servo_controller = ServoController(pca_driver=self._pca, debug=self.debug)
            
            print("[BodyControl] 初始化电机控制器...")
            self._motor_controller = CarMotor(pca_driver=self._pca, debug=self.debug)
            
            print("[BodyControl] 添加关节舵机...")
            self._servos['lf_joint'] = self._servo_controller.add_servo('左前关节', self.LF_JOINT)
            self._servos['rf_joint'] = self._servo_controller.add_servo('右前关节', self.RF_JOINT)
            self._servos['lb_joint'] = self._servo_controller.add_servo('左后关节', self.LB_JOINT)
            self._servos['rb_joint'] = self._servo_controller.add_servo('右后关节', self.RB_JOINT)
            
            self._set_initial_posture()
            
            print("[BodyControl] 硬件初始化完成")
            
        except PCA9685InitError as e:
            raise BodyControlInitError(f"PCA9685初始化失败: {e}")
        except ServoError as e:
            raise BodyControlInitError(f"舵机控制器初始化失败: {e}")
        except Exception as e:
            raise BodyControlInitError(f"硬件初始化失败: {e}")
    
    def _set_initial_posture(self):
        """设置初始姿态为正身姿态"""
        self._set_posture('upright')
        self._motor_controller.car_stop()
        self._current_posture = 'upright'
        self._current_direction = 'stop'
    
    def _set_posture(self, posture_name, transition_time=0.5):
        """
        设置机器人姿态，实现平滑过渡
        
        :param posture_name: 姿态名称，可选值: 'upright', 'lean_forward', 'lie_down'
        :param transition_time: 过渡时间（秒），默认0.5秒
        """
        if posture_name not in self.POSTURE_ANGLES:
            raise ValueError(f"未知姿态: {posture_name}")
        target_angles = self.POSTURE_ANGLES[posture_name]['angles']
        current_angles = {}
        servo_map = {}
        for ch in self.ALL_JOINTS:
            s_name = self._get_servo_name_by_channel(ch)
            servo = self._servos[s_name]
            current_angles[ch] = servo.get_angle()
            servo_map[ch] = servo

        # 降低步频至20步每秒，减少I2C刷屏
        steps = max(8, int(transition_time * 20))
        step_time = transition_time / steps

        for step in range(steps + 1):
            progress = step / steps
            # 先批量计算所有目标角度，不立即写入
            temp_target = {}
            for ch in self.ALL_JOINTS:
                start = current_angles[ch]
                end = target_angles[ch]
                temp_target[ch] = start + (end - start) * progress
            # 批量同步更新四路舵机
            for ch, ang in temp_target.items():
                servo_map[ch].set_angle(ang)
            time.sleep(step_time)

        self._current_posture = posture_name
        if self.debug:
            posture_info = self.POSTURE_ANGLES[posture_name]
            print(f"[BodyControl] 姿态切换完成: {posture_info['name']}")
    
    def _get_servo_name_by_channel(self, channel):
        """根据通道号获取舵机名称"""
        channel_map = {
            self.LF_JOINT: 'lf_joint',
            self.RF_JOINT: 'rf_joint',
            self.LB_JOINT: 'lb_joint',
            self.RB_JOINT: 'rb_joint'
        }
        return channel_map.get(channel)
    
    def _set_motor_direction(self, direction):
        """
        设置电机方向
        
        :param direction: 方向，可选值: 'forward', 'backward', 'stop'
        """
        if direction == 'forward':
            self._motor_controller.car_forward()
        elif direction == 'backward':
            self._motor_controller.car_backward()
        elif direction == 'stop':
            self._motor_controller.car_stop()
        else:
            raise ValueError(f"未知方向: {direction}")
        
        self._current_direction = direction
        
        if self.debug:
            print(f"[BodyControl] 电机方向: {direction}")
    
    def get_current_status(self):
        """
        获取当前状态信息
        
        :return: 包含姿态和方向信息的字典
        """
        posture_info = self.POSTURE_ANGLES.get(self._current_posture, {})
        
        return {
            'posture': self._current_posture,
            'posture_name': posture_info.get('name', '未知'),
            'direction': self._current_direction,
            'is_running': self._is_running,
            'joint_angles': {
                'lf_joint': self._servos['lf_joint'].get_angle(),
                'rf_joint': self._servos['rf_joint'].get_angle(),
                'lb_joint': self._servos['lb_joint'].get_angle(),
                'rb_joint': self._servos['rb_joint'].get_angle()
            }
        }
    
    def print_status(self):
        """打印当前状态"""
        status = self.get_current_status()
        print("\n" + "=" * 50)
        print("机器人当前状态")
        print("=" * 50)
        print(f"姿态: {status['posture_name']} ({status['posture']})")
        print(f"方向: {status['direction']}")
        print(f"运行状态: {'运行中' if status['is_running'] else '停止'}")
        print("\n关节角度:")
        for name, angle in status['joint_angles'].items():
            print(f"  {name}: {angle}°")
        print("=" * 50 + "\n")
    
    # ===================== 姿态控制方法 =====================
    
    def upright_forward(self, transition_time=0.5):
        """
        正身向前：控制4个关节舵机同时保持90度角度，同时控制电机保持向前转动状态
        
        :param transition_time: 姿态过渡时间（秒），默认0.5秒
        """
        self._is_running = True
        print("[BodyControl] 执行: 正身向前")
        
        try:
            self._set_posture('upright', transition_time)
            time.sleep(0.2)
            self._set_motor_direction('forward')
            
            if self.debug:
                self.print_status()
            
        except Exception as e:
            self._is_running = False
            raise BodyControlError(f"正身向前执行失败: {e}")
    
    def upright_backward(self, transition_time=0.5):
        """
        正身向后：控制4个关节舵机同时保持90度角度，同时控制电机保持向后转动状态
        
        :param transition_time: 姿态过渡时间（秒），默认0.5秒
        """
        self._is_running = True
        print("[BodyControl] 执行: 正身向后")
        
        try:
            self._set_posture('upright', transition_time)
            time.sleep(0.2)
            self._set_motor_direction('backward')
            
            if self.debug:
                self.print_status()
            
        except Exception as e:
            self._is_running = False
            raise BodyControlError(f"正身向后执行失败: {e}")
    
    def lean_forward(self, transition_time=0.5):
        """
        附身向前：控制4个关节舵机前倾，左前/右前60度，左后/右后120度，同时控制电机保持向前转动状态
        
        :param transition_time: 姿态过渡时间（秒），默认0.5秒
        """
        self._is_running = True
        print("[BodyControl] 执行: 附身向前")
        
        try:
            self._set_posture('lean_forward', transition_time)
            time.sleep(0.2)
            self._set_motor_direction('forward')
            
            if self.debug:
                self.print_status()
            
        except Exception as e:
            self._is_running = False
            raise BodyControlError(f"附身向前执行失败: {e}")
    
    def lean_backward(self, transition_time=0.5):
        """
        附身向后：控制4个关节舵机前倾，左前/右前60度，左后/右后120度，同时控制电机保持向后转动状态
        
        :param transition_time: 姿态过渡时间（秒），默认0.5秒
        """
        self._is_running = True
        print("[BodyControl] 执行: 附身向后")
        
        try:
            self._set_posture('lean_forward', transition_time)
            time.sleep(0.2)
            self._set_motor_direction('backward')
            
            if self.debug:
                self.print_status()
            
        except Exception as e:
            self._is_running = False
            raise BodyControlError(f"附身向后执行失败: {e}")
    
    def lie_down_forward(self, transition_time=0.5):
        """
        趴下向前：控制4个关节舵机平放，左前/右前30度，左后/右后150度，同时控制电机保持向前转动状态
        
        :param transition_time: 姿态过渡时间（秒），默认0.5秒
        """
        self._is_running = True
        print("[BodyControl] 执行: 趴下向前")
        
        try:
            self._set_posture('lie_down', transition_time)
            time.sleep(0.2)
            self._set_motor_direction('forward')
            
            if self.debug:
                self.print_status()
            
        except Exception as e:
            self._is_running = False
            raise BodyControlError(f"趴下向前执行失败: {e}")
    
    def lie_down_backward(self, transition_time=0.5):
        """
        趴下向后：控制4个关节舵机平放，左前/右前30度，左后/右后150度，同时控制电机保持向后转动状态
        
        :param transition_time: 姿态过渡时间（秒），默认0.5秒
        """
        self._is_running = True
        print("[BodyControl] 执行: 趴下向后")
        
        try:
            self._set_posture('lie_down', transition_time)
            time.sleep(0.2)
            self._set_motor_direction('backward')
            
            if self.debug:
                self.print_status()
            
        except Exception as e:
            self._is_running = False
            raise BodyControlError(f"趴下向后执行失败: {e}")
    
    def upright_left_turn(self, angle=90, speed=0.65):
        """
        正身左转：控制设备正身向左转动指定角度
        
        :param angle: 转动角度（度），范围 0~360，默认90度
        :param speed: 转动速度，范围 0.2~1.0，默认0.65
        :raises ValueError: 参数超出范围时抛出
        :raises BodyControlError: 执行失败时抛出
        """
        if not 0 <= angle <= 360:
            raise ValueError("转动角度范围 0~360 度")
        if not self.MIN_SPEED <= speed <= self.MAX_SPEED:
            raise ValueError(f"速度范围 {self.MIN_SPEED}~{self.MAX_SPEED}")
        
        self._is_running = True
        print(f"[BodyControl] 执行: 正身左转 {angle}°，速度 {speed}")
        
        try:
            self._set_posture('upright', transition_time=0.3)
            self._motor_controller.car_left_group_run()
            time.sleep(0.2)
            
            if self.debug:
                self.print_status()
                
        except Exception as e:
            self._is_running = False
            self._set_motor_direction('stop')
            raise BodyControlError(f"正身左转执行失败: {e}")
    
    def upright_right_turn(self, angle=90, speed=0.65):
        """
        正身右转：控制设备正身向右转动指定角度
        
        :param angle: 转动角度（度），范围 0~360，默认90度
        :param speed: 转动速度，范围 0.2~1.0，默认0.65
        :raises ValueError: 参数超出范围时抛出
        :raises BodyControlError: 执行失败时抛出
        """
        if not 0 <= angle <= 360:
            raise ValueError("转动角度范围 0~360 度")
        if not self.MIN_SPEED <= speed <= self.MAX_SPEED:
            raise ValueError(f"速度范围 {self.MIN_SPEED}~{self.MAX_SPEED}")
        
        self._is_running = True
        print(f"[BodyControl] 执行: 正身右转 {angle}°，速度 {speed}")
        
        try:
            self._set_posture('upright', transition_time=0.3)
            self._motor_controller.car_right_group_run()
            time.sleep(0.2)
            
            if self.debug:
                self.print_status()
                
        except Exception as e:
            self._is_running = False
            self._set_motor_direction('stop')
            raise BodyControlError(f"正身右转执行失败: {e}")
    
    def grab(self, angle_a=90, angle_b=90):
        """
        设置两个抓取通道的角度占空比
        
        :param angle_a: 通道14的角度（度），范围0~180，默认90度
        :param angle_b: 通道15的角度（度），范围0~180，默认90度
        :raises ValueError: 参数超出范围时抛出
        :raises BodyControlError: 执行失败时抛出
        """
        if not 0 <= angle_a <= 180:
            raise ValueError("角度范围 0~180 度")
        if not 0 <= angle_b <= 180:
            raise ValueError("角度范围 0~180 度")
        
        self._is_running = True
        print(f"[BodyControl] 设置抓取通道角度: CH14={angle_a}°, CH15={angle_b}°")
        
        try:
            servo_min_pulse = 0.5
            servo_max_pulse = 2.5
            servo_period = 20.0
            
            ratio_a = angle_a / 180.0
            pulse_width_a = servo_min_pulse + ratio_a * (servo_max_pulse - servo_min_pulse)
            duty_a = pulse_width_a / servo_period
            
            ratio_b = angle_b / 180.0
            pulse_width_b = servo_min_pulse + ratio_b * (servo_max_pulse - servo_min_pulse)
            duty_b = pulse_width_b / servo_period
            
            self._pca.set_duty_cycle(self.GRAB_CH_A, duty_a)
            self._pca.set_duty_cycle(self.GRAB_CH_B, duty_b)
            
            if self.debug:
                ch14_on, ch14_off = self._pca.get_channel_raw(self.GRAB_CH_A)
                ch15_on, ch15_off = self._pca.get_channel_raw(self.GRAB_CH_B)
                print(f"[BodyControl] CH14: ON={ch14_on}, OFF={ch14_off}, 占空比={duty_a:.4f}")
                print(f"[BodyControl] CH15: ON={ch15_on}, OFF={ch15_off}, 占空比={duty_b:.4f}")
            
            print("[BodyControl] 抓取通道角度设置完成")
            self._is_running = False
            
        except Exception as e:
            self._is_running = False
            raise BodyControlError(f"抓取通道角度设置失败: {e}")
    
    def stop(self):
        """
        停止所有动作：电机停止，保持当前姿态
        """
        print("[BodyControl] 执行: 停止")
        self._is_running = False
        self._set_motor_direction('stop')
        
        if self.debug:
            self.print_status()
    
    def reset(self, transition_time=0.5):
        """
        重置到初始状态：正身姿态，电机停止
        
        :param transition_time: 姿态过渡时间（秒），默认0.5秒
        """
        print("[BodyControl] 执行: 重置")
        self._is_running = False
        self._set_posture('upright', transition_time)
        time.sleep(0.2)
        self._set_motor_direction('stop')
        
        if self.debug:
            self.print_status()
    
    def cleanup(self):
        """
        清理资源：停止所有动作，释放硬件资源
        """
        print("[BodyControl] 清理资源...")
        self._is_running = False
        
        try:
            self._motor_controller.car_stop()
            
            self._servo_controller.disable_all()
            
            if self._pca:
                self._pca.close()
                self._pca = None
            
            print("[BodyControl] 资源清理完成")
            
        except Exception as e:
            print(f"[BodyControl] 清理资源时发生错误: {e}")
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.cleanup()
    
    def __del__(self):
        try:
            if self._pca is not None:
                self.cleanup()
        except Exception:
            pass


def run_unit_tests():
    """
    单元测试函数：验证新增功能的参数校验和边界条件
    
    测试内容：
    1. 正身左转参数校验（角度、速度超出范围）
    2. 正身右转参数校验（角度、速度超出范围）
    3. 抓取功能参数校验（占空比、动作时间超出范围）
    4. 验证PCA9685通道14/15占空比输出正确性
    """
    print("\n" + "=" * 60)
    print("单元测试：参数校验和边界条件验证")
    print("=" * 60)
    
    test_results = []
    
    class MockPCA9685:
        """模拟PCA9685类用于单元测试"""
        def __init__(self, bus_num=5, addr=0x40, debug=False):
            self.bus_num = bus_num
            self.addr = addr
            self.debug = debug
            self._channels = {}
            for ch in range(16):
                self._channels[ch] = {'on': 0, 'off': 0}
        
        def set_pwm_freq(self, freq):
            pass
        
        def set_duty_cycle(self, ch, duty):
            duty = max(0.0, min(1.0, duty))
            raw_val = int(duty * 4095)
            self._channels[ch]['on'] = 0
            self._channels[ch]['off'] = raw_val
        
        def set_raw_pwm(self, ch, on, off):
            self._channels[ch]['on'] = on
            self._channels[ch]['off'] = off
        
        def get_channel_raw(self, ch):
            return self._channels[ch]['on'], self._channels[ch]['off']
        
        def single_channel_zero(self, ch):
            self._channels[ch]['on'] = 0
            self._channels[ch]['off'] = 0
        
        def close(self):
            pass
        
        def __enter__(self):
            return self
        
        def __exit__(self, exc_type, exc_val, exc_tb):
            self.close()
    
    class MockMotorController:
        """模拟电机控制器类用于单元测试"""
        MOTOR_SPEED_DUTY = 650000
        PWM_A_ID = 0
        PWM_B_ID = 1
        
        def __init__(self, pca_driver, debug=False):
            self.pca = pca_driver
            self.debug = debug
            self._pwm_duty = {0: 0, 1: 0}
        
        def _set_pwm_duty(self, pwm_id, duty_ns):
            self._pwm_duty[pwm_id] = duty_ns
        
        def car_stop(self):
            pass
        
        def car_forward(self):
            pass
        
        def car_backward(self):
            pass
        
        def car_left_group_run(self):
            pass
        
        def car_right_group_run(self):
            pass
    
    class MockServoController:
        """模拟舵机控制器类用于单元测试"""
        def __init__(self, pca_driver, debug=False):
            self.pca = pca_driver
        
        def add_servo(self, name, pwm_ch):
            return MockServo(pwm_ch)
        
        def disable_all(self):
            pass
    
    class MockServo:
        """模拟单个舵机类用于单元测试"""
        def __init__(self, channel):
            self._channel = channel
            self._angle = 0
        
        def set_angle(self, angle):
            self._angle = angle
        
        def get_angle(self):
            return self._angle
    
    def test_param_validation():
        """测试参数校验"""
        print("\n--- 参数校验测试 ---")
        
        test_cases = [
            ('upright_left_turn', {'angle': -10}, ValueError),
            ('upright_left_turn', {'angle': 400}, ValueError),
            ('upright_left_turn', {'speed': 0.1}, ValueError),
            ('upright_left_turn', {'speed': 1.1}, ValueError),
            ('upright_right_turn', {'angle': -10}, ValueError),
            ('upright_right_turn', {'angle': 400}, ValueError),
            ('upright_right_turn', {'speed': 0.1}, ValueError),
            ('upright_right_turn', {'speed': 1.1}, ValueError),
            ('grab', {'angle_a': -10}, ValueError),
            ('grab', {'angle_a': 200}, ValueError),
            ('grab', {'angle_b': -10}, ValueError),
            ('grab', {'angle_b': 200}, ValueError),
        ]
        
        passed = 0
        failed = 0
        
        for method_name, params, expected_exception in test_cases:
            try:
                body = BodyControl.__new__(BodyControl)
                body._is_running = False
                body.debug = False
                
                body._motor_controller = MockMotorController(None)
                body._servo_controller = MockServoController(None)
                body._servos = {
                    'lf_joint': MockServo(8),
                    'rf_joint': MockServo(9),
                    'lb_joint': MockServo(10),
                    'rb_joint': MockServo(11)
                }
                body._pca = MockPCA9685()
                body._current_posture = 'upright'
                body._current_direction = 'stop'
                
                method = getattr(body, method_name)
                method(**params)
                
                print(f"❌ {method_name}({params}) 未抛出预期异常")
                failed += 1
            except expected_exception:
                print(f"✅ {method_name}({params}) 正确抛出异常")
                passed += 1
            except Exception as e:
                print(f"❌ {method_name}({params}) 抛出意外异常: {type(e).__name__}")
                failed += 1
        
        test_results.append(('参数校验', passed, failed))
        return passed, failed
    
    def test_grab_output():
        """测试抓取功能角度占空比设置"""
        print("\n--- 抓取功能角度占空比设置测试 ---")
        
        pca = MockPCA9685()
        
        body = BodyControl.__new__(BodyControl)
        body._is_running = False
        body.debug = False
        body._motor_controller = MockMotorController(pca)
        body._servo_controller = MockServoController(pca)
        body._servos = {
            'lf_joint': MockServo(8),
            'rf_joint': MockServo(9),
            'lb_joint': MockServo(10),
            'rb_joint': MockServo(11)
        }
        body._pca = pca
        body._current_posture = 'upright'
        body._current_direction = 'stop'
        
        test_angle_cases = [
            (90, 90),
            (120, 30),
            (60, 20),
        ]
        
        passed = 0
        failed = 0
        
        for angle_a, angle_b in test_angle_cases:
            try:
                pca._channels[14] = {'on': 0, 'off': 0}
                pca._channels[15] = {'on': 0, 'off': 0}
                
                body.grab(angle_a=angle_a, angle_b=angle_b)
                
                ch14_on, ch14_off = pca.get_channel_raw(14)
                ch15_on, ch15_off = pca.get_channel_raw(15)
                
                servo_min_pulse = 0.5
                servo_max_pulse = 2.5
                servo_period = 20.0
                
                ratio_a = angle_a / 180.0
                pulse_width_a = servo_min_pulse + ratio_a * (servo_max_pulse - servo_min_pulse)
                expected_duty_a = pulse_width_a / servo_period
                expected_raw_a = int(expected_duty_a * 4095)
                
                ratio_b = angle_b / 180.0
                pulse_width_b = servo_min_pulse + ratio_b * (servo_max_pulse - servo_min_pulse)
                expected_duty_b = pulse_width_b / servo_period
                expected_raw_b = int(expected_duty_b * 4095)
                
                if abs(ch14_off - expected_raw_a) <= 20 and abs(ch15_off - expected_raw_b) <= 20:
                    print(f"✅ 抓取角度占空比设置正确 (CH14={angle_a}°, CH15={angle_b}°)")
                    passed += 1
                else:
                    print(f"❌ 抓取角度占空比设置错误: CH14={ch14_off}(预期{expected_raw_a}), CH15={ch15_off}(预期{expected_raw_b})")
                    failed += 1
            except Exception as e:
                print(f"❌ 抓取测试异常: {e}")
                failed += 1
        
        test_results.append(('抓取角度占空比设置', passed, failed))
        return passed, failed
    
    def test_turn_speed_control():
        """测试转向速度控制"""
        print("\n--- 转向速度控制测试 ---")
        
        motor = MockMotorController(None)
        
        body = BodyControl.__new__(BodyControl)
        body._is_running = False
        body.debug = False
        body._motor_controller = motor
        body._servo_controller = MockServoController(None)
        body._servos = {
            'lf_joint': MockServo(8),
            'rf_joint': MockServo(9),
            'lb_joint': MockServo(10),
            'rb_joint': MockServo(11)
        }
        body._pca = MockPCA9685()
        body._current_posture = 'upright'
        body._current_direction = 'stop'
        
        test_speed_cases = [0.3, 0.5, 0.8, 1.0]
        
        passed = 0
        failed = 0
        
        for speed in test_speed_cases:
            try:
                motor._pwm_duty = {0: 0, 1: 0}
                
                body.upright_left_turn(angle=1, speed=speed)
                
                expected_duty = int(speed * MockMotorController.MOTOR_SPEED_DUTY)
                
                if motor._pwm_duty[0] == expected_duty and motor._pwm_duty[1] == expected_duty:
                    print(f"✅ 左转速度控制正确 (speed={speed}, duty={motor._pwm_duty[0]})")
                    passed += 1
                else:
                    print(f"❌ 左转速度控制错误: 预期{expected_duty}, 实际A={motor._pwm_duty[0]}, B={motor._pwm_duty[1]}")
                    failed += 1
            except Exception as e:
                print(f"❌ 左转速度测试异常: {e}")
                failed += 1
        
        for speed in test_speed_cases:
            try:
                motor._pwm_duty = {0: 0, 1: 0}
                
                body.upright_right_turn(angle=1, speed=speed)
                
                expected_duty = int(speed * MockMotorController.MOTOR_SPEED_DUTY)
                
                if motor._pwm_duty[0] == expected_duty and motor._pwm_duty[1] == expected_duty:
                    print(f"✅ 右转速度控制正确 (speed={speed}, duty={motor._pwm_duty[0]})")
                    passed += 1
                else:
                    print(f"❌ 右转速度控制错误: 预期{expected_duty}, 实际A={motor._pwm_duty[0]}, B={motor._pwm_duty[1]}")
                    failed += 1
            except Exception as e:
                print(f"❌ 右转速度测试异常: {e}")
                failed += 1
        
        test_results.append(('转向速度控制', passed, failed))
        return passed, failed
    
    test_param_validation()
    test_grab_output()
    test_turn_speed_control()
    
    print("\n" + "=" * 60)
    print("单元测试汇总")
    print("=" * 60)
    
    total_passed = 0
    total_failed = 0
    
    for test_name, passed, failed in test_results:
        total_passed += passed
        total_failed += failed
        print(f"{test_name}: ✅ {passed} / ❌ {failed}")
    
    print(f"\n总计: ✅ {total_passed} / ❌ {total_failed}")
    
    if total_failed == 0:
        print("\n🎉 所有单元测试通过！")
        return 0
    else:
        print("\n❌ 部分单元测试失败！")
        return 1

def main():
    print("=" * 60)
    print("机器人姿态控制测试程序")
    print("=" * 60)
    
    debug_mode = len(sys.argv) > 1 and sys.argv[1] == "--debug"
    
    try:
        with BodyControl(bus_num=5, addr=0x40, debug=debug_mode) as body:
            print("\n=== 1. 初始状态 ===")
            body.print_status()
            print("\n=== 6. 测试抓取功能（张开90度）===")
            body.upright_forward()
            body.grab(90)
            time.sleep(2000)

            
    except BodyControlInitError as e:
        print(f"\n❌ 初始化失败: {e}")
        print("\n排查建议:")
        print("1. 检查PCA9685硬件连接")
        print("2. 确认OE引脚接GND")
        print("3. 运行 'i2cdetect -y 5' 确认设备地址")
        sys.exit(1)
    
    except BodyControlError as e:
        print(f"\n❌ 姿态控制错误: {e}")
        sys.exit(1)
    
    except KeyboardInterrupt:
        print("\n\n✋ 手动终止程序")
    
    except Exception as e:
        print(f"\n❌ 未知错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--test":
        sys.exit(run_unit_tests())
    else:
        main()
