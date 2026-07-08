#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import json
import random
import threading
from datetime import datetime

current_dir = os.path.dirname(os.path.abspath(__file__))
driver_dir = os.path.join(current_dir, 'Driver')
control_dir = os.path.join(current_dir, 'Control')
app_dir = os.path.join(current_dir, 'App')

for dir_path in [driver_dir, control_dir, app_dir]:
    if dir_path not in sys.path:
        sys.path.insert(0, dir_path)

from pca9685 import PCA9685
from servo import ServoController
from motor import CarMotor
from lora import LoraDevice, list_all_com, select_serial_port
from voice import ASRVoiceSerial
from hmi import RobotHMI
from arm_wheel import BodyControl
from ds_api import DeepSeek


class CameraSimulator:
    """
    摄像头模拟模块：生成符合真实场景格式和频率的模拟数据
    
    预留标准摄像头接口，确保未来硬件接入时无需大规模修改
    """
    
    TERRAIN_TYPES = ["flat", "rubble", "slope", "stairs", "narrow", "obstacle"]
    OBSTACLE_TYPES = ["rock", "steel_bar", "concrete_block", "wood", "wire", "none"]
    SURVIVOR_STATUSES = ["conscious", "unconscious", "trapped", "injured", "none"]
    
    def __init__(self, fps=10, simulate_survivors=True):
        """
        初始化摄像头模拟器
        
        :param fps: 模拟帧率（帧/秒）
        :param simulate_survivors: 是否模拟发现幸存者
        """
        self.fps = fps
        self.simulate_survivors = simulate_survivors
        self.running = False
        self.last_frame_time = 0
        self.frame_count = 0
        
    def start(self):
        """启动摄像头模拟"""
        self.running = True
        print("[CameraSimulator] 摄像头模拟已启动 (FPS={})".format(self.fps))
    
    def stop(self):
        """停止摄像头模拟"""
        self.running = False
        print("[CameraSimulator] 摄像头模拟已停止")
    
    def capture_frame(self):
        """
        模拟采集一帧图像数据
        
        :return: 包含图像信息和场景分析的字典
        """
        current_time = time.time()
        
        if current_time - self.last_frame_time < 1.0 / self.fps:
            return None
        
        self.last_frame_time = current_time
        self.frame_count += 1
        
        frame_data = {
            "timestamp": datetime.now().isoformat(),
            "frame_id": self.frame_count,
            "image_width": 640,
            "image_height": 480,
            "scene_analysis": self._generate_scene_analysis(),
            "survivors": self._generate_survivor_data() if self.simulate_survivors else [],
            "raw_image": "simulated_image_data_{:06d}".format(self.frame_count)
        }
        
        return frame_data
    
    def _generate_scene_analysis(self):
        """生成模拟场景分析数据"""
        terrain = random.choice(self.TERRAIN_TYPES)
        slope = random.randint(0, 30) if terrain == "slope" else 0
        obstacles = []
        
        if terrain == "obstacle":
            obstacle_count = random.randint(1, 3)
            for _ in range(obstacle_count):
                obstacles.append(random.choice([o for o in self.OBSTACLE_TYPES if o != "none"]))
        elif random.random() < 0.3:
            obstacles.append(random.choice([o for o in self.OBSTACLE_TYPES if o != "none"]))
        
        return {
            "terrain": terrain,
            "slope": slope,
            "obstacles": obstacles,
            "passable": random.random() > 0.15,
            "lighting": random.choice(["bright", "dim", "dark"]),
            "visibility": random.uniform(0.3, 1.0)
        }
    
    def _generate_survivor_data(self):
        """生成模拟幸存者数据"""
        survivors = []
        
        if random.random() < 0.1:
            survivor_count = random.randint(1, 2)
            for i in range(survivor_count):
                status = random.choice([s for s in self.SURVIVOR_STATUSES if s != "none"])
                injuries = []
                if status == "injured":
                    injuries = random.sample(["leg_fracture", "arm_fracture", "head_injury", "bruises"], 
                                           random.randint(1, 2))
                
                survivors.append({
                    "id": i + 1,
                    "location": {
                        "x": round(random.uniform(-5.0, 5.0), 2),
                        "y": round(random.uniform(-5.0, 5.0), 2),
                        "z": round(random.uniform(0.0, 1.5), 2)
                    },
                    "status": status,
                    "injuries": injuries,
                    "distance": round(random.uniform(1.0, 10.0), 2)
                })
        
        return survivors
    
    def get_real_image(self):
        """
        预留标准摄像头接口：获取真实图像
        
        当接入实际硬件摄像头时，实现此方法
        """
        raise NotImplementedError("硬件摄像头接口尚未实现，当前使用模拟数据")


class RescueRobot:
    """
    救援机器人大脑：集成所有模块，实现完整业务流程
    
    核心流程：感知 → 决策 → 执行 → 通信 → 循环
    """
    
    ACTION_MAP = {
        "forward": "upright_forward",
        "backward": "upright_backward",
        "turn_left": "upright_left_turn",
        "turn_right": "upright_right_turn",
        "stop": "stop",
        "lie_down": "lie_down_forward",
        "lean": "lean_forward",
        "grab": "grab"
    }
    
    def __init__(self, config=None):
        """
        初始化救援机器人
        
        :param config: 配置字典
        """
        self.config = config or {}
        self.running = False
        self.pca = None
        self.body_control = None
        self.ds_client = None
        self.lora = None
        self.voice = None
        self.hmi = None
        self.camera = None
        
        self.current_status = {
            "posture": "upright",
            "direction": "stop",
            "speed": 0,
            "is_running": False,
            "encoder_counts": {
                "lf": 0, "rf": 0, "lb": 0, "rb": 0
            }
        }
        
        self.discovered_survivors = []
        self.loop_count = 0
    
    def initialize(self):
        """
        初始化所有模块
        
        按照技术文档的初始化流程：
        1. 初始化PCA9685
        2. 初始化舵机控制器
        3. 初始化电机控制器
        4. 初始化姿态控制器
        5. 初始化AI决策模块
        6. 初始化LoRa通信模块
        7. 初始化语音模块
        8. 初始化HMI模块
        9. 初始化摄像头模拟器
        """
        print("\n" + "=" * 60)
        print("初始化救援机器人系统")
        print("=" * 60)
        
        try:
            self._init_pca()
            self._init_body_control()
            self._init_ai_decision()
            self._init_lora()
            self._init_voice()
            self._init_hmi()
            self._init_camera()
            
            print("\n✅ 所有模块初始化完成")
            return True
            
        except Exception as e:
            print("\n❌ 初始化失败: {}".format(e))
            self.cleanup()
            return False
    
    def _init_pca(self):
        """初始化PCA9685"""
        print("\n--- 初始化PCA9685 ---")
        bus_num = self.config.get('pca_bus', 5)
        addr = self.config.get('pca_addr', 0x40)
        
        try:
            self.pca = PCA9685(bus_num=bus_num, addr=addr, debug=False)
            print("✅ PCA9685初始化成功 (I2C{}, 地址0x{:02X})".format(bus_num, addr))
        except Exception as e:
            print("⚠️ PCA9685初始化失败: {} (将使用模拟模式)".format(e))
            self.pca = None
    
    def _init_body_control(self):
        """初始化姿态控制器"""
        print("\n--- 初始化姿态控制器 ---")
        try:
            if self.pca:
                self.body_control = BodyControl(bus_num=self.config.get('pca_bus', 5), 
                                               addr=self.config.get('pca_addr', 0x40))
            else:
                from unittest.mock import MagicMock
                self.body_control = MagicMock()
                self.body_control.get_current_status.return_value = self.current_status
                self.body_control.print_status.return_value = None
            
            print("✅ 姿态控制器初始化成功")
            
        except Exception as e:
            print("⚠️ 姿态控制器初始化失败: {}".format(e))
            self.body_control = None
    
    def _init_ai_decision(self):
        """初始化AI决策模块"""
        print("\n--- 初始化AI决策模块 ---")
        try:
            api_key = self.config.get('ds_api_key', "sk-530fb7fc0a0b4bab81486c0b005e681f")
            base_url = self.config.get('ds_base_url', "https://api.deepseek.com/v1")
            
            system_prompt = """你是地震救援机器人的智能决策助手。
根据路况数据，输出机器人应该执行的动作指令。
路况数据包含：地形类型(terrain)、坡度(slope)、障碍物(obstacles)、是否可通行(passable)、能见度(visibility)。
动作指令只能是以下关键词之一：forward, backward, turn_left, turn_right, stop, lie_down, lean, grab。
请根据实际情况选择最合适的动作。"""
            
            self.ds_client = DeepSeek(api_key=api_key, base_url=base_url, 
                                     default_sys_prompt=system_prompt)
            print("✅ AI决策模块初始化成功")
            
        except Exception as e:
            print("⚠️ AI决策模块初始化失败: {} (将使用本地决策)".format(e))
            self.ds_client = None
    
    def _init_lora(self):
        """初始化LoRa通信模块"""
        print("\n--- 初始化LoRa通信模块 ---")
        try:
            port_name = self.config.get('lora_port', None)
            
            if port_name is None:
                ports = list_all_com()
                if ports:
                    port_name = select_serial_port(ports)
                else:
                    print("⚠️ 未检测到串口设备 (将使用模拟模式)")
                    self.lora = None
                    return
            
            self.lora = LoraDevice(port_name)
            if self.lora.open():
                print("✅ LoRa通信模块初始化成功")
            else:
                print("⚠️ LoRa串口打开失败 (将使用模拟模式)")
                self.lora = None
                
        except Exception as e:
            print("⚠️ LoRa模块初始化失败: {} (将使用模拟模式)".format(e))
            self.lora = None
    
    def _init_voice(self):
        """初始化语音模块"""
        print("\n--- 初始化语音模块 ---")
        try:
            port = self.config.get('voice_port', "/dev/ttyS1")
            
            self.voice = ASRVoiceSerial(port=port, baud=115200, timeout=0.2)
            self.voice.open()
            self.voice.play_audio(20101)
            print("✅ 语音模块初始化成功")
            
        except Exception as e:
            print("⚠️ 语音模块初始化失败: {} (将使用模拟模式)".format(e))
            self.voice = None
    
    def _init_hmi(self):
        """初始化HMI模块"""
        print("\n--- 初始化HMI模块 ---")
        try:
            port = self.config.get('hmi_port', "/dev/ttyUSB0")
            
            self.hmi = RobotHMI(port=port, baud=115200, timeout=0.2)
            self.hmi.open()
            self.hmi.init_all_ui()
            self._register_hmi_actions()
            print("✅ HMI模块初始化成功")
            
        except Exception as e:
            print("⚠️ HMI模块初始化失败: {} (将使用模拟模式)".format(e))
            self.hmi = None
    
    def _register_hmi_actions(self):
        """注册HMI按钮动作回调"""
        action_map = {
            "stop": self._action_stop,
            "scan_path": self._action_scan,
            "gripper_open": self._action_gripper_open,
            "gripper_close": self._action_gripper_close,
            "arm_raise": self._action_arm_raise,
            "arm_lower": self._action_arm_lower,
            "arm_home": self._action_arm_home,
            "look_left": self._action_look_left,
            "look_right": self._action_look_right,
            "look_front": self._action_look_front,
            "shake_hand": self._action_shake_hand,
            "greet": self._action_greet,
            "thigh_center": self._action_thigh_center,
            "system_check": self._action_system_check,
            "comm_clear": self._action_comm_clear,
        }
        
        for action_name, func in action_map.items():
            self.hmi.register_action(action_name, func)
    
    def _action_stop(self):
        """停止动作"""
        self.execute_action("stop")
        return "已停止"
    
    def _action_scan(self):
        """扫描路径"""
        print("[HMI动作] 执行路径扫描")
        return "扫描中..."
    
    def _action_gripper_open(self):
        """夹爪张开"""
        if self.body_control:
            self.body_control.grab(angle_a=90, angle_b=90)
        return "夹爪已张开"
    
    def _action_gripper_close(self):
        """夹爪闭合"""
        if self.body_control:
            self.body_control.grab(angle_a=15, angle_b=15)
        return "夹爪已闭合"
    
    def _action_arm_raise(self):
        """机械臂抬起"""
        print("[HMI动作] 机械臂抬起")
        return "机械臂已抬起"
    
    def _action_arm_lower(self):
        """机械臂放下"""
        print("[HMI动作] 机械臂放下")
        return "机械臂已放下"
    
    def _action_arm_home(self):
        """机械臂归位"""
        print("[HMI动作] 机械臂归位")
        return "机械臂已归位"
    
    def _action_look_left(self):
        """向左看"""
        self.execute_action("turn_left")
        return "正在左转"
    
    def _action_look_right(self):
        """向右看"""
        self.execute_action("turn_right")
        return "正在右转"
    
    def _action_look_front(self):
        """向前看"""
        self.execute_action("forward")
        return "正在前进"
    
    def _action_shake_hand(self):
        """握手"""
        print("[HMI动作] 握手")
        return "握手动作"
    
    def _action_greet(self):
        """打招呼"""
        print("[HMI动作] 打招呼")
        return "打招呼动作"
    
    def _action_thigh_center(self):
        """腿部居中"""
        print("[HMI动作] 腿部居中")
        return "腿部已居中"
    
    def _action_system_check(self):
        """系统检查"""
        print("[HMI动作] 系统检查")
        status = {
            "pca": "正常" if self.pca else "未连接",
            "body_control": "正常" if self.body_control else "未连接",
            "lora": "正常" if self.lora else "未连接",
            "voice": "正常" if self.voice else "未连接",
            "hmi": "正常" if self.hmi else "未连接",
            "camera": "正常" if self.camera else "未连接",
            "ai_decision": "正常" if self.ds_client else "未连接",
        }
        print("系统状态:", status)
        if self.hmi:
            self.hmi.update_system_info(
                asr="正常" if self.voice else "未连接",
                rdk="运行中",
                screen="已连接" if self.hmi else "未连接",
                power="正常",
                version="V1.0"
            )
        return "系统检查完成"
    
    def _action_comm_clear(self):
        """清除通信记录"""
        if self.hmi:
            self.hmi.clear_comm_box()
        return "通信记录已清除"
    
    def _init_camera(self):
        """初始化摄像头模拟器"""
        print("\n--- 初始化摄像头 ---")
        try:
            fps = self.config.get('camera_fps', 10)
            self.camera = CameraSimulator(fps=fps, simulate_survivors=True)
            self.camera.start()
            print("✅ 摄像头模拟器初始化成功 (FPS={})".format(fps))
            
        except Exception as e:
            print("⚠️ 摄像头初始化失败: {}".format(e))
            self.camera = None
    
    def start(self):
        """启动机器人主循环"""
        print("\n" + "=" * 60)
        print("启动救援机器人主循环")
        print("=" * 60)
        
        self.running = True
        
        if self.hmi:
            self.hmi.update_main_page(system="运行中", voice="等待指令", action="启动")
        
        self._start_event_listeners()
        
        try:
            while self.running:
                self.main_loop()
                time.sleep(0.2)
                
        except KeyboardInterrupt:
            print("\n收到中断信号，正在停止...")
            self.stop()
    
    def _start_event_listeners(self):
        """启动事件监听器（语音和HMI）"""
        if self.voice:
            voice_thread = threading.Thread(target=self._voice_listener, daemon=True)
            voice_thread.start()
        
        if self.hmi:
            hmi_thread = threading.Thread(target=self._hmi_listener, daemon=True)
            hmi_thread.start()
    
    def _voice_listener(self):
        """语音指令监听线程"""
        def voice_callback(snid, text, action_name):
            print("\n[语音指令] SNID={}, 文本={}, 动作={}".format(snid, text, action_name))
            self.execute_action(action_name)
        
        try:
            self.voice.listen(callback=voice_callback)
        except Exception as e:
            print("[语音监听异常] {}".format(e))
    
    def _hmi_listener(self):
        """HMI指令监听线程"""
        try:
            self.hmi.listen_loop()
        except Exception as e:
            print("[HMI监听异常] {}".format(e))
    
    def main_loop(self):
        """
        机器人主循环：感知 → 决策 → 执行 → 通信
        
        按照技术文档的核心业务流程执行
        """
        self.loop_count += 1
        
        if self.loop_count % 10 == 0:
            print("\n--- 主循环第 {} 次 ---".format(self.loop_count))
        
        frame_data = self._perceive()
        
        if frame_data:
            decision = self._decide(frame_data)
            
            if decision:
                self._execute(decision)
            
            self._communicate(frame_data)
    
    def _perceive(self):
        """
        感知阶段：获取摄像头数据，提取路况特征
        
        :return: 包含场景分析和幸存者信息的字典
        """
        if not self.camera:
            return None
        
        frame_data = self.camera.capture_frame()
        
        if frame_data:
            if self.loop_count % 10 == 0:
                analysis = frame_data['scene_analysis']
                print("[感知] 地形={}, 坡度={}°, 障碍物={}, 能见度={:.1f}".format(
                    analysis['terrain'], analysis['slope'], 
                    analysis['obstacles'], analysis['visibility']))
                
                if frame_data['survivors']:
                    for survivor in frame_data['survivors']:
                        print("[发现幸存者] ID={}, 状态={}, 距离={}m".format(
                            survivor['id'], survivor['status'], survivor['distance']))
                        self.discovered_survivors.append(survivor)
        else:
            if self.loop_count % 50 == 0:
                print("[感知] 帧间隔未到，跳过采集")
        
        return frame_data
    
    def _decide(self, frame_data):
        """
        决策阶段：构建决策输入，调用AI进行智能决策
        
        :param frame_data: 感知数据
        :return: 动作指令字符串
        """
        if not frame_data or not self.ds_client:
            return self._local_decision(frame_data)
        
        scene = frame_data['scene_analysis']
        
        user_input = "路况数据：地形={}, 坡度={}°, 障碍物={}, 可通行={}, 能见度={:.1f}".format(
            scene['terrain'], scene['slope'], scene['obstacles'], 
            scene['passable'], scene['visibility'])
        
        try:
            reply = self.ds_client.get_reply(user_input)
            action = self._parse_decision(reply)
            
            if self.loop_count % 10 == 0:
                print("[决策] AI回复={}, 解析动作={}".format(reply, action))
            
            return action
            
        except Exception as e:
            print("[决策异常] AI调用失败: {}, 使用本地决策".format(e))
            return self._local_decision(frame_data)
    
    def _local_decision(self, frame_data):
        """
        本地决策：当AI模块不可用时的备用决策逻辑
        
        :param frame_data: 感知数据
        :return: 动作指令字符串
        """
        if not frame_data:
            return "stop"
        
        scene = frame_data['scene_analysis']
        
        if not scene['passable']:
            return "turn_left"
        
        if scene['terrain'] == "obstacle":
            return "turn_right"
        
        if scene['terrain'] == "narrow":
            return "lean"
        
        if scene['terrain'] == "slope":
            return "lean"
        
        if scene['terrain'] == "stairs":
            return "lie_down"
        
        if random.random() < 0.05:
            return random.choice(["turn_left", "turn_right"])
        
        return "forward"
    
    def _parse_decision(self, reply):
        """
        解析AI决策结果
        
        :param reply: AI回复文本
        :return: 标准化的动作指令
        """
        reply = reply.lower().strip()
        
        for action_key in self.ACTION_MAP.keys():
            if action_key in reply:
                return action_key
        
        return "forward"
    
    def _execute(self, action):
        """
        执行阶段：根据决策结果执行相应动作
        
        :param action: 动作指令
        """
        if not self.body_control:
            return
        
        try:
            method_name = self.ACTION_MAP.get(action)
            
            if method_name == "upright_left_turn":
                self.body_control.upright_left_turn(angle=45, speed=0.5)
            elif method_name == "upright_right_turn":
                self.body_control.upright_right_turn(angle=45, speed=0.5)
            elif method_name == "grab":
                self.body_control.grab(angle_a=15, angle_b=15)
                time.sleep(1)
                self.body_control.grab(angle_a=90, angle_b=90)
            elif method_name and hasattr(self.body_control, method_name):
                getattr(self.body_control, method_name)()
            
            if self.loop_count % 10 == 0:
                print("[执行] 动作={}".format(action))
            
            if self.hmi:
                self.hmi.update_main_page(action=action)
            
        except Exception as e:
            print("[执行异常] {}: {}".format(action, e))
    
    def _communicate(self, frame_data):
        """
        通信阶段：收集状态信息，通过LoRa发送至指挥中心
        
        :param frame_data: 感知数据
        """
        if not self.lora:
            return
        
        try:
            status_data = self._build_status_data(frame_data)
            json_data = json.dumps(status_data, ensure_ascii=False, indent=2)
            
            self.lora.send(json_data)
            
            if self.loop_count % 10 == 0:
                print("[通信] 数据已发送 (长度={}字节)".format(len(json_data)))
            
            if self.hmi:
                self.hmi.update_comm_status(send_state="已发送")
            
        except Exception as e:
            print("[通信异常] {}".format(e))
    
    def _build_status_data(self, frame_data):
        """
        构建发送数据结构
        
        :param frame_data: 感知数据
        :return: 完整的状态数据字典
        """
        status = self.body_control.get_current_status() if self.body_control else self.current_status
        
        data = {
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "status": {
                "posture": status.get("posture", "upright"),
                "direction": status.get("direction", "stop"),
                "speed": status.get("speed", 0),
                "encoder_counts": status.get("encoder_counts", {
                    "lf": 0, "rf": 0, "lb": 0, "rb": 0
                })
            },
            "environment": {},
            "survivors": []
        }
        
        if frame_data:
            analysis = frame_data['scene_analysis']
            data['environment'] = {
                "terrain": analysis['terrain'],
                "obstacles": analysis['obstacles'],
                "slope": analysis['slope'],
                "visibility": analysis['visibility']
            }
            
            if frame_data['survivors']:
                data['survivors'] = frame_data['survivors']
        
        if self.discovered_survivors:
            data['total_survivors_discovered'] = len(self.discovered_survivors)
        
        return data
    
    def execute_action(self, action_name):
        """
        执行外部触发的动作（语音/HMI）
        
        :param action_name: 动作名称
        """
        if self.hmi:
            self.hmi.update_main_page(action=action_name)
        
        self._execute(action_name)
        
        if self.voice:
            snid = next((k for k, v in self.voice.ACTION_MAP.items() if v == action_name), None)
            if snid:
                self.voice.play_by_snid(snid)
    
    def stop(self):
        """停止机器人"""
        self.running = False
        
        if self.body_control:
            self.body_control.stop()
            self.body_control.cleanup()
        
        if self.lora:
            self.lora.close()
        
        if self.voice:
            self.voice.close()
        
        if self.hmi:
            self.hmi.update_main_page(system="已停止", action="空闲")
            self.hmi.close()
        
        if self.camera:
            self.camera.stop()
        
        if self.pca:
            self.pca.close()
        
        print("\n✅ 救援机器人已停止，资源已清理")
    
    def cleanup(self):
        """清理所有资源"""
        self.running = False
        
        try:
            if self.body_control:
                self.body_control.cleanup()
        except:
            pass
        
        try:
            if self.lora:
                self.lora.close()
        except:
            pass
        
        try:
            if self.voice:
                self.voice.close()
        except:
            pass
        
        try:
            if self.hmi:
                self.hmi.close()
        except:
            pass
        
        try:
            if self.camera:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
                self.camera.stop()
        except:
            pass
        
        try:
            if self.pca:
                self.pca.close()
        except:
            pass


def main():
    """主程序入口"""
    config = {
        'pca_bus': 5,
        'pca_addr': 0x40,
        'lora_port': None,
        'voice_port': "/dev/ttyS1",
        'hmi_port': "/dev/ttyUSB0",
        'camera_fps': 10,
        'ds_api_key': "sk-530fb7fc0a0b4bab81486c0b005e681f",
        'ds_base_url': "https://api.deepseek.com/v1"
    }
    
    robot = RescueRobot(config=config)
    
    if robot.initialize():
        try:
            robot.start()
        except KeyboardInterrupt:
            print("\n程序被用户中断")
        except Exception as e:
            print("\n程序异常退出: {}".format(e))
        finally:
            robot.cleanup()
    else:
        print("初始化失败，程序退出")


if __name__ == "__main__":
    main()