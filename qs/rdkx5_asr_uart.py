#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import serial


# =========================================================
# 串口配置
# =========================================================

# ASRPRO 接到 RDK X5 的哪个串口，就填哪个
# 你之前测试 ASRPRO 用的是 /dev/ttyS1
ASR_PORT = "/dev/ttyS1"

# 必须和 ASRPRO 端 Serial.begin(115200) 一致
ASR_BAUD = 115200


# =========================================================
# ASR 指令映射表
# =========================================================
# 逻辑：
# RDK 收到 <ASR>id,text
# 先发送 <PLAY>play_id 给 ASRPRO 播报
# 再等待 play_delay 秒
# 最后执行 action 动作
# =========================================================

ASR_COMMANDS = {
    # =====================
    # 基础运动
    # =====================
    0: {
        "text": "唤醒",
        "play_id": 20102,
        "action": "wake",
        "delay": 1.2,
    },
    1: {
        "text": "向前走",
        "play_id": 20001,
        "action": "forward",
        "delay": 1.8,
    },
    2: {
        "text": "向后退",
        "play_id": 20002,
        "action": "backward",
        "delay": 1.5,
    },
    3: {
        "text": "向左转",
        "play_id": 20003,
        "action": "turn_left",
        "delay": 1.4,
    },
    4: {
        "text": "向右转",
        "play_id": 20004,
        "action": "turn_right",
        "delay": 1.4,
    },
    5: {
        "text": "停一下",
        "play_id": 20005,
        "action": "stop",
        "delay": 1.2,
    },
    6: {
        "text": "坐下来",
        "play_id": 20006,
        "action": "sit_down",
        "delay": 1.4,
    },
    7: {
        "text": "趴下来",
        "play_id": 20007,
        "action": "lie_down",
        "delay": 1.5,
    },
    8: {
        "text": "站起来",
        "play_id": 20008,
        "action": "stand_up",
        "delay": 1.3,
    },
    9: {
        "text": "握个手",
        "play_id": 20009,
        "action": "shake_hand",
        "delay": 1.4,
    },
    10: {
        "text": "转个圈",
        "play_id": 20010,
        "action": "turn_around",
        "delay": 1.5,
    },
    11: {
        "text": "打招呼",
        "play_id": 20011,
        "action": "greet",
        "delay": 1.6,
    },
    12: {
        "text": "你是谁",
        "play_id": 20101,
        "action": "introduce",
        "delay": 2.4,
    },

    # =====================
    # 云台与避障
    # =====================
    20: {
        "text": "前方障碍",
        "play_id": 20201,
        "action": "obstacle_scan",
        "delay": 2.2,
    },
    21: {
        "text": "寻找出路",
        "play_id": 20201,
        "action": "scan_path",
        "delay": 2.2,
    },
    22: {
        "text": "看左边",
        "play_id": 20202,
        "action": "look_left",
        "delay": 1.1,
    },
    23: {
        "text": "看右边",
        "play_id": 20203,
        "action": "look_right",
        "delay": 1.1,
    },
    24: {
        "text": "看前方",
        "play_id": 20204,
        "action": "look_front",
        "delay": 1.1,
    },

    # =====================
    # 机械臂与夹爪
    # =====================
    30: {
        "text": "机械臂抬起来",
        "play_id": 20301,
        "action": "arm_raise",
        "delay": 1.5,
    },
    31: {
        "text": "机械臂放下",
        "play_id": 20302,
        "action": "arm_lower",
        "delay": 1.5,
    },
    32: {
        "text": "张开夹爪",
        "play_id": 20303,
        "action": "gripper_open",
        "delay": 1.3,
    },
    33: {
        "text": "合上夹爪",
        "play_id": 20304,
        "action": "gripper_close",
        "delay": 1.3,
    },
    34: {
        "text": "机械臂回位",
        "play_id": 20305,
        "action": "arm_home",
        "delay": 1.6,
    },
    35: {
        "text": "抓一下",
        "play_id": 20304,
        "action": "grab_once",
        "delay": 1.3,
    },

    # =====================
    # 四条大腿舵机预留
    # =====================
    50: {
        "text": "四条腿回中",
        "play_id": 20401,
        "action": "thigh_center",
        "delay": 1.5,
    },
    51: {
        "text": "大腿测试",
        "play_id": 20402,
        "action": "thigh_test",
        "delay": 1.5,
    },
}


# =========================================================
# 机器人动作类
# 这里先保留动作接口，后面你把真实 motor / servo / arm / gimbal 接进来
# =========================================================

class XiaoyaRobot:
    def __init__(self):
        print("[Robot] 小娅机器人动作系统初始化")

        # 后面你可以在这里接入真实模块：
        # from control.gimbal import Gimbal
        # from control.arm import Arm
        # from control.leg import LegController
        # from control.motor import MotorController
        #
        # self.gimbal = Gimbal()
        # self.arm = Arm()
        # self.leg = LegController()
        # self.motor = MotorController()

    # =====================
    # 基础运动
    # =====================
    def wake(self):
        print("[动作] 小娅已唤醒")

    def forward(self):
        print("[动作] 向前走")
        # TODO: 接入真实电机前进
        # self.motor.forward()

    def backward(self):
        print("[动作] 向后退")
        # TODO: self.motor.backward()

    def turn_left(self):
        print("[动作] 向左转")
        # TODO: self.motor.turn_left()

    def turn_right(self):
        print("[动作] 向右转")
        # TODO: self.motor.turn_right()

    def stop(self):
        print("[动作] 停止")
        # TODO: self.motor.stop()

    def sit_down(self):
        print("[动作] 坐下")
        # TODO: 接入姿态控制

    def lie_down(self):
        print("[动作] 趴下")
        # TODO: 接入姿态控制

    def stand_up(self):
        print("[动作] 站起来")
        # TODO: 接入姿态控制

    def shake_hand(self):
        print("[动作] 握手")
        # TODO: 接入机械臂握手

    def turn_around(self):
        print("[动作] 转圈")
        # TODO: 接入电机转圈

    def greet(self):
        print("[动作] 打招呼")
        # TODO: 接入机械臂打招呼动作

    def introduce(self):
        print("[动作] 自我介绍")
        # 这个一般只播报，不需要机械动作

    # =====================
    # 云台与避障
    # =====================
    def obstacle_scan(self):
        print("[云台] 前方障碍，开始寻找其他出路")
        self.scan_path()

    def scan_path(self):
        print("[云台] 扫描出路")
        # TODO:
        # self.gimbal.look_left()
        # time.sleep(1)
        # self.gimbal.look_right()
        # time.sleep(1)
        # self.gimbal.look_front()

    def look_left(self):
        print("[云台] 看左边")
        # TODO: self.gimbal.look_left()

    def look_right(self):
        print("[云台] 看右边")
        # TODO: self.gimbal.look_right()

    def look_front(self):
        print("[云台] 看前方")
        # TODO: self.gimbal.look_front()

    # =====================
    # 机械臂与夹爪
    # =====================
    def arm_raise(self):
        print("[机械臂] 抬起来")
        # TODO: self.arm.raise_arm()

    def arm_lower(self):
        print("[机械臂] 放下")
        # TODO: self.arm.lower_arm()

    def gripper_open(self):
        print("[夹爪] 张开")
        # TODO: self.arm.gripper_open()

    def gripper_close(self):
        print("[夹爪] 合上")
        # TODO: self.arm.gripper_close()

    def arm_home(self):
        print("[机械臂] 回位")
        # TODO: self.arm.home()

    def grab_once(self):
        print("[夹爪] 抓一下")
        # TODO:
        # self.arm.gripper_open()
        # time.sleep(0.5)
        # self.arm.gripper_close()

    # =====================
    # 大腿舵机
    # =====================
    def thigh_center(self):
        print("[腿部] 四条腿回中")
        # TODO: self.leg.center_all()

    def thigh_test(self):
        print("[腿部] 大腿测试")
        # TODO: self.leg.test_all()


# =========================================================
# ASRPRO 串口通信类
# =========================================================

class ASRProSerial:
    def __init__(self, port=ASR_PORT, baud=ASR_BAUD):
        self.port = port
        self.baud = baud
        self.ser = None

    def open(self):
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            timeout=0.05
        )
        time.sleep(0.3)
        print(f"[ASR] 串口已打开：{self.port}, baud={self.baud}")

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[ASR] 串口已关闭")

    def send_line(self, text):
        """
        给 ASRPRO 发送一行指令。
        ASRPRO 端 readStringUntil('\\n')，所以必须带 \\n。
        """
        if not self.ser or not self.ser.is_open:
            print("[ASR] 串口未打开，发送失败")
            return

        data = (text.strip() + "\n").encode("utf-8", errors="ignore")
        self.ser.write(data)
        self.ser.flush()
        print("[RDK -> ASR]", text.strip())

    def play(self, play_id):
        self.send_line(f"<PLAY>{play_id}")

    def set_vol(self, vol):
        if vol < 1:
            vol = 1
        if vol > 7:
            vol = 7
        self.send_line(f"<VOL>{vol}")

    def wake(self, ms=20000):
        self.send_line(f"<WAKE>{ms}")

    def read_line(self):
        if not self.ser or not self.ser.is_open:
            return ""

        raw = self.ser.readline()
        if not raw:
            return ""

        for enc in ("utf-8", "gbk"):
            try:
                return raw.decode(enc).strip()
            except UnicodeDecodeError:
                pass

        return raw.decode("utf-8", errors="ignore").strip()


# =========================================================
# 协议解析
# =========================================================

def parse_asr_line(line):
    """
    解析 ASRPRO 发来的数据。

    支持：
        <READY>ASRPRO
        <ASR>1,向前走

    返回：
        ("ready", None, text)
        ("asr", snid, text)
        ("unknown", None, line)
    """
    line = line.strip()

    if not line:
        return None

    if line.startswith("<READY>"):
        text = line.replace("<READY>", "", 1).strip()
        return ("ready", None, text)

    if line.startswith("<ASR>"):
        body = line.replace("<ASR>", "", 1).strip()

        if "," in body:
            sid_str, text = body.split(",", 1)
        else:
            sid_str, text = body, ""

        try:
            sid = int(sid_str)
        except ValueError:
            return ("unknown", None, line)

        return ("asr", sid, text.strip())

    return ("unknown", None, line)


# =========================================================
# 小娅中控
# =========================================================

class XiaoyaASRController:
    def __init__(self):
        self.asr = ASRProSerial()
        self.robot = XiaoyaRobot()

    def open(self):
        self.asr.open()

        # 给 ASRPRO 设置音量
        self.asr.set_vol(7)

        # 让 ASRPRO 保持唤醒一段时间
        self.asr.wake(20000)

    def close(self):
        self.asr.close()

    def handle_asr_command(self, snid, text):
        print(f"[ASR识别] snid={snid}, text={text}")

        cmd = ASR_COMMANDS.get(snid)

        if cmd is None:
            print("[ASR] 未知指令，播放不会提示")
            self.asr.play(20199)
            return

        play_id = cmd["play_id"]
        action_name = cmd["action"]
        delay = cmd["delay"]

        print(f"[流程] 先播报：play_id={play_id}")
        self.asr.play(play_id)

        print(f"[流程] 等待播报完成约 {delay} 秒")
        time.sleep(delay)

        print(f"[流程] 再执行动作：{action_name}")
        action_func = getattr(self.robot, action_name, None)

        if action_func is None:
            print(f"[错误] 动作函数不存在：{action_name}")
            self.asr.play(20199)
            return

        action_func()

    def run(self):
        self.open()

        print("========== 小娅 ASRPRO RDK X5 中控启动 ==========")
        print("等待 ASRPRO 数据...")
        print("协议示例：<ASR>1,向前走")
        print("按 Ctrl+C 退出")
        print("================================================")

        try:
            while True:
                line = self.asr.read_line()

                if not line:
                    time.sleep(0.01)
                    continue

                print("[ASR -> RDK]", line)

                parsed = parse_asr_line(line)
                if parsed is None:
                    continue

                msg_type, snid, text = parsed

                if msg_type == "ready":
                    print(f"[ASR] 模块已启动：{text}")
                    self.asr.play(10001)

                elif msg_type == "asr":
                    self.handle_asr_command(snid, text)

                else:
                    print("[ASR] 未知数据：", text)

        except KeyboardInterrupt:
            print("\n[退出] 用户终止程序")

        finally:
            self.close()
            print("========== 小娅 ASRPRO 中控结束 ==========")


# =========================================================
# 程序入口
# =========================================================

if __name__ == "__main__":
    controller = XiaoyaASRController()
    controller.run()