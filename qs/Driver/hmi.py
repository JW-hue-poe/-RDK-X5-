#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import serial


# 按钮-动作映射、救援消息映射（内置进类，无需外部导入）
BTN_ACTIONS = {
    "STOP": "stop",
    "SCAN": "scan_path",
    "SHAKE_HAND": "shake_hand",
    "GREET": "greet",
    "LOOK_LEFT": "look_left",
    "LOOK_RIGHT": "look_right",
    "LOOK_FRONT": "look_front",
    "GRIPPER_OPEN": "gripper_open",
    "GRIPPER_CLOSE": "gripper_close",
    "ARM_RAISE": "arm_raise",
    "ARM_LOWER": "arm_lower",
    "ARM_HOME": "arm_home",
    "THIGH_CENTER": "thigh_center",
    "THIGH_TEST": "thigh_test",
    "THIGH_CH8_TEST": "thigh_ch8_test",
    "THIGH_CH9_TEST": "thigh_ch9_test",
    "THIGH_CH10_TEST": "thigh_ch10_test",
    "THIGH_CH11_TEST": "thigh_ch11_test",
    "THIGH_CH12_TEST": "thigh_ch12_test",
    "COMM_CLEAR": "comm_clear",
    "SYSTEM_CHECK": "system_check",
}

RESCUE_MESSAGES = {
    "HELP": "请求救援",
    "INJURED": "发现伤员",
    "BAD_AIR": "通风不足",
    "NEED_WATER": "缺少饮水",
    "IMPORTANT": "我有重要信息汇报",
    "SAFE": "我们所在位置较安全",
    "NEED_FOOD_WATER": "缺少水和食物",
    "SMELL": "现场有异常气味",
    "WAITING": "我们正在等待救援",
}


class RobotHMI:
    """
    一体化串口HMI触摸屏控制类
    融合：串口底层驱动 + 协议解析
    对外功能：屏幕状态刷新、指令接收解析、页面切换、文本设置
    """
    def __init__(self, port="/dev/ttyUSB0", baud=115200, timeout=0.2, encoding="gbk"):
        # 串口基础配置
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.encoding = encoding
        self.ser = None
        # 外部动作回调容器，用户注册自定义动作函数
        self.action_callback_map = {}

    # ===================== 串口底层接口 =====================
    def open(self):
        """打开串口屏幕"""
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            timeout=self.timeout
        )
        time.sleep(0.2)
        print(f"[RobotHMI] 屏幕串口打开成功 {self.port} {self.baud}")

    def close(self):
        """关闭串口释放资源"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[RobotHMI] 串口已关闭")

    def is_connected(self):
        """判断串口是否正常连接"""
        return self.ser is not None and self.ser.is_open

    # ===================== 屏幕下发指令底层 =====================
    def send_raw_cmd(self, cmd: str):
        """发送原始ATK屏幕指令"""
        if not self.is_connected():
            raise RuntimeError("HMI串口未打开，请先调用open()")
        cmd = cmd.strip()
        if not cmd.endswith(";"):
            cmd += ";"
        data = (cmd + "\r\n").encode(self.encoding, errors="ignore")
        self.ser.write(data)
        self.ser.flush()
        print(f"[HMI_TX] {cmd}")

    def set_text(self, obj_name: str, text):
        """设置屏幕文本控件内容，自动转义引号"""
        text = str(text).replace("\\", "\\\\").replace('"', '\\"')
        self.send_raw_cmd(f'{obj_name}.txt.str="{text}"')

    def clear_text(self, obj_name: str):
        """清空指定文本框"""
        self.set_text(obj_name, "")

    def switch_page(self, page_id: int):
        """切换屏幕页面 page(0)"""
        self.send_raw_cmd(f"page({page_id})")

    # ===================== 分页状态快速刷新API =====================
    def update_main_page(self, system=None, voice=None, action=None):
        if system is not None:
            self.set_text("t1x", system)
        if voice is not None:
            self.set_text("t1y", voice)
        if action is not None:
            self.set_text("t0d", action)

    def update_gimbal_page(self, system=None, voice=None, action=None):
        if system is not None:
            self.set_text("t2x", system)
        if voice is not None:
            self.set_text("t2y", voice)
        if action is not None:
            self.set_text("t2d", action)

    def update_arm_page(self, system=None, voice=None, action=None):
        if system is not None:
            self.set_text("t3x", system)
        if voice is not None:
            self.set_text("t3y", voice)
        if action is not None:
            self.set_text("t3d", action)

    def update_leg_page(self, system=None, voice=None, action=None):
        if system is not None:
            self.set_text("t4x", system)
        if voice is not None:
            self.set_text("t4y", voice)
        if action is not None:
            self.set_text("t4d", action)

    def update_system_info(self, asr=None, rdk=None, screen=None, power=None, version=None):
        if asr is not None:
            self.set_text("t5a", asr)
        if rdk is not None:
            self.set_text("t5r", rdk)
        if screen is not None:
            self.set_text("t5p", screen)
        if power is not None:
            self.set_text("t5d", power)
        if version is not None:
            self.set_text("t5b", version)

    def update_comm_status(self, serial_state=None, baud=None, send_state=None, recv_state=None):
        if serial_state is not None:
            self.set_text("t6c", serial_state)
        if baud is not None:
            self.set_text("t6b", str(baud))
        if send_state is not None:
            self.set_text("t6f", send_state)
        if recv_state is not None:
            self.set_text("t6l", recv_state)

    def set_comm_send_box(self, text):
        self.set_text("t_send", text)

    def set_comm_recv_box(self, text):
        self.set_text("t_recv", text)

    def clear_comm_box(self):
        self.clear_text("t_send")
        self.clear_text("t_recv")

    def init_all_ui(self):
        """一键初始化全部页面默认状态"""
        self.update_main_page(system="运行中", voice="等待指令", action="空闲")
        self.update_gimbal_page(system="运行中", voice="等待指令", action="空闲")
        self.update_arm_page(system="运行中", voice="等待指令", action="空闲")
        self.update_leg_page(system="运行中", voice="等待指令", action="空闲")
        self.update_system_info(asr="正常", rdk="运行中", screen="已连接", power="正常", version="V1.0")
        self.update_comm_status(serial_state="已连接", baud=self.baud, send_state="空闲", recv_state="等待数据")

    # ===================== 内置协议解析逻辑（融合原hmi_protocol） =====================
    @staticmethod
    def _decode_raw_bytes(raw: bytes):
        """自动兼容GBK/UTF8解码，防止中文乱码"""
        if not raw:
            return ""
        for enc in ("gbk", "utf-8"):
            try:
                return raw.decode(enc).strip()
            except UnicodeDecodeError:
                continue
        return raw.decode("utf-8", errors="ignore").strip()

    def parse_recv_line(self, line: str):
        """解析屏幕上行数据，返回结构化元组"""
        line = line.strip()
        if not line:
            return None
        if line.startswith("<BTN>"):
            key = line.replace("<BTN>", "", 1).strip()
            action = BTN_ACTIONS.get(key)
            if action:
                return ("button", action, key)
            return ("unknown_button", key, None)
        if line.startswith("<RESCUE>"):
            key = line.replace("<RESCUE>", "", 1).strip()
            msg = RESCUE_MESSAGES.get(key)
            if msg:
                return ("rescue_msg", msg, key)
            return ("unknown_rescue", key, None)
        if line.startswith("<MSG>"):
            text = line.replace("<MSG>", "", 1).strip()
            return ("user_text", text, None)
        return ("unknown_data", line, None)

    # ===================== 事件回调注册（用户绑定动作函数） =====================
    def register_action(self, action_name: str, func):
        """注册动作回调，按钮触发时自动执行func"""
        self.action_callback_map[action_name] = func

    # ===================== 阻塞式循环监听（主循环直接调用） =====================
    def listen_loop(self, sleep_ms=20):
        """持续监听屏幕串口数据，自动解析并执行注册动作"""
        if not self.is_connected():
            raise RuntimeError("请先调用 open() 打开串口")
        print("[RobotHMI] 进入HMI监听循环，等待屏幕指令...")
        try:
            while True:
                raw_data = self.ser.readline()
                line = self._decode_raw_bytes(raw_data)
                if not line:
                    time.sleep(sleep_ms / 1000)
                    continue
                print(f"[HMI_RX] {line}")
                parse_result = self.parse_recv_line(line)
                if parse_result is None:
                    continue
                msg_type, val, raw_key = parse_result

                # 按钮事件：自动执行注册的动作函数
                if msg_type == "button":
                    act_name = val
                    self.update_comm_status(recv_state=f"按键:{raw_key}")
                    self.set_comm_recv_box(f"触发按键 {raw_key}")
                    if act_name in self.action_callback_map:
                        try:
                            ret = self.action_callback_map[act_name]()
                            print(f"[动作执行成功] {act_name} 返回：{ret}")
                        except Exception as e:
                            print(f"[动作执行异常] {act_name} error: {e}")
                            self.update_main_page(system="异常", action="动作执行失败")
                    else:
                        print(f"[警告] 未注册动作 {act_name}")
                        self.set_comm_recv_box(f"未注册按键 {raw_key}")

                # 救援消息事件
                elif msg_type == "rescue_msg":
                    text = val
                    print(f"[救援消息] {text}")
                    self.update_comm_status(send_state="已发送", recv_state="救援快捷消息")
                    self.set_comm_recv_box(f"救援信息：{text}")
                    self.update_main_page(action=f"发送救援：{text}")

                # 用户自定义输入文本
                elif msg_type == "user_text":
                    text = val
                    if not text:
                        self.set_comm_recv_box("输入内容为空")
                        self.update_comm_status(recv_state="空消息")
                        continue
                    print(f"[用户文本] {text}")
                    self.update_comm_status(send_state="已发送", recv_state="自定义文本")
                    self.set_comm_recv_box(f"用户输入：{text}")
                    self.update_main_page(action="发送自定义消息")

                # 未知数据处理
                else:
                    self.set_comm_recv_box(f"未知数据：{val}")
                    self.update_comm_status(recv_state="未知指令")
        except KeyboardInterrupt:
            print("\n[RobotHMI] 监听循环手动退出")