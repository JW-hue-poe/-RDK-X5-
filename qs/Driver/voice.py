#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import serial
import time


class ASRVoiceSerial:
    """
    ASRPRO 语音模块双向串口通信类
    包含：串口收发、ASR数据解析、语音播报发送，纯语音协议层，无任何硬件舵机控制
    """
    ACTION_MAP = {
        0: "wake",
        1: "forward",
        2: "backward",
        3: "turn_left",
        4: "turn_right",
        5: "stop",
        6: "sit",
        7: "lie_down",
        8: "stand_up",
        9: "shake_hand",
        10: "spin",
        11: "greet",
        12: "self_intro",
        20: "obstacle_scan",
        21: "find_path",
        22: "look_left",
        23: "look_right",
        24: "look_front",
        30: "arm_raise",
        31: "arm_lower",
        32: "gripper_open",
        33: "gripper_close",
        34: "arm_home",
        35: "grab_once",
        50: "thigh_center",
        51: "thigh_test",
    }

    PLAY_MAP = {
        1: 20001, 2: 20002, 3: 20003, 4: 20004, 5: 20005,
        6: 20006, 7: 20007, 8: 20008, 9: 20009, 10: 20010,
        11: 20011, 12: 20101, 20: 20201, 21: 20201, 22: 20202,
        23: 20203, 24: 20204, 30: 20301, 31: 20302, 32: 20303,
        33: 20304, 34: 20305, 35: 20304, 50: 20401, 51: 20402,
    }

    ERR_AUDIO_ID = 20199

    def __init__(self, port="/dev/ttyS1", baud=115200, timeout=0.2):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None

    def open(self):
        """打开串口，清空缓冲区"""
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            timeout=self.timeout
        )
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        time.sleep(0.1)
        print(f"[ASR] 语音串口已打开 {self.port} {self.baud}")

    def close(self):
        """关闭串口释放资源"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None
            print("[ASR] 语音串口已关闭")

    def is_connected(self):
        return self.ser is not None and self.ser.is_open

    def parse_line(self, raw_line: str):
        """
        解析串口读到的一行数据
        返回格式：(snid:int, text:str, action_name:str) / None
        """
        line = raw_line.strip()
        if line.startswith("<READY>"):
            return ("ready", None, None)
        if not line.startswith("<ASR>"):
            return None

        payload = line[5:].strip()
        if "," in payload:
            snid_str, text = payload.split(",", 1)
        else:
            snid_str, text = payload, ""

        try:
            snid = int(snid_str)
        except ValueError:
            return None

        action_name = self.ACTION_MAP.get(snid, "unknown")
        return ("asr", snid, text, action_name)

    def _send_play_cmd(self, audio_id: int):
        """底层发送播放指令 <PLAY>xxx"""
        if not self.is_connected():
            raise RuntimeError("串口未打开，请先调用open()")
        cmd = f"<PLAY>{audio_id}\n"
        self.ser.write(cmd.encode("utf-8"))
        self.ser.flush()
        print(f"[ASR_TX] {cmd.strip()}")
        time.sleep(0.05)

    def play_by_snid(self, snid: int):
        """根据ASR编号自动匹配语音播报"""
        audio_id = self.PLAY_MAP.get(snid, self.ERR_AUDIO_ID)
        self._send_play_cmd(audio_id)
        return audio_id

    def play_audio(self, audio_id: int):
        """直接指定音频编号播放（触摸屏/手动调用）"""
        self._send_play_cmd(audio_id)

    def listen(self, callback=None):
        """
        持续循环读取ASR串口数据
        :param callback: 回调函数，收到有效ASR指令时自动回调
                         回调入参 (snid, text, action_name)
        """
        if not self.is_connected():
            raise RuntimeError("串口未打开，无法监听")
        print("[ASR] 开始监听语音指令...")
        try:
            while True:
                raw_bytes = self.ser.readline()
                if not raw_bytes:
                    continue
                line = raw_bytes.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                print(f"[ASR_RX] {line}")

                parse_res = self.parse_line(line)
                if parse_res is None:
                    continue
                if parse_res[0] == "ready":
                    print("[ASR 模块就绪]")
                    continue
                if parse_res[0] == "asr":
                    _, snid, text, action_name = parse_res
                    if callback is not None:
                        callback(snid, text, action_name)
                    self.play_by_snid(snid)
        except KeyboardInterrupt:
            print("\n[ASR] 监听循环退出")


class MockSerial:
    """模拟串口类，用于无硬件时的测试"""
    def __init__(self, port, baudrate, timeout=0.2):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._buffer = []
        self._simulate_commands = []
        self.is_open = True

    def reset_input_buffer(self):
        self._buffer = []

    def reset_output_buffer(self):
        pass

    def write(self, data):
        print(f"[模拟串口发送] {data.decode('utf-8').strip()}")

    def flush(self):
        pass

    def readline(self):
        time.sleep(0.1)
        if self._buffer:
            return self._buffer.pop(0)
        return b""

    def close(self):
        self.is_open = False
        print(f"[模拟串口] {self.port} 已关闭")

    def add_simulated_command(self, command):
        """添加模拟指令到缓冲区"""
        self._buffer.append((command + "\n").encode("utf-8"))


def test_with_mock_serial():
    """使用模拟串口进行测试"""
    print("\n" + "=" * 60)
    print("模拟串口测试模式")
    print("=" * 60)
    print("测试场景：模拟用户说'左转'")
    print("=" * 60)

    mock_ser = MockSerial("/dev/ttyS1", 115200)
    mock_ser.add_simulated_command("<ASR>3,向左转")
    mock_ser.add_simulated_command("<ASR>4,向右转")
    mock_ser.add_simulated_command("<ASR>1,向前走")

    asr = ASRVoiceSerial(port="/dev/ttyS1", baud=115200, timeout=0.2)
    asr.ser = mock_ser

    def handle_voice_command(snid, text, action_name):
            print("\n" + "-" * 50)
            print(f"[指令解析]")
            print(f"  SNID: {snid}")
            print(f"  识别文本: {text}")
            print(f"  动作名称: {action_name}")

            if action_name == "turn_left":
                print("  ✅ 检测到'左转'指令")
                print("  ✅ 将发送 <PLAY>20003 给语音模块播报'向左转'")
            elif action_name == "turn_right":
                print("  ✅ 检测到'右转'指令")
                print("  ✅ 将发送 <PLAY>20004 给语音模块播报'向右转'")
            elif action_name == "forward":
                print("  ✅ 检测到'前进'指令")
                print("  ✅ 将发送 <PLAY>20001 给语音模块播报'向前走'")
            elif action_name == "backward":
                print("  ✅ 检测到'后退'指令")
                print("  ✅ 将发送 <PLAY>20002 给语音模块播报'向后退'")
            elif action_name == "stop":
                print("  ✅ 检测到'停止'指令")
                print("  ✅ 将发送 <PLAY>20005 给语音模块播报'停一下'")
            elif action_name == "wake":
                print("  ℹ️ 检测到唤醒词，不额外播报")
            else:
                print(f"  🔄 未知指令，将播放默认反馈")
            print("-" * 50)

    try:
        count = 0
        while count < 3:
            raw_bytes = asr.ser.readline()
            if not raw_bytes:
                continue
            line = raw_bytes.decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            print(f"[ASR_RX] {line}")

            parse_res = asr.parse_line(line)
            if parse_res is None:
                continue
            if parse_res[0] == "ready":
                print("[ASR 模块就绪]")
                continue
            if parse_res[0] == "asr":
                _, snid, text, action_name = parse_res
                handle_voice_command(snid, text, action_name)
                
                if action_name != "wake":
                    audio_id = asr.play_by_snid(snid)
                    print(f"[语音播报] <PLAY>{audio_id}")
                
                count += 1
                time.sleep(0.5)

        print("\n✅ 模拟测试完成！")
        asr.close()

    except Exception as e:
        print(f"\n❌ 测试异常: {e}")


if __name__ == "__main__":
    print("=" * 60)
    print("ASRPRO 语音模块测试程序")
    print("=" * 60)

    try:
        asr = ASRVoiceSerial(port="/dev/ttyS1", baud=115200, timeout=0.2)
        asr.open()

        print("测试说明：")
        print("1. 串口 /dev/ttyS1 已打开，波特率 115200")
        print("2. 等待语音识别模块发送指令")
        print("3. 当您说'左转'时，串口将收到 <ASR>3,向左转")
        print("4. 程序解析后会发送 <PLAY>20003 给语音模块")
        print("5. 语音模块将播报'向左转'")
        print("=" * 60)

        def handle_voice_command(snid, text, action_name):
            print("\n" + "-" * 50)
            print(f"[指令解析]")
            print(f"  SNID: {snid}")
            print(f"  识别文本: {text}")
            print(f"  动作名称: {action_name}")

            if action_name == "turn_left":
                print("  ✅ 检测到'左转'指令")
                print("  ✅ 将发送 <PLAY>20003 给语音模块播报'向左转'")
            elif action_name == "turn_right":
                print("  ✅ 检测到'右转'指令")
                print("  ✅ 将发送 <PLAY>20004 给语音模块播报'向右转'")
            elif action_name == "forward":
                print("  ✅ 检测到'前进'指令")
                print("  ✅ 将发送 <PLAY>20001 给语音模块播报'向前走'")
            elif action_name == "backward":
                print("  ✅ 检测到'后退'指令")
                print("  ✅ 将发送 <PLAY>20002 给语音模块播报'向后退'")
            elif action_name == "stop":
                print("  ✅ 检测到'停止'指令")
                print("  ✅ 将发送 <PLAY>20005 给语音模块播报'停一下'")
            elif action_name == "wake":
                print("  ℹ️ 检测到唤醒词，不额外播报")
            else:
                print(f"  🔄 未知指令，将播放默认反馈")
            print("-" * 50)

        asr.listen(callback=handle_voice_command)

    except serial.SerialException as e:
        print(f"\n❌ 串口打开失败: {e}")
        print("切换到模拟测试模式...")
        test_with_mock_serial()
    except KeyboardInterrupt:
        print("\n\n✅ 测试结束，正在关闭串口...")
        if 'asr' in dir():
            asr.close()
        print("✅ 串口已关闭")
    except Exception as e:
        print(f"\n❌ 测试异常: {e}")
        if 'asr' in dir():
            asr.close()