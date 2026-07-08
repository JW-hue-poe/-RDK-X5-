import serial
import serial.tools.list_ports
import time
import threading

# --------------------------串口基础配置--------------------------
BAUD_RATE = 9600
DATA_BITS = 8
STOP_BITS = 1
PARITY = serial.PARITY_NONE
READ_TIMEOUT = 0.3
# ----------------------------------------------------------------

class LoraDevice:
    def __init__(self, port_name):
        self.port_name = port_name
        self.ser = None
        self.running = False
        self.recv_buffer = []  # 缓存收到的完整帧数据
        self.recv_thread = None

    def open(self):
        """打开串口，启动后台接收线程"""
        try:
            self.ser = serial.Serial(
                port=self.port_name,
                baudrate=BAUD_RATE,
                bytesize=DATA_BITS,
                stopbits=STOP_BITS,
                parity=PARITY,
                timeout=READ_TIMEOUT
            )
            self.running = True
            # 开启子线程持续接收数据
            self.recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
            self.recv_thread.start()
            print(f"✅ LoRa串口 {self.port_name} 打开成功")
            return True
        except serial.SerialException as e:
            print(f"❌ 串口打开失败: {e}")
            return False

    def _recv_loop(self):
        """后台循环接收数据（私有）"""
        while self.running:
            recv_data = self.ser.read_all()
            if len(recv_data) > 0:
                # 过滤1/2字节干扰杂波
                if len(recv_data) <= 2:
                    continue
                # 存入接收缓存供外部读取
                self.recv_buffer.append(recv_data)
            time.sleep(0.01)

    def send(self, text: str):
        """
        对外发送接口
        :param text: 要发送的字符串
        :return: True发送成功 / False失败
        """
        if self.ser is None or not self.ser.is_open:
            print("LoRa串口未打开，无法发送")
            return False
        try:
            send_bytes = text.encode("utf-8")
            self.ser.write(send_bytes)
            time.sleep(0.05)
            send_hex = " ".join(f"{b:02X}" for b in send_bytes)
            print(f"【发送】文本:{text} | HEX:{send_hex}")
            return True
        except Exception as e:
            print(f"发送失败: {e}")
            return False

    def get_recv_data(self):
        """
        对外读取接收缓存接口
        :return: list[bytes] 所有未读取的帧，读完清空缓存
        """
        data_list = self.recv_buffer.copy()
        self.recv_buffer.clear()
        return data_list

    def close(self):
        """关闭串口，停止接收线程"""
        self.running = False
        if self.recv_thread is not None:
            self.recv_thread.join(timeout=0.5)
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            print("✅ LoRa串口已关闭")


# 工具函数：枚举并选择串口
def list_all_com():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("未检测到任何串口设备！检查USB转TTL是否插好、驱动是否正常")
        return None
    print("===== 当前可用串口列表 =====")
    for idx, port in enumerate(ports):
        print(f"[{idx}] {port.device}  | 描述：{port.description}")
    return ports

def select_serial_port(ports):
    while True:
        try:
            idx = input("\n请输入要使用的串口序号，直接回车选第0个：")
            if idx == "":
                idx = 0
            idx = int(idx)
            if 0 <= idx < len(ports):
                return ports[idx].device
            else:
                print("输入序号超出范围，请重新输入！")
        except ValueError:
            print("输入错误，请输入数字！")


# 测试主程序（演示外部如何调用LoraDevice类接口）
if __name__ == "__main__":
    port_list = list_all_com()
    if port_list is None:
        exit(1)
    port = select_serial_port(port_list)

    # 实例化LoRa对象
    lora = LoraDevice(port)
    if not lora.open():
        exit(1)

    print("\n👉 输入文字回车发送，输入 quit 退出程序")
    while True:
        # 主动读取收到的数据
        recv_frames = lora.get_recv_data()
        for frame in recv_frames:
            print("-" * 40)
            print(f"原始字节: {frame}")
            hex_str = " ".join([f"{byte:02X}" for byte in frame])
            print(f"【LoRa接收十六进制帧】{hex_str}")
            try:
                text = frame.decode("utf-8")
                print(f"解析文本: {text}")
            except UnicodeDecodeError:
                print("解析文本: 二进制数据，无法转UTF8")
            print("-" * 40 + "\n")

        # 控制台输入发送
        send_msg = input("请输入发送内容：")
        if send_msg.strip().lower() == "quit":
            break
        lora.send(send_msg)

    # 退出关闭资源
    lora.close()