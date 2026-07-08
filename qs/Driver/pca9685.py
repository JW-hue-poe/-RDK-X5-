import time
import os
import sys
from smbus2 import SMBus

# ===================== PCA9685 寄存器常量 =====================
PCA_ADDR = 0x40
PCA_MODE1 = 0x00
PCA_MODE2 = 0x01
PCA_PRESCALE = 0xFE
PCA_LED0_ON_L = 0x06

PCA_MAX_RAW = 4095  # PCA原生12位PWM范围 0~4095

# ===================== 自定义异常 =====================
class PCA9685Error(Exception):
    pass

class PCA9685InitError(PCA9685Error):
    pass

class PCA9685CommError(PCA9685Error):
    pass

# ===================== 工具函数：I2C检测 =====================
def check_i2c_permission(bus_num):
    dev_path = f"/dev/i2c-{bus_num}"
    if not os.path.exists(dev_path):
        return False, f"I2C总线 {dev_path} 不存在"
    if not os.access(dev_path, os.R_OK | os.W_OK):
        return False, "无读写权限，请sudo运行或加入i2c用户组"
    return True, ""

def validate_i2c_dev(bus_num, target_addr):
    try:
        bus = SMBus(bus_num)
        bus.read_byte_data(target_addr, PCA_MODE1)
        bus.close()
        return True, ""
    except OSError as e:
        if e.errno == 121:
            return False, f"0x{target_addr:02X} 设备无响应，检查硬件接线"
        elif e.errno == 13:
            return False, "权限不足"
        elif e.errno == 2:
            return False, f"I2C总线 {bus_num} 不存在"
        else:
            return False, str(e)

# ===================== 独立 PCA9685 底层驱动类 =====================
class PCA9685:
    def __init__(self, bus_num=5, addr=PCA_ADDR, debug=False):
        self.bus_num = bus_num
        self.addr = addr
        self.debug = debug
        self._bus = None

        # 前置硬件校验
        self._pre_check()
        # 打开I2C总线
        self._open_bus()
        # 芯片基础初始化
        self._chip_init()
        # 初始化全部16通道输出
        self.all_channel_zero()

    def _pre_check(self):
        """I2C权限+设备存在性校验"""
        perm_ok, msg = check_i2c_permission(self.bus_num)
        if not perm_ok:
            raise PCA9685InitError(msg)
        dev_ok, msg = validate_i2c_dev(self.bus_num, self.addr)
        if not dev_ok:
            raise PCA9685InitError(msg)

    def _open_bus(self):
        try:
            self._bus = SMBus(self.bus_num)
            if self.debug:
                print(f"[PCA DEBUG] I2C-{self.bus_num} 总线打开成功")
        except OSError as e:
            raise PCA9685InitError(f"打开I2C失败: {e}")

    def _chip_init(self):
        """芯片基础唤醒，默认推挽输出"""
        self._bus.write_byte_data(self.addr, PCA_MODE1, 0x00)
        time.sleep(0.005)
        # MODE2=0x04 推挽输出
        self._bus.write_byte_data(self.addr, PCA_MODE2, 0x04)
        time.sleep(0.005)

        mode1 = self._bus.read_byte_data(self.addr, PCA_MODE1)
        mode2 = self._bus.read_byte_data(self.addr, PCA_MODE2)

        if self.debug:
            print("[PCA DEBUG] 芯片基础初始化完成")
            print(f"[PCA DEBUG] MODE1: 0x{mode1:02X}, MODE2: 0x{mode2:02X}")
            if mode1 & 0x10:
                print("[PCA DEBUG] ⚠ 芯片仍处于休眠状态 (SLEEP bit=1)")
            if mode2 != 0x04:
                print(f"[PCA DEBUG] ⚠ MODE2预期0x04，实际0x{mode2:02X}")

    def set_pwm_freq(self, freq: int):
        """
        接口：设置PWM全局频率
        :param freq: 24 ~ 1526 Hz
        """
        if not 24 <= freq <= 1526:
            raise ValueError("频率范围仅支持 24 ~ 1526 Hz")
        prescale = int(25000000 / (PCA_MAX_RAW * freq) - 1)
        prescale = max(3, min(255, prescale))

        old_mode = self._bus.read_byte_data(self.addr, PCA_MODE1)
        # 进入休眠写预分频
        self._bus.write_byte_data(self.addr, PCA_MODE1, old_mode | 0x10)
        time.sleep(0.005)
        self._bus.write_byte_data(self.addr, PCA_PRESCALE, prescale)
        time.sleep(0.005)
        # 退出休眠，开启地址自增AI
        new_mode = (old_mode & ~0x10) | 0x20
        self._bus.write_byte_data(self.addr, PCA_MODE1, new_mode)
        time.sleep(0.005)

        mode1_final = self._bus.read_byte_data(self.addr, PCA_MODE1)
        mode2_final = self._bus.read_byte_data(self.addr, PCA_MODE2)
        prescale_read = self._bus.read_byte_data(self.addr, PCA_PRESCALE)

        if self.debug:
            print(f"[PCA DEBUG] 设置PWM频率={freq}Hz 预分频={prescale}")
            print(f"[PCA DEBUG] MODE1: 0x{mode1_final:02X}, MODE2: 0x{mode2_final:02X}")
            print(f"[PCA DEBUG] PRESCALE: 0x{prescale_read:02X} (预期0x{prescale:02X})")
            if mode1_final & 0x10:
                print("[PCA DEBUG] ⚠ 芯片仍处于休眠状态 (SLEEP bit=1)")
            if not (mode1_final & 0x20):
                print("[PCA DEBUG] ⚠ 地址自增未启用 (AI bit=0)")
            if prescale_read != prescale:
                print(f"[PCA DEBUG] ⚠ 预分频写入失败，预期0x{prescale:02X}，实际0x{prescale_read:02X}")

    def set_raw_pwm(self, ch: int, on: int, off: int):
        """
        底层接口：直接写入原生12位PWM寄存器
        :param ch: 通道 0~15
        :param on: 上升沿点 0~4095
        :param off: 下降沿点 0~4095
        """
        if not 0 <= ch <= 15:
            raise ValueError("通道必须 0~15")
        if not (0 <= on <= PCA_MAX_RAW and 0 <= off <= PCA_MAX_RAW):
            raise ValueError("on/off 范围 0~4095")
        reg = PCA_LED0_ON_L + 4 * ch
        self._bus.write_byte_data(self.addr, reg, on & 0xFF)
        self._bus.write_byte_data(self.addr, reg + 1, on >> 8)
        self._bus.write_byte_data(self.addr, reg + 2, off & 0xFF)
        self._bus.write_byte_data(self.addr, reg + 3, off >> 8)

    def set_duty_cycle(self, ch: int, duty: float):
        """
        上层接口：设置占空比
        :param ch: 通道0~15
        :param duty: 0.0 ~ 1.0
        """
        duty = max(0.0, min(1.0, duty))
        raw_val = int(duty * PCA_MAX_RAW)
        self.set_raw_pwm(ch, 0, raw_val)
        if self.debug:
            print(f"[PCA DEBUG] CH{ch} 占空比={duty:.2f} raw={raw_val}")

    def get_channel_raw(self, ch: int) -> tuple[int, int]:
        """读取通道当前on/off原生值"""
        reg = PCA_LED0_ON_L + 4 * ch
        on_l = self._bus.read_byte_data(self.addr, reg)
        on_h = self._bus.read_byte_data(self.addr, reg + 1)
        off_l = self._bus.read_byte_data(self.addr, reg + 2)
        off_h = self._bus.read_byte_data(self.addr, reg + 3)
        on = (on_h << 8) | on_l
        off = (off_h << 8) | off_l
        return on, off

    def single_channel_zero(self, ch: int):
        """【修复】通道输出全低电平，电机自由停止，不再刹车锁死"""
        self.set_raw_pwm(ch, 0, 0)

    def all_channel_zero(self):
        """【修复】全部16通道输出低电平"""
        for ch in range(16):
            self.single_channel_zero(ch)
        if self.debug:
            print("[PCA DEBUG] 全部16通道清零完成")

    def close(self):
        """释放总线资源"""
        if self._bus:
            self.all_channel_zero()
            self._bus.close()
            self._bus = None
            if self.debug:
                print("[PCA DEBUG] I2C总线已关闭")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def __del__(self):
        if hasattr(self, '_bus') and self._bus:
            self.close()

if __name__ == "__main__":
    print("=" * 60)
    print("舵机8-11保持20度 + 四轮电机同步直行测试程序")
    print("=" * 60)

    debug_mode = len(sys.argv) > 1 and sys.argv[1] == "--debug"

    try:
        with PCA9685(bus_num=5, addr=0x40) as pca:
            # 3. 电机前进，此时8~11舵机依然保持20度输出
            car_forward(pca)
            time.sleep(2000)
            
            # 停止电机，舵机8-11依旧保持20°
            car_stop(pca)
            time.sleep(1)

    except PCA9685InitError as e:
        print(f"\n❌ PCA9685初始化失败: {e}")
        print("\n排查建议:")
        print("1. 检查PCA9685硬件连接: VCC接3.3V, V+接5V, GND共地")
        print("2. 确认OE引脚接GND（使能输出）")
        print("3. 确认SDA/SCL连接到正确的I2C引脚")
        print("4. 运行 'i2cdetect -y 5' 确认设备地址")
        sys.exit(1)

    except KeyboardInterrupt:
        print("\n\n✋ 手动终止程序，全车电机停止")

    except Exception as e:
        print(f"\n❌ 未知错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)