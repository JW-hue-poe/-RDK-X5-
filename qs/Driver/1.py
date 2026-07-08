import time


class Gripper360:
    """
    360°连续旋转舵机夹爪控制类。

    当前已标定：
    停止死区：1630us ~ 1730us
    稳定停止值：1680us
    低脉宽 1500us：张开
    高脉宽 1850us：合上
    """

    def __init__(
        self,
        pca,
        channel=0,
        stop_us=1680,
        open_us=1500,
        close_us=1850,
        default_run_time=0.25,
        name="gripper"
    ):
        self.pca = pca
        self.channel = channel
        self.stop_us = stop_us
        self.open_us = open_us
        self.close_us = close_us
        self.default_run_time = default_run_time
        self.name = name

    def stop(self):
        self.pca.set_pwm_us(self.channel, self.stop_us, freq_hz=50)
        print(f"[{self.name}] stop: {self.stop_us}us")

    def open_for(self, seconds=None):
        if seconds is None:
            seconds = self.default_run_time

        print(f"[{self.name}] open_for {seconds}s, pulse={self.open_us}us")
        self.pca.set_pwm_us(self.channel, self.open_us, freq_hz=50)
        time.sleep(seconds)
        self.stop()

    def close_for(self, seconds=None):
        if seconds is None:
            seconds = self.default_run_time

        print(f"[{self.name}] close_for {seconds}s, pulse={self.close_us}us")
        self.pca.set_pwm_us(self.channel, self.close_us, freq_hz=50)
        time.sleep(seconds)
        self.stop()

    def open_once(self):
        self.open_for(self.default_run_time)

    def close_once(self):
        self.close_for(self.default_run_time)

    def release(self):
        self.pca.set_raw_pwm(self.channel, 0, 0)
        print(f"[{self.name}] release CH{self.channel}")


if __name__ == "__main__":
    # 导入PCA9685驱动，修改为你实际的导入路径
    from pca9685 import PCA9685

    print("===== 360°旋转夹爪舵机测试程序 =====")
    try:
        # 初始化PCA9685 I2C总线5，地址0x40，和你机器人硬件一致
        pca = PCA9685(bus_num=5, addr=0x40, debug=False)
        pca.set_pwm_freq(50)

        # 实例化夹爪，根据你的硬件修改channel通道号
        gripper = Gripper360(pca=pca, channel=15, default_run_time=0.3)

        print("\n1. 执行单次张开夹爪")
        gripper.open_once()
        time.sleep(1)

        print("\n2. 执行单次闭合夹爪")
        gripper.close_once()
        time.sleep(1)

        print("\n3. 自定义时长张开0.5秒")
        gripper.open_for(0.5)
        time.sleep(1)

        print("\n4. 自定义时长闭合0.5秒")
        gripper.close_for(0.5)
        time.sleep(1)

        print("\n测试完成，释放舵机通道")
        gripper.release()

    except KeyboardInterrupt:
        print("\n\n用户手动终止测试，释放舵机")
        gripper.release()
    except Exception as e:
        print(f"\n测试异常：{e}")
    finally:
        if "pca" in locals() and pca is not None:
            pca.close()
        print("PCA9685硬件已关闭")