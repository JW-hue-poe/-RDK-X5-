from pca9685 import PCA9685
import os
import time

class CarMotor:
    # ===================== PCA AIN/BIN 方向通道 =====================
    CH_LB_A = 0  # 左后轮 AIN
    CH_LB_B = 1  # 左后轮 BIN
    CH_RB_A = 2  # 右后轮 AIN
    CH_RB_B = 3  # 右后轮 BIN
    CH_LF_A = 4  # 左前轮 AIN
    CH_LF_B = 5  # 左前轮 BIN
    CH_RF_A = 6  # 右前轮 AIN
    CH_RF_B = 7  # 右前轮 BIN

    # ===================== SYSFS PWM 配置 =====================
    PWM_CHIP = 0
    PWM_A_ID = 0  # PWM6 物理32脚 PWMA
    PWM_B_ID = 1  # PWM7 物理31脚 PWMB
    PWM_PERIOD_NS = 1000000  # 1000Hz 周期1ms
    MOTOR_SPEED_DUTY = 650000  # 65%占空比
    DIR_HIGH = 1.0
    DIR_LOW = 0.0

    def __init__(self, pca_driver: PCA9685, debug=False):
        self.pca = pca_driver
        self.debug = debug
        self.chip_path = f"/sys/class/pwm/pwmchip{self.PWM_CHIP}"

        # 先判断pwmchip是否存在
        if not os.path.exists(self.chip_path):
            raise RuntimeError(f"PWM芯片不存在：{self.chip_path}，内核未启用硬件PWM")

        # 导出两路PWM通道
        self._pwm_export(self.PWM_A_ID)
        self._pwm_export(self.PWM_B_ID)
        time.sleep(0.05)

        # 配置周期、初始占空比0、开启PWM
        self._set_pwm_period(self.PWM_A_ID, self.PWM_PERIOD_NS)
        self._set_pwm_period(self.PWM_B_ID, self.PWM_PERIOD_NS)
        self._set_pwm_duty(self.PWM_A_ID, 0)
        self._set_pwm_duty(self.PWM_B_ID, 0)
        self._pwm_enable(self.PWM_A_ID, 1)
        self._pwm_enable(self.PWM_B_ID, 1)

    # ---------------------- 修复后的sysfs底层读写 ----------------------
    def _write_global_pwm_file(self, filename: str, value):
        """写入pwmchip根目录export/unexport"""
        path = os.path.join(self.chip_path, filename)
        with open(path, "w") as f:
            f.write(str(value))

    def _write_channel_pwm_file(self, pwm_id: int, filename: str, value):
        """写入pwm0/pwm1子目录文件"""
        ch_path = os.path.join(self.chip_path, f"pwm{pwm_id}", filename)
        with open(ch_path, "w") as f:
            f.write(str(value))

    def _pwm_export(self, pwm_id: int):
        ch_folder = os.path.join(self.chip_path, f"pwm{pwm_id}")
        if not os.path.exists(ch_folder):
            self._write_global_pwm_file("export", pwm_id)
            if self.debug:
                print(f"[PWM] 导出通道 pwm{pwm_id}")

    def _pwm_unexport(self, pwm_id: int):
        self._write_global_pwm_file("unexport", pwm_id)
        if self.debug:
            print(f"[PWM] 取消导出通道 pwm{pwm_id}")

    def _set_pwm_period(self, pwm_id: int, period_ns: int):
        self._write_channel_pwm_file(pwm_id, "period", period_ns)

    def _set_pwm_duty(self, pwm_id: int, duty_ns: int):
        self._write_channel_pwm_file(pwm_id, "duty_cycle", duty_ns)

    def _pwm_enable(self, pwm_id: int, enable: int):
        self._write_channel_pwm_file(pwm_id, "enable", enable)

    # ---------------------- 电机基础操作 ----------------------
    def _clear_all_dir(self):
        dir_channels = [
            self.CH_LB_A, self.CH_LB_B, self.CH_RB_A, self.CH_RB_B,
            self.CH_LF_A, self.CH_LF_B, self.CH_RF_A, self.CH_RF_B
        ]
        for ch in dir_channels:
            self.pca.single_channel_zero(ch)

    def car_stop(self):
        self._clear_all_dir()
        self._set_pwm_duty(self.PWM_A_ID, 0)
        self._set_pwm_duty(self.PWM_B_ID, 0)
        if self.debug:
            print("[Motor] 全车电机停止")

    def car_forward(self):
        # 左侧反转
        self.pca.set_duty_cycle(self.CH_LB_A, self.DIR_LOW)
        self.pca.set_duty_cycle(self.CH_LB_B, self.DIR_HIGH)
        self.pca.set_duty_cycle(self.CH_LF_A, self.DIR_LOW)
        self.pca.set_duty_cycle(self.CH_LF_B, self.DIR_HIGH)
        # 右侧反转
        self.pca.set_duty_cycle(self.CH_RB_A, self.DIR_HIGH)
        self.pca.set_duty_cycle(self.CH_RB_B, self.DIR_LOW)
        self.pca.set_duty_cycle(self.CH_RF_A, self.DIR_HIGH)
        self.pca.set_duty_cycle(self.CH_RF_B, self.DIR_LOW)
        self._set_pwm_duty(self.PWM_A_ID, self.MOTOR_SPEED_DUTY)
        self._set_pwm_duty(self.PWM_B_ID, self.MOTOR_SPEED_DUTY)
        
        if self.debug:
            print("[Motor] 车辆直行")

    def car_backward(self):
        # 左侧正转方向
        self.pca.set_duty_cycle(self.CH_LB_A, self.DIR_HIGH)
        self.pca.set_duty_cycle(self.CH_LB_B, self.DIR_LOW)
        self.pca.set_duty_cycle(self.CH_LF_A, self.DIR_HIGH)
        self.pca.set_duty_cycle(self.CH_LF_B, self.DIR_LOW)
        # 右侧正转方向
        self.pca.set_duty_cycle(self.CH_RB_A, self.DIR_LOW)
        self.pca.set_duty_cycle(self.CH_RB_B, self.DIR_HIGH)
        self.pca.set_duty_cycle(self.CH_RF_A, self.DIR_LOW)
        self.pca.set_duty_cycle(self.CH_RF_B, self.DIR_HIGH)
        # 调速PWM
        self._set_pwm_duty(self.PWM_A_ID, self.MOTOR_SPEED_DUTY)
        self._set_pwm_duty(self.PWM_B_ID, self.MOTOR_SPEED_DUTY)

        if self.debug:
            print("[Motor] 车辆后退")

    def car_left_group_run(self):
        self.pca.set_duty_cycle(self.CH_LB_A, self.DIR_HIGH)
        self.pca.set_duty_cycle(self.CH_LB_B, self.DIR_LOW)
        self.pca.set_duty_cycle(self.CH_LF_A, self.DIR_LOW)
        self.pca.set_duty_cycle(self.CH_LF_B, self.DIR_HIGH)
        # 右侧反转
        self.pca.set_duty_cycle(self.CH_RB_A, self.DIR_LOW)
        self.pca.set_duty_cycle(self.CH_RB_B, self.DIR_HIGH)
        self.pca.set_duty_cycle(self.CH_RF_A, self.DIR_HIGH)
        self.pca.set_duty_cycle(self.CH_RF_B, self.DIR_LOW)
        self._set_pwm_duty(self.PWM_A_ID, self.MOTOR_SPEED_DUTY)
        self._set_pwm_duty(self.PWM_B_ID, self.MOTOR_SPEED_DUTY)
        
        if self.debug:
            print("[Motor] 原地左转")

    def car_right_group_run(self):
        # 左反转、右正转
        self.pca.set_duty_cycle(self.CH_LB_A, self.DIR_LOW)
        self.pca.set_duty_cycle(self.CH_LB_B, self.DIR_HIGH)
        self.pca.set_duty_cycle(self.CH_LF_A, self.DIR_HIGH)
        self.pca.set_duty_cycle(self.CH_LF_B, self.DIR_LOW)
        # 右侧反转
        self.pca.set_duty_cycle(self.CH_RB_A, self.DIR_HIGH)
        self.pca.set_duty_cycle(self.CH_RB_B, self.DIR_LOW)
        self.pca.set_duty_cycle(self.CH_RF_A, self.DIR_HIGH)
        self.pca.set_duty_cycle(self.CH_RF_B, self.DIR_LOW)

        self._set_pwm_duty(self.PWM_A_ID, self.MOTOR_SPEED_DUTY)
        self._set_pwm_duty(self.PWM_B_ID, self.MOTOR_SPEED_DUTY)
        if self.debug:
            print("[Motor] 原地右转")

    def cleanup(self):
        self.car_stop()
        self._pwm_enable(self.PWM_A_ID, 0)
        self._pwm_enable(self.PWM_B_ID, 0)
        self._pwm_unexport(self.PWM_A_ID)
        self._pwm_unexport(self.PWM_B_ID)
        if self.debug:
            print("[PWM] 资源释放完成")

# 测试入口
if __name__ == "__main__":
    import time
    import sys
    from pca9685 import PCA9685, PCA9685InitError

    print("=" * 60)
    print("电机测试 | PCA控制方向 + /sys/class/pwm 文件调速")
    print("=" * 60)
    debug_flag = len(sys.argv) > 1 and sys.argv[1] == "--debug"

    try:
        with PCA9685(bus_num=5, addr=0x40, debug=debug_flag) as pca:
            pca.set_pwm_freq(50)
            motor = CarMotor(pca_driver=pca, debug=debug_flag)

            print("\n>>> 直行2s")
            motor.car_forward()
            time.sleep(2900)

            print(">>> 停止1s")
            motor.car_stop()
            time.sleep(1)

            print("\n>>> 后退2s")
            motor.car_backward()
            time.sleep(2)

            print(">>> 停止1s")
            motor.car_stop()
            time.sleep(1)

            print("\n>>> 原地左转2s")
            motor.car_left_group_run()
            time.sleep(2)

            print(">>> 停止1s")
            motor.car_stop()
            time.sleep(1)

            print("\n>>> 原地右转2s")
            motor.car_right_group_run()
            time.sleep(2)

            print("\n>>> 测试完成，释放资源")
            motor.car_stop()
            motor.cleanup()

    except PCA9685InitError as e:
        print(f"\n❌ PCA9685初始化失败: {e}")
        sys.exit(1)
    except RuntimeError as e:
        print(f"\n❌ PWM硬件初始化失败: {e}")
        print("排查：1. 内核未加载PWM驱动 2. 设备树未开启PWM6/PWM7")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n✋ 手动终止，紧急停机释放PWM")
        try:
            temp_pca = PCA9685(5,0x40)
            temp_motor = CarMotor(temp_pca)
            temp_motor.car_stop()
            temp_motor.cleanup()
        except:
            pass
        sys.exit(0)
    except Exception as e:
        print(f"\n❌ 运行错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)