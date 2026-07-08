import time
from pca9685 import PCA9685, PCA_MAX_RAW

# ===================== 舵机全局常量 =====================
SERVO_PWM_FREQ = 50
SERVO_MIN_ANGLE = 0
SERVO_MAX_ANGLE = 180
SERVO_MIN_PULSE = 0.5
SERVO_MAX_PULSE = 2.5
SERVO_PERIOD = 20.0


class ServoError(Exception):
    pass


class ServoInitError(ServoError):
    pass


class SingleServo:
    """单个舵机实例，仅内部ServoController使用"""
    def __init__(self, pca_driver:PCA9685, pwm_ch, name="Servo"):
        self.pca = pca_driver
        self.pwm_ch = pwm_ch
        self.name = name

        self._angle = 90
        self._enabled = False

        self._pwm_min = int(PCA_MAX_RAW * SERVO_MIN_PULSE / SERVO_PERIOD)
        self._pwm_max = int(PCA_MAX_RAW * SERVO_MAX_PULSE / SERVO_PERIOD)
        self._pwm_range = self._pwm_max - self._pwm_min

        self.set_angle(90)
        self.enable()

    def _angle_to_pwm(self, angle):
        angle = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, angle))
        ratio = (angle - SERVO_MIN_ANGLE) / (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE)
        return self._pwm_min + int(ratio * self._pwm_range)

    def _pwm_to_angle(self, pwm_val):
        ratio = (pwm_val - self._pwm_min) / self._pwm_range
        return SERVO_MIN_ANGLE + ratio * (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE)

    def set_angle(self, angle):
        if not SERVO_MIN_ANGLE <= angle <= SERVO_MAX_ANGLE:
            raise ValueError(f"角度范围 {SERVO_MIN_ANGLE} ~ {SERVO_MAX_ANGLE}")

        self._angle = angle
        pwm_val = self._angle_to_pwm(angle)

        if self._enabled:
            self.pca.set_raw_pwm(self.pwm_ch, 0, pwm_val)

    def get_angle(self):
        return self._angle

    def enable(self):
        self._enabled = True
        pwm_val = self._angle_to_pwm(self._angle)
        self.pca.set_raw_pwm(self.pwm_ch, 0, pwm_val)

    def disable(self):
        self._enabled = False
        self.pca.single_channel_zero(self.pwm_ch)

    def is_enabled(self):
        return self._enabled

    def get_pwm_value(self):
        on_val, off_val = self.pca.get_channel_raw(self.pwm_ch)
        return off_val

    def cleanup(self):
        self.disable()


class ServoController:
    # ===================== 舵机全局常量（类静态常量） =====================
    SERVO_PWM_FREQ = 50
    SERVO_MIN_ANGLE = 0
    SERVO_MAX_ANGLE = 180
    SERVO_MIN_PULSE = 0.5
    SERVO_MAX_PULSE = 2.5
    SERVO_PERIOD = 20.0

    def __init__(self, pca_driver:PCA9685,debug=False):
        """
        舵机控制器，内置PCA9685驱动，与CarMotor格式统一
        :param debug: 是否开启PCA调试输出
        """
        self.pca = pca_driver
        self.pca.set_pwm_freq(self.SERVO_PWM_FREQ)
        self.servos = {}

    def all_channel_set_angle(self, angle, hold_sec=3):
        """仅8~11通道输出舵机角度波形，其他通道关闭"""
        angle = max(self.SERVO_MIN_ANGLE, min(self.SERVO_MAX_ANGLE, angle))
        # 计算对应raw值
        pwm_min = int(PCA_MAX_RAW * self.SERVO_MIN_PULSE / self.SERVO_PERIOD)
        pwm_max = int(PCA_MAX_RAW * self.SERVO_MAX_PULSE / self.SERVO_PERIOD)
        pwm_range = pwm_max - pwm_min
        ratio = (angle - self.SERVO_MIN_ANGLE) / (self.SERVO_MAX_ANGLE - self.SERVO_MIN_ANGLE)
        target_raw = pwm_min + int(ratio * pwm_range)

        print(f"\n===== 通道8~11测试 角度={angle}° 持续{hold_sec}s =====")
        # 先全部通道归零
        for ch in range(16):
            self.pca.single_channel_zero(ch)
        # 只开启 8,9,10,11
        for ch in [8,9,10,11]:
            self.pca.set_raw_pwm(ch, 0, target_raw)
        print(f"通道8/9/10/11已输出舵机PWM，Raw值={target_raw}")
        time.sleep(hold_sec)
        # 结束再次全部归零
        for ch in range(16):
            self.pca.single_channel_zero(ch)
        print("8~11通道测试完成，所有通道关闭\n")

    def add_servo(self, name, pwm_ch):
        servo = SingleServo(
            pca_driver=self.pca,
            pwm_ch=pwm_ch,
            name=name
        )
        self.servos[name] = servo
        return servo

    def get_servo(self, name):
        return self.servos.get(name)

    def disable_all(self):
        for servo in self.servos.values():
            servo.disable()

    def cleanup(self):
        self.disable_all()
        for servo in self.servos.values():
            servo.cleanup()
        self.pca.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.cleanup()