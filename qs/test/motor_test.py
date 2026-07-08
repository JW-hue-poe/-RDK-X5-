#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import signal
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from Driver.motor import MotorController, MotorPWMError, MotorGPIOError
from Driver.pca9685 import PCA9685InitError

# ========== 电机配置参数 ==========
# PWM通道分配：
#   左侧电机组（左前、左后）共用 PWM通道0 (物理Pin32)
#   右侧电机组（右前、右后）共用 PWM通道1 (物理Pin33)
LEFT_PWM_CHIP = 0
LEFT_PWM_CHANNEL = 0
RIGHT_PWM_CHIP = 0
RIGHT_PWM_CHANNEL = 1

# 电机方向控制通道（PCA9685）
LF_AIN_CH = 0
LF_BIN_CH = 1
RF_AIN_CH = 2
RF_BIN_CH = 3
LB_AIN_CH = 4
LB_BIN_CH = 5
RB_AIN_CH = 6
RB_BIN_CH = 7

# 编码器GPIO引脚（BCM编号）
LF_ENC_A = 17
LF_ENC_B = 27
RF_ENC_A = 22
RF_ENC_B = 23
LB_ENC_A = 5
LB_ENC_B = 6
RB_ENC_A = 26
RB_ENC_B = 16

# 全局电机控制器对象，用于信号捕获停止
mc_global = None

def sigint_handler(signum, frame):
    """捕获Ctrl+C，安全停止所有电机"""
    print("\n\n检测到 Ctrl+C，正在停止所有电机...")
    if mc_global is not None:
        mc_global.stop_all()
    print("电机已全部停止，程序退出")
    sys.exit(0)

def init_motor_controller(debug=False):
    """初始化电机控制器并添加四个电机"""
    mc = MotorController(bus_num=5, addr=0x40, debug=debug)

    mc.add_motor(
        name="左前电机",
        pwm_chip=LEFT_PWM_CHIP,
        pwm_channel=LEFT_PWM_CHANNEL,
        ain_ch=LF_AIN_CH,
        bin_ch=LF_BIN_CH,
        enc_a_pin=LF_ENC_A,
        enc_b_pin=LF_ENC_B
    )

    mc.add_motor(
        name="右前电机",
        pwm_chip=RIGHT_PWM_CHIP,
        pwm_channel=RIGHT_PWM_CHANNEL,
        ain_ch=RF_AIN_CH,
        bin_ch=RF_BIN_CH,
        enc_a_pin=RF_ENC_A,
        enc_b_pin=RF_ENC_B
    )

    mc.add_motor(
        name="左后电机",
        pwm_chip=LEFT_PWM_CHIP,
        pwm_channel=LEFT_PWM_CHANNEL,
        ain_ch=LB_AIN_CH,
        bin_ch=LB_BIN_CH,
        enc_a_pin=LB_ENC_A,
        enc_b_pin=LB_ENC_B
    )

    mc.add_motor(
        name="右后电机",
        pwm_chip=RIGHT_PWM_CHIP,
        pwm_channel=RIGHT_PWM_CHANNEL,
        ain_ch=RB_AIN_CH,
        bin_ch=RB_BIN_CH,
        enc_a_pin=RB_ENC_A,
        enc_b_pin=RB_ENC_B
    )

    return mc


def run_forever(mc, speed=50):
    """四个电机持续一直向前运行，循环打印状态"""
    print("=" * 60)
    print("四电机持续直行程序")
    print(f"运行速度: {speed}%")
    print("按下 Ctrl+C 安全停止电机并退出")
    print("=" * 60)

    motors = list(mc.motors.values())

    # 启动所有电机向前
    print("\n启动全部四个电机向前运行...")
    for m in motors:
        m.set_speed(speed)
        print(f"{m.name} 已启动，速度 {speed}%")

    print("\n电机持续运行中，每秒打印一次编码器计数：")
    print("-" * 80)

    # 无限循环持续运行
    while True:
        info_lines = []
        for m in motors:
            cnt = m.get_encoder_count()
            info_lines.append(f"{m.name:<10} 计数:{cnt:<8}")
        print("\r" + "  ".join(info_lines), end="")
        sys.stdout.flush()
        time.sleep(1)


def main():
    global mc_global
    print("=" * 60)
    print("电机持续直行程序")
    print("版本: 1.0")
    print("日期:", time.strftime("%Y-%m-%d %H:%M:%S"))
    print("=" * 60)

    debug_mode = len(sys.argv) > 1 and sys.argv[1] == "--debug"

    # 注册Ctrl+C中断处理
    signal.signal(signal.SIGINT, sigint_handler)

    try:
        print("\n初始化电机控制器...")
        mc = init_motor_controller(debug=debug_mode)
        mc_global = mc
        print("✓ 电机控制器初始化成功")
        print(f"  已添加 {len(mc.motors)} 个电机")

        # 持续运行，速度50%（避开低速压降不动的问题）
        run_forever(mc, speed=50)

    except PCA9685InitError as e:
        print(f"\n❌ PCA9685初始化失败: {e}")
        print("\n排查建议:")
        print("1. 检查PCA9685硬件连接")
        print("2. 确认OE引脚接GND")
        print("3. 运行 'i2cdetect -y 5' 确认设备地址")
        sys.exit(1)

    except MotorPWMError as e:
        print(f"\n❌ 硬件PWM错误: {e}")
        print("\n排查建议:")
        print("1. 确认 pwmchip0 已存在（ls /sys/class/pwm/）")
        print("2. 确认你有 root 权限（sudo 运行）")
        sys.exit(1)

    except MotorGPIOError as e:
        print(f"\n❌ GPIO初始化失败: {e}")
        print("\n排查建议:")
        print("1. 确认GPIO引脚未占用")
        print("2. 使用 hobot-config 配置引脚复用")
        print("3. 确认运行权限（sudo执行）")
        sys.exit(1)

    except Exception as e:
        print(f"\n❌ 未知错误: {e}")
        import traceback
        traceback.print_exc()
        # 异常出现也停止电机
        if mc_global is not None:
            mc_global.stop_all()
        sys.exit(1)


if __name__ == "__main__":
    main()