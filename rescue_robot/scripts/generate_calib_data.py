#!/usr/bin/env python3
"""生成 Depth Anything V2 BPU 转换用的标定图片

用法：
  python3 scripts/generate_calib_data.py --output-dir /opt/rescue_robot/calibration_data --num 100
"""

import argparse
import os
import numpy as np
import cv2


def main():
    parser = argparse.ArgumentParser(description="生成 BPU 标定图片")
    parser.add_argument("--output-dir", default="/opt/rescue_robot/calibration_data",
                        help="输出目录")
    parser.add_argument("--num", type=int, default=100,
                        help="生成图片数量")
    parser.add_argument("--height", type=int, default=392,
                        help="图片高度")
    parser.add_argument("--width", type=int, default=518,
                        help="图片宽度")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    for i in range(args.num):
        # 随机彩色噪声 + 一些结构化内容，模拟真实场景纹理
        img = np.random.randint(0, 256, (args.height, args.width, 3), dtype=np.uint8)

        # 添加一些渐变和边缘，让标定数据更有区分度
        if i % 3 == 0:
            grad = np.linspace(0, 255, args.width, dtype=np.uint8)
            img[:, :, 0] = grad[np.newaxis, :]
        elif i % 3 == 1:
            cv2.rectangle(img, (50, 50), (args.width - 50, args.height - 50),
                          (128, 128, 128), thickness=3)

        path = os.path.join(args.output_dir, f"calib_{i:04d}.jpg")
        cv2.imwrite(path, img)

    print(f"已生成 {args.num} 张标定图片到 {args.output_dir}")


if __name__ == "__main__":
    main()
