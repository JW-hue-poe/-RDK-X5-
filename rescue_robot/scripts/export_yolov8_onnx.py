#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
export_yolov8_onnx.py
在 PC 上导出 YOLOv8 ONNX 模型，用于 RDK 部署

运行方式：
    python3 scripts/export_yolov8_onnx.py

依赖：
    pip install ultralytics

默认导出 yolov8n，可通过命令行参数选择其他模型：
    python3 scripts/export_yolov8_onnx.py yolov8s
"""

import sys
import os


def main():
    model_name = sys.argv[1] if len(sys.argv) > 1 else 'yolov8n'
    try:
        from ultralytics import YOLO
    except ImportError:
        print("请先安装 ultralytics: pip install ultralytics")
        sys.exit(1)

    print(f"正在加载 {model_name} ...")
    model = YOLO(model_name)

    print(f"正在导出 ONNX ...")
    # dynamic=False 保证输入尺寸固定，方便 OpenCV DNN 和 BPU 工具链处理
    model.export(format='onnx', dynamic=False, simplify=True)

    onnx_path = f"{model_name}.onnx"
    if os.path.exists(onnx_path):
        print(f"导出成功: {onnx_path}")
        print(f"建议复制到 RDK 的 /opt/rescue_robot/models/ 目录")
    else:
        print("导出失败，请检查 ultralytics 版本")


if __name__ == '__main__':
    main()
