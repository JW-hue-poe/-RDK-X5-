#!/bin/bash
# -*- coding: utf-8 -*-
# deploy_models.sh
# RDK 模型部署脚本（在 RDK 板端执行）
#
# 本脚本负责：
#   1. 创建模型目录
#   2. 检查/复制人体检测模型（YOLOv8 ONNX 或 TROS mono2d_body_detection）
#   3. 检查深度估计模型（Depth Anything V2 Small ONNX / BPU bin）

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MODEL_DIR="/opt/rescue_robot/models"
mkdir -p "$MODEL_DIR"

# ----------------------- 人体检测模型 -----------------------
YOLOV8_ONNX="$MODEL_DIR/yolov8n.onnx"
YOLOV8_BIN="$MODEL_DIR/yolov8n.bin"
MONO2D_BIN_SRC="/opt/tros/lib/mono2d_body_detection/config/mono2d_body_detection.bin"
MONO2D_BIN_DST="$MODEL_DIR/mono2d_body_detection.bin"

# 1) 自定义 YOLOv8 BPU 模型（优先级最高）
if [ -f "$YOLOV8_BIN" ]; then
    echo "已找到自定义 YOLOv8 BPU 模型: $YOLOV8_BIN"
    echo "提示：如需使用，请将 robot_params.yaml 中 model_path 改为 $YOLOV8_BIN 并设置 use_bpu: true"
fi

# 2) YOLOv8 ONNX 模型（OpenCV DNN 推理用）
if [ -f "$YOLOV8_ONNX" ]; then
    echo "已找到 YOLOv8 ONNX 模型: $YOLOV8_ONNX"
else
    echo "未找到 YOLOv8 ONNX 模型: $YOLOV8_ONNX"
    echo "请在 PC 上运行: python3 scripts/export_yolov8_onnx.py"
    echo "然后将生成的 yolov8n.onnx 复制到 RDK 的 $MODEL_DIR"
fi

# 3) TROS 自带 BPU 人体检测模型（最简方案，无需转换）
if [ -f "$MONO2D_BIN_SRC" ]; then
    cp "$MONO2D_BIN_SRC" "$MONO2D_BIN_DST"
    echo "已复制 TROS 人体检测模型: $MONO2D_BIN_DST"
    echo "提示：这是开箱即用的 BPU 模型，config/robot_params.yaml 默认已指向该文件"
fi

# ----------------------- 单目深度模型 -----------------------
# 复用 setup_monocular_depth.sh 的检查/导出逻辑
bash "$SCRIPT_DIR/setup_monocular_depth.sh" || true

chmod 644 "$MODEL_DIR"/* 2>/dev/null || true
echo "模型部署检查完成"
