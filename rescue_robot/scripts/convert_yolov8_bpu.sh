#!/bin/bash
# -*- coding: utf-8 -*-
# convert_yolov8_bpu.sh
# 在 Linux / WSL 环境中将 YOLOv8 ONNX 模型转换为 RDK BPU bin 模型
#
# 前置条件：
#   1. 已安装 D-Robotics hb_mapper 工具链（参考官方 Docker 或 apt 安装方式）
#   2. 已导出 yolov8n.onnx（python3 scripts/export_yolov8_onnx.py yolov8n）
#   3. calibration_data/ 目录下已准备标定图片
#
# 用法：
#   bash scripts/convert_yolov8_bpu.sh [模型名，默认 yolov8n]

set -e

MODEL_NAME="${1:-yolov8n}"
ONNX_FILE="${MODEL_NAME}.onnx"
BPU_CONFIG="config/${MODEL_NAME}_bpu.yaml"
CALIB_DIR="calibration_data"
OUTPUT_DIR="bpu_output"

if ! command -v hb_mapper &> /dev/null; then
    echo "错误：未找到 hb_mapper 命令"
    echo "请先安装 D-Robotics 工具链，或使用官方 Docker 环境："
    echo "  docker pull drobotics/horizon_xj3_open_explorer"
    exit 1
fi

if [ ! -f "$ONNX_FILE" ]; then
    echo "错误：未找到 ONNX 模型: $ONNX_FILE"
    echo "请先运行: python3 scripts/export_yolov8_onnx.py $MODEL_NAME"
    exit 1
fi

if [ ! -f "$BPU_CONFIG" ]; then
    echo "错误：未找到 BPU 配置文件: $BPU_CONFIG"
    echo "请检查 config/${MODEL_NAME}_bpu.yaml 是否存在"
    exit 1
fi

if [ ! -d "$CALIB_DIR" ] || [ -z "$(ls -A "$CALIB_DIR" 2>/dev/null)" ]; then
    echo "警告：标定数据目录 $CALIB_DIR 不存在或为空"
    echo "转换仍可继续，但建议准备 100~200 张 640x640 的 COCO 图片以提升量化精度"
    read -p "是否继续? [y/N] " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

mkdir -p "$OUTPUT_DIR"

echo "开始转换 $ONNX_FILE -> BPU bin ..."
echo "配置文件: $BPU_CONFIG"
echo "输出目录: $OUTPUT_DIR"

hb_mapper makertbin --config "$BPU_CONFIG" --model "$ONNX_FILE"

BIN_FILE="${OUTPUT_DIR}/${MODEL_NAME}_bin_model_x1.hbm"
if [ ! -f "$BIN_FILE" ]; then
    # 新版工具链输出文件名可能不同，尝试查找
    BIN_FILE=$(find "$OUTPUT_DIR" -name "*.bin" -o -name "*.hbm" | head -n 1)
fi

if [ -f "$BIN_FILE" ]; then
    echo "转换成功: $BIN_FILE"
    echo ""
    echo "部署到 RDK："
    echo "  1. 复制到 RDK: scp $BIN_FILE sunrise@<RDK_IP>:/opt/rescue_robot/models/${MODEL_NAME}.bin"
    echo "  2. 修改 config/robot_params.yaml 中 body_detector_node.body_detection.model_path 为 /opt/rescue_robot/models/${MODEL_NAME}.bin"
    echo "  3. 设置 use_bpu: true"
else
    echo "未找到输出的 bin/hbm 文件，请检查 $OUTPUT_DIR 目录"
    exit 1
fi
