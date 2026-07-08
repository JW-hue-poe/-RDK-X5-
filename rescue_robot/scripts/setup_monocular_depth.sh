#!/bin/bash
# setup_monocular_depth.sh
# RDK 单目深度模型准备脚本（可在 RDK 或 PC 上运行）
# 默认输出路径与 config/robot_params.yaml 中 depth.model_path 保持一致

set -e

MODEL_DIR="/opt/rescue_robot/models"
mkdir -p "$MODEL_DIR"

ONNX_DST="$MODEL_DIR/depth_anything_v2_small.onnx"
BIN_DST="$MODEL_DIR/depth_anything_v2_small.bin"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "==> 检查单目深度模型"

if [ -f "$BIN_DST" ]; then
    echo "已找到 BPU 单目深度模型: $BIN_DST"
    echo "配置文件中 depth.use_bpu=true 时将直接使用 BPU 加速"
    exit 0
fi

if [ -f "$ONNX_DST" ]; then
    echo "已找到 ONNX 单目深度模型: $ONNX_DST"
    echo "当前将使用 OpenCV DNN 在 CPU 上推理；如需 BPU 加速，请用 D-Robotics 工具链转换为 .bin"
    exit 0
fi

echo "未找到单目深度模型，尝试自动导出 ONNX ..."

# 判断当前环境是否有 torch，有则自动导出；否则给出手动命令
if python3 -c "import torch" 2>/dev/null; then
    python3 "$SCRIPT_DIR/export_depth_anything_v2_onnx.py" --model-dir "$MODEL_DIR"
else
    echo ""
    echo "当前环境未检测到 torch，无法自动导出。请在有 torch 的 PC 上执行："
    echo "  python3 $SCRIPT_DIR/export_depth_anything_v2_onnx.py --model-dir $MODEL_DIR"
    echo ""
    echo "或手动准备模型："
    echo "  1) 将已导出的 ONNX 复制到 $ONNX_DST"
    echo "  2) 将已转换的 BPU bin 复制到 $BIN_DST"
    echo ""
    echo "BPU 转换示例（在安装了 hb_mapper 的 x86 环境）："
    echo "  hb_mapper makertbin --config depth_vits_config.yaml --model $ONNX_DST --output $BIN_DST"
    exit 1
fi
