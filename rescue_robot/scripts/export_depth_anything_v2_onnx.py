#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
export_depth_anything_v2_onnx.py
在 PC 端导出 Depth Anything V2 Small ONNX 模型

用法：
  python3 scripts/export_depth_anything_v2_onnx.py --model-dir /opt/rescue_robot/models

说明：
  - 会自动克隆官方仓库、安装依赖、下载 Small (vits) 权重
  - 导出输入尺寸 518x518 的 ONNX，输出文件名为 depth_anything_v2_small.onnx
  - 需要 torch / torchvision / opencv-python 环境
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
import urllib.request

REPO_URL = "https://github.com/DepthAnything/Depth-Anything-V2.git"
HF_MIRROR = "https://hf-mirror.com"
REPO_ID = "depth-anything/Depth-Anything-V2-Small"
CKPT_NAME = "depth_anything_v2_vits.pth"

ENCODER = "vits"
FEATURES = 64
OUT_CHANNELS = [48, 96, 192, 384]
INPUT_SIZE = 518


def run(cmd: str, cwd: str = None):
    print(f">>> {cmd}")
    subprocess.check_call(cmd, shell=True, cwd=cwd)


def download(url: str, dst: str):
    if os.path.exists(dst):
        print(f"已存在，跳过下载: {dst}")
        return
    print(f"下载: {url} -> {dst}")
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    urllib.request.urlretrieve(url, dst)


def main():
    parser = argparse.ArgumentParser(description="导出 Depth Anything V2 Small ONNX")
    parser.add_argument(
        "--model-dir", default="/opt/rescue_robot/models",
        help="输出 ONNX 模型的目录"
    )
    parser.add_argument(
        "--work-dir", default=os.path.join(tempfile.gettempdir(), "depth_anything_v2"),
        help="临时工作目录"
    )
    args = parser.parse_args()

    os.makedirs(args.model_dir, exist_ok=True)
    repo_dir = args.work_dir

    # 1) 克隆官方仓库
    if not os.path.isdir(os.path.join(repo_dir, ".git")):
        run(f"git clone --depth 1 {REPO_URL} {repo_dir}")

    # 2) 安装依赖
    run("pip install -r requirements.txt", cwd=repo_dir)

    # 3) 下载 Small 权重
    ckpt_path = os.path.join(repo_dir, "checkpoints", CKPT_NAME)
    download(f"{HF_MIRROR}/{REPO_ID}/resolve/main/{CKPT_NAME}", ckpt_path)

    # 4) 导入并导出 ONNX
    sys.path.insert(0, repo_dir)
    from depth_anything_v2.dpt import DepthAnythingV2  # noqa: E402
    import torch  # noqa: E402

    model = DepthAnythingV2(
        encoder=ENCODER,
        features=FEATURES,
        out_channels=OUT_CHANNELS,
    )
    state_dict = torch.load(ckpt_path, map_location="cpu")
    model.load_state_dict(state_dict)
    model.eval()

    dummy_input = torch.randn(1, 3, INPUT_SIZE, INPUT_SIZE)
    onnx_name = f"depth_anything_v2_{ENCODER}.onnx"
    onnx_path = os.path.join(repo_dir, onnx_name)

    torch.onnx.export(
        model,
        dummy_input,
        onnx_path,
        input_names=["image"],
        output_names=["depth"],
        opset_version=14,
        dynamic_axes={
            "image": {0: "batch"},
            "depth": {0: "batch"},
        },
    )

    dst = os.path.join(args.model_dir, "depth_anything_v2_small.onnx")
    shutil.copy(onnx_path, dst)
    print(f"\nONNX 导出完成: {dst}")
    print("下一步（可选）：使用 D-Robotics hb_mapper 转换为 BPU .bin，或直接用 OpenCV DNN 推理")


if __name__ == "__main__":
    main()
