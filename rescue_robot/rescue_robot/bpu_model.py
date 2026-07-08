#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bpu_model.py
RDK BPU 模型通用封装（修复版 v2）

关键修复：
1. NV12 模型输出 NV12 格式，不是 RGB NCHW
2. forward() 直接传 numpy，不 clone
3. 添加输入校验防止 segfault
4. 保留异常信息便于调试
5. 在导入 hobot_dnn 前设置环境变量，确保 BPU 驱动正确加载

依赖 D-Robotics TROS 中的 hobot_dnn / pyeasy_dnn，仅在 RDK 或官方 Docker 中可用。
"""

import os

_hobot_lib_paths = [
    '/usr/hobot/lib',
    '/app/lib',
    '/app/pub/lib',
    '/middleware/lib',
    '/middleware/pub/lib',
    '/usr/hobot/lib/sensor',
    '/system/lib',
    '/system/usr/lib',
    '/lib',
]
_hobot_bin_paths = ['/usr/hobot/bin']

_current_ld = os.environ.get('LD_LIBRARY_PATH', '')
_new_ld = ':'.join(_hobot_lib_paths + ([_current_ld] if _current_ld else []))
os.environ['LD_LIBRARY_PATH'] = _new_ld

_current_path = os.environ.get('PATH', '')
_new_path = ':'.join(_hobot_bin_paths + ([_current_path] if _current_path else []))
os.environ['PATH'] = _new_path

import cv2
import numpy as np
from typing import List, Tuple, Optional
import logging

logger = logging.getLogger(__name__)


class BPUModel:
    """基于 D-Robotics pyeasy_dnn 的通用模型封装（修复版 v2）"""

    def __init__(self, model_path: str, input_size: Tuple[int, int] = (640, 640),
                 letterbox: bool = True, input_format: str = 'rgb',
                 norm_mean: Optional[Tuple[float, float, float]] = None,
                 norm_std: Optional[Tuple[float, float, float]] = None):
        if not os.path.exists(model_path):
            raise FileNotFoundError(f'BPU 模型未找到: {model_path}')

        try:
            from hobot_dnn import pyeasy_dnn as dnn
        except ImportError as e:
            raise RuntimeError(f'未安装 hobot_dnn: {e}') from e

        self.dnn = dnn

        # dnn.load() 返回 list
        models = dnn.load(model_path)
        if not models:
            raise RuntimeError(f'模型加载失败，返回空列表: {model_path}')
        self.model = models[0]
        logger.info(f'BPU 模型加载成功: {model_path}, 模型名: {self.model.name}')

        self.input_h, self.input_w = input_size
        self.letterbox = letterbox
        self.input_format = input_format.lower()
        self.norm_mean = np.array(norm_mean, dtype=np.float32) if norm_mean is not None else None
        self.norm_std = np.array(norm_std, dtype=np.float32) if norm_std is not None else None

        # 获取模型输入属性
        input_info = self.model.inputs[0]
        props = input_info.properties
        shape = list(props.shape)
        tensor_type = getattr(props, 'tensor_type', 'unknown')

        logger.info(f'模型输入属性: shape={shape}, tensor_type={tensor_type}')

        # 判断是否为 NV12 模型
        self.is_nv12_model = (tensor_type == 'NV12' or 'nv12' in model_path.lower())

        if self.is_nv12_model:
            logger.info('检测到 NV12 模型，预处理将输出 NV12 格式')
            self.input_type = 'nv12'
            # NV12 模型输入尺寸
            self.model_h, self.model_w = input_size
        else:
            # 从 shape 推断 layout
            if len(shape) == 4:
                if shape[1] == 3:  # [N, 3, H, W] -> NCHW
                    self.input_layout = 'NCHW'
                    self.model_h, self.model_w = shape[2], shape[3]
                elif shape[3] == 3:  # [N, H, W, 3] -> NHWC
                    self.input_layout = 'NHWC'
                    self.model_h, self.model_w = shape[1], shape[2]
                else:
                    self.input_layout = 'NCHW'
                    self.model_h, self.model_w = shape[2], shape[3]
            else:
                self.input_layout = 'NCHW'
                self.model_h, self.model_w = input_size

            self.input_type = self.input_format

        self._logger_msg = (
            f'BPU 模型: name={self.model.name}, is_nv12={self.is_nv12_model}, '
            f'type={self.input_type}, shape={shape}, letterbox={letterbox}, '
            f'model_size=({self.model_h},{self.model_w})'
        )
        logger.info(self._logger_msg)

    def get_info(self) -> str:
        return self._logger_msg

    def preprocess(self, frame: np.ndarray) -> Tuple[np.ndarray, float, Tuple[float, float]]:
        """
        将 BGR 图像预处理为 BPU 所需格式。
        返回 (input_tensor, letterbox_ratio, (pad_x, pad_y))。
        """
        h, w = frame.shape[:2]

        if self.letterbox:
            img, ratio, pad = self._letterbox(frame, (self.model_w, self.model_h))
        else:
            img = cv2.resize(frame, (self.model_w, self.model_h))
            ratio = min(self.model_w / w, self.model_h / h)
            pad = ((self.model_w - w * ratio) / 2.0, (self.model_h - h * ratio) / 2.0)

        # ===== NV12 模型：输出 NV12 格式 =====
        if self.is_nv12_model:
            nv12_data = self._bgr2nv12(img)
            # NV12 shape: (height * 3 // 2, width) = (960, 640) for 640x640
            logger.debug(f'NV12 preprocess: shape={nv12_data.shape}, dtype={nv12_data.dtype}')
            return nv12_data, ratio, pad

        # ===== RGB/BGR 模型 =====
        if self.input_format == 'bgr':
            converted = img
        else:
            converted = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # BPU 量化模型通常需要 uint8 输入
        if self.norm_mean is not None or self.norm_std is not None:
            converted = converted.astype(np.float32)
            if self.norm_mean is not None:
                converted = converted - self.norm_mean
            if self.norm_std is not None:
                converted = converted / self.norm_std

        if self.input_layout == 'NCHW':
            tensor = np.transpose(converted, (2, 0, 1))[np.newaxis, ...]
        else:
            tensor = converted[np.newaxis, ...]

        return tensor, ratio, pad

    @staticmethod
    def _letterbox(frame: np.ndarray, target_size: Tuple[int, int],
                   color: Tuple[int, int, int] = (114, 114, 114)):
        """保持长宽比的 resize + 灰边填充"""
        h, w = frame.shape[:2]
        tw, th = target_size
        scale = min(tw / w, th / h)
        nw, nh = int(round(w * scale)), int(round(h * scale))
        resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((th, tw, 3), color, dtype=np.uint8)
        top = (th - nh) // 2
        left = (tw - nw) // 2
        canvas[top:top + nh, left:left + nw] = resized
        pad = (left, top)
        return canvas, scale, pad

    @staticmethod
    def _bgr2nv12(image: np.ndarray) -> np.ndarray:
        """BGR -> NV12 (YUV420 semi-planar)

        NV12 格式:
        - Y plane: height x width
        - UV plane: height/2 x width (interleaved U and V)
        """
        height, width = image.shape[:2]

        # 转为 YUV I420
        yuv = cv2.cvtColor(image, cv2.COLOR_BGR2YUV_I420)
        # 确保为一维数组，避免不同 OpenCV 版本返回形状不一致
        yuv = yuv.reshape(-1)

        y_size = height * width
        uv_size = height * width // 2
        expected = y_size + uv_size
        if len(yuv) < expected:
            raise ValueError(f'I420 数据长度不足: {len(yuv)} < {expected}')

        y = yuv[:y_size]
        u = yuv[y_size:y_size + y_size // 4]
        v = yuv[y_size + y_size // 4:expected]

        # Interleave U and V for NV12
        uv = np.empty(uv_size, dtype=np.uint8)
        uv[0::2] = u
        uv[1::2] = v

        # Combine Y + UV
        nv12 = np.concatenate([y, uv])

        # Reshape to (height * 3 // 2, width)
        return nv12.reshape((height * 3 // 2, width))

    def forward(self, input_data: np.ndarray):
        """对已经预处理好的输入执行 BPU 推理，返回 numpy 输出列表"""
        if not isinstance(input_data, np.ndarray):
            raise TypeError(f"输入必须是 numpy array，得到 {type(input_data)}")

        # 确保内存连续
        input_data = np.ascontiguousarray(input_data)

        # 输入校验
        logger.debug(f'forward 输入: shape={input_data.shape}, dtype={input_data.dtype}, '
                    f'contiguous={input_data.flags["C_CONTIGUOUS"]}')

        # 直接传 numpy array，pyeasy_dnn 内部处理
        outputs = self.model.forward(input_data)

        results = []
        for out in outputs:
            arr = np.array(out.buffer)
            results.append(arr)
        return results

    def infer(self, frame: np.ndarray):
        """执行 BPU 推理（预处理 + forward），返回原始输出列表"""
        input_data, ratio, pad = self.preprocess(frame)
        outputs = self.forward(input_data)
        return outputs, ratio, pad


def create_bpu_model(model_path: str, input_size: Tuple[int, int] = (640, 640),
                     letterbox: bool = True, input_format: str = 'rgb',
                     norm_mean: Optional[Tuple[float, float, float]] = None,
                     norm_std: Optional[Tuple[float, float, float]] = None) -> Optional[BPUModel]:
    """工厂函数：尝试创建 BPU 模型，失败返回 None 并记录错误"""
    try:
        return BPUModel(model_path, input_size, letterbox=letterbox, input_format=input_format,
                        norm_mean=norm_mean, norm_std=norm_std)
    except Exception as e:
        logger.error(f'创建 BPU 模型失败: {e}')
        import traceback
        logger.error(traceback.format_exc())
        return None