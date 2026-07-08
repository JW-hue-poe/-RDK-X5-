#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
utils.py
公共工具函数
"""

import numpy as np
import math
from enum import IntEnum


class RobotState(IntEnum):
    EXPLORING = 0
    AVOIDING = 1
    HUMAN_DETECTED = 2
    HUMAN_PAUSED = 3
    U_TURN = 4
    STOPPED = 5


def clamp(value, min_val, max_val):
    """将值限制在 [min_val, max_val] 范围内"""
    return max(min_val, min(max_val, value))


def compute_iou(a, b):
    """计算两个框的 IOU，a、b 格式为 (x1, y1, x2, y2)"""
    x1 = max(a[0], b[0])
    y1 = max(a[1], b[1])
    x2 = min(a[2], b[2])
    y2 = min(a[3], b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    union = area_a + area_b - inter
    return inter / (union + 1e-6)


def estimate_human_distance(pixel_height, fy, average_height, camera_height, pitch_rad):
    """
    基于相似三角形的人体测距算法（考虑相机俯仰角近似修正）。
    参数：
      pixel_height: 检测框像素高度
      fy: 相机焦距（像素）
      average_height: 人体平均身高（米）
      camera_height: 相机离地高度（米）
      pitch_rad: 相机俯仰角（弧度，向下为正）
    """
    pixel_height = max(1, pixel_height)
    # 将像素高度折算到与光轴垂直的平面上，再扣除相机高度带来的视差
    distance = (fy * average_height * math.cos(pitch_rad)) / pixel_height
    distance -= camera_height * math.sin(pitch_rad)
    return max(0.1, distance)


def compute_passable_offset(passable_mask, image_width, min_passable_width=120):
    """
    计算可通行区域相对于图像中心的水平偏移。
    优先使用 ROI 底部区域，并按列统计最宽的可通行窗口。
    正数表示应向右转，负数表示应向左转。
    """
    ys, xs = np.where(passable_mask > 0)
    if len(xs) == 0:
        return 0

    # 仅使用最下方的 1/3 像素，越靠近机器人越重要
    threshold_y = np.percentile(ys, 66)
    bottom_mask = ys >= threshold_y
    xs_bottom = xs[bottom_mask]
    if len(xs_bottom) == 0:
        return 0

    # 寻找宽度大于 min_passable_width 的最宽可通行区间中心
    sorted_xs = np.sort(xs_bottom)
    gaps = np.diff(sorted_xs)
    gap_idx = np.where(gaps > 1)[0]
    segments = []
    start = 0
    for idx in gap_idx:
        segments.append((sorted_xs[start], sorted_xs[idx]))
        start = idx + 1
    segments.append((sorted_xs[start], sorted_xs[-1]))

    best_center = None
    best_width = 0
    for x1, x2 in segments:
        width = x2 - x1 + 1
        if width >= min_passable_width and width > best_width:
            best_width = width
            best_center = (x1 + x2) // 2

    if best_center is None:
        # 没有满足最小宽度的区域，退回到全部底部像素的重心
        best_center = int(np.mean(xs_bottom))
    else:
        best_center = int(best_center)

    return int(best_center - image_width // 2)


def normalize_depth_for_display(depth_map, min_depth, max_depth):
    """将深度图归一化并转为 uint8，用于显示"""
    depth_normalized = (depth_map - min_depth) / (max_depth - min_depth + 1e-6)
    depth_normalized = np.clip(depth_normalized, 0.0, 1.0)
    return (depth_normalized * 255).astype(np.uint8)
