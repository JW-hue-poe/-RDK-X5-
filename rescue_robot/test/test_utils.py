#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_utils.py
utils.py 单元测试
"""

import pytest
import numpy as np
from rescue_robot.utils import (
    clamp,
    compute_iou,
    estimate_human_distance,
    compute_passable_offset,
    normalize_depth_for_display,
    RobotState,
)


def test_clamp():
    assert clamp(5, 0, 10) == 5
    assert clamp(-1, 0, 10) == 0
    assert clamp(11, 0, 10) == 10


def test_compute_iou():
    a = (0, 0, 10, 10)
    b = (5, 5, 15, 15)
    iou = compute_iou(a, b)
    assert 0.0 < iou < 1.0
    assert compute_iou(a, a) == pytest.approx(1.0)


def test_compute_iou_no_overlap():
    a = (0, 0, 10, 10)
    b = (20, 20, 30, 30)
    assert compute_iou(a, b) == pytest.approx(0.0)


def test_estimate_human_distance():
    d = estimate_human_distance(100, 600, 1.7, 0.45, np.radians(10))
    assert d > 0.1


def test_compute_passable_offset():
    mask = np.zeros((100, 200), dtype=np.uint8)
    mask[80:, 120:180] = 255
    offset = compute_passable_offset(mask, 200, min_passable_width=40)
    assert offset > 0  # 可通行区域在右侧，应向右转


def test_compute_passable_offset_empty():
    mask = np.zeros((100, 200), dtype=np.uint8)
    assert compute_passable_offset(mask, 200) == 0


def test_normalize_depth_for_display():
    depth = np.array([[0.5, 1.5, 2.5]], dtype=np.float32)
    out = normalize_depth_for_display(depth, 0.0, 3.0)
    assert out.dtype == np.uint8
    assert out.shape == depth.shape
    assert out.min() >= 0 and out.max() <= 255


def test_robot_state_enum():
    assert int(RobotState.EXPLORING) == 0
    assert int(RobotState.STOPPED) == 5
