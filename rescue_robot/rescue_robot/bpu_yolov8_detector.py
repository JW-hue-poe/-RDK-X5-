#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bpu_yolov8_detector.py
为兼容旧代码保留的别名，实际逻辑已迁移到 bpu_model.py。
"""

from rescue_robot.bpu_model import BPUModel as BPUYolov8Detector
from rescue_robot.bpu_model import create_bpu_model as create_bpu_detector

__all__ = ['BPUYolov8Detector', 'create_bpu_detector']
