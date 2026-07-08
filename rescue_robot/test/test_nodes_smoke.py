#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_nodes_smoke.py
节点导入/实例化冒烟测试
"""

import pytest


def test_import_utils():
    from rescue_robot import utils
    assert utils is not None


def test_import_nodes():
    """确保所有节点模块可被导入（不启动 rclpy）"""
    rclpy = pytest.importorskip('rclpy', reason='需要 ROS2 环境')
    from rescue_robot import (
        camera_node,
        body_detector_node,
        depth_estimator_node,
        visualization_node,
        bpu_model,
    )
    assert camera_node.CameraNode is not None
    assert body_detector_node.BodyDetectorNode is not None
    assert depth_estimator_node.DepthEstimatorNode is not None
    assert visualization_node.VisualizationNode is not None


@pytest.mark.skip(reason='需要 ROS2 环境，作为本地验证使用')
def test_camera_node_init():
    import rclpy
    from rescue_robot.camera_node import CameraNode
    rclpy.init()
    node = CameraNode()
    assert node.get_name() == 'camera_node'
    node.destroy_node()
    rclpy.shutdown()
