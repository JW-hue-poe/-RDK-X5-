#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
depth_estimator_node.py
单目深度估计与可通行区域分析节点
- 订阅图像，输出深度图
- 对 ROI 区域做阈值分割，判断前方障碍和可通行区域
- 发布深度图、可通行区域掩码、最近障碍距离、可行区域中心偏移
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32
from typing import List
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

from rescue_robot.utils import compute_passable_offset, normalize_depth_for_display
from rescue_robot.bpu_yolov8_detector import create_bpu_detector


class DepthEstimatorNode(Node):
    def __init__(self):
        super().__init__('depth_estimator_node')

        self._declare_params()
        self._load_params()

        self.bridge = CvBridge()
        self.net = None
        self.bpu_depth = None
        self.external_depth_sub = None
        self._init_model()

        if not self.depth_topic:
            self.image_sub = self.create_subscription(Image, '/rescue_robot/image_raw', self.image_callback, 10)
        else:
            self.image_sub = None

        self.depth_pub = self.create_publisher(Image, '/rescue_robot/depth_image', 10)
        self.mask_pub = self.create_publisher(Image, '/rescue_robot/passable_mask', 10)
        self.obstacle_dist_pub = self.create_publisher(Float32, '/rescue_robot/nearest_obstacle_distance', 10)
        self.passable_offset_pub = self.create_publisher(Int32, '/rescue_robot/passable_offset', 10)

        self.add_on_set_parameters_callback(self._on_parameter_change)
        self.get_logger().info('深度估计节点已启动')

    def _declare_params(self):
        """声明参数"""
        self.declare_parameter('depth.model_path', '/opt/rescue_robot/models/depth_anything_v2_small.bin')
        self.declare_parameter('depth.use_bpu', True)
        self.declare_parameter('depth.input_width', 518)
        self.declare_parameter('depth.input_height', 518)
        self.declare_parameter('depth.min_depth', 0.1)
        self.declare_parameter('depth.max_depth', 10.0)
        self.declare_parameter('depth.depth_topic', '')
        self.declare_parameter('depth.scale', 1.0)
        self.declare_parameter('depth.offset', 0.0)

        self.declare_parameter('obstacle_avoidance.safe_distance', 0.8)
        self.declare_parameter('obstacle_avoidance.danger_distance', 0.4)
        self.declare_parameter('obstacle_avoidance.roi_top', 0.45)
        self.declare_parameter('obstacle_avoidance.roi_bottom', 0.85)
        self.declare_parameter('obstacle_avoidance.passable_width', 120)

    def _load_params(self):
        """加载参数"""
        self.model_path = self.get_parameter('depth.model_path').value
        self.use_bpu = self.get_parameter('depth.use_bpu').value
        self.input_width = self.get_parameter('depth.input_width').value
        self.input_height = self.get_parameter('depth.input_height').value
        self.min_depth = self.get_parameter('depth.min_depth').value
        self.max_depth = self.get_parameter('depth.max_depth').value
        self.depth_topic = self.get_parameter('depth.depth_topic').value
        self.depth_scale = self.get_parameter('depth.scale').value
        self.depth_offset = self.get_parameter('depth.offset').value

        self.safe_distance = self.get_parameter('obstacle_avoidance.safe_distance').value
        self.danger_distance = self.get_parameter('obstacle_avoidance.danger_distance').value
        self.roi_top = self.get_parameter('obstacle_avoidance.roi_top').value
        self.roi_bottom = self.get_parameter('obstacle_avoidance.roi_bottom').value
        self.passable_width = self.get_parameter('obstacle_avoidance.passable_width').value

    def _init_model(self):
        """初始化深度模型：优先外部话题 > BPU > ONNX > 测试模式"""
        if self.depth_topic:
            self.external_depth_sub = self.create_subscription(
                Image, self.depth_topic, self._external_depth_callback, 10
            )
            self.get_logger().info(f'订阅外部深度话题: {self.depth_topic}')
            return

        if self.use_bpu and os.path.exists(self.model_path):
            try:
                # 深度模型不需要 letterbox，直接 resize 到模型输入尺寸
                self.bpu_depth = create_bpu_detector(
                    self.model_path, (self.input_height, self.input_width),
                    letterbox=False, input_format='rgb',
                    norm_mean=(0.485, 0.456, 0.406), norm_std=(0.229, 0.224, 0.225)
                )
                if self.bpu_depth is not None:
                    self.get_logger().info(
                        f'已加载 BPU 深度模型: {self.model_path}，{self.bpu_depth.get_info()}'
                    )
                    return
            except Exception as e:
                self.get_logger().warn(f'BPU 深度模型加载失败: {e}')

        onnx_path = self.model_path.replace('.bin', '.onnx')
        if os.path.exists(onnx_path):
            try:
                self.net = cv2.dnn.readNetFromONNX(onnx_path)
                self.get_logger().info(f'已加载 ONNX 深度模型: {onnx_path}')
                self._warmup_model()
                return
            except Exception as e:
                self.get_logger().warn(f'ONNX 加载失败: {e}')

        self.get_logger().warn('未找到深度模型，进入测试模式（模拟深度图）')

    def _preprocess_onnx(self, frame: np.ndarray) -> np.ndarray:
        """Depth Anything V2 ONNX 预处理：resize -> RGB -> /255 -> (x-mean)/std -> NCHW"""
        img = cv2.resize(frame, (self.input_width, self.input_height))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.astype(np.float32) / 255.0
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        img = (img - mean) / std
        blob = np.transpose(img, (2, 0, 1))[np.newaxis, ...]
        return blob

    def _warmup_model(self):
        """预热深度模型"""
        dummy = np.zeros((self.input_height, self.input_width, 3), dtype=np.uint8)
        try:
            if self.net is not None:
                self.net.setInput(self._preprocess_onnx(dummy))
                self.net.forward()
            self.get_logger().debug('深度模型预热完成')
        except Exception as e:
            self.get_logger().warn(f'深度模型预热失败: {e}', throttle_duration_sec=5.0)

    def image_callback(self, msg: Image):
        if self.depth_topic:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'图像转换失败: {e}', throttle_duration_sec=5.0)
            return

        depth_map = self.estimate_depth(frame)
        self._process_depth(depth_map, msg.header)

    def _external_depth_callback(self, msg: Image):
        """订阅外部深度话题，支持 32FC1（米）或 16UC1（毫米）"""
        try:
            encoding = msg.encoding.lower()
            if encoding == '16uc1':
                depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                depth_map = depth_mm.astype(np.float32) / 1000.0
            elif encoding == '32fc1':
                depth_map = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            else:
                self.get_logger().warn(f'外部深度图编码不支持: {msg.encoding}，期望 16UC1 或 32FC1',
                                       throttle_duration_sec=5.0)
                return
        except Exception as e:
            self.get_logger().warn(f'深度图转换失败: {e}', throttle_duration_sec=5.0)
            return
        self._process_depth(depth_map, msg.header)

    def estimate_depth(self, frame: np.ndarray) -> np.ndarray:
        """返回深度图（米）"""
        if self.bpu_depth is not None:
            return self._estimate_with_bpu(frame)
        if self.net is not None:
            return self._estimate_with_dnn(frame)
        return self._estimate_dummy(frame)

    def _estimate_with_bpu(self, frame: np.ndarray) -> np.ndarray:
        """BPU 深度推理"""
        h, w = frame.shape[:2]
        try:
            outputs, _, _ = self.bpu_depth.infer(frame)
            depth = outputs[0].squeeze()
            if depth.ndim != 2:
                depth = depth[0]
            depth = cv2.resize(depth, (w, h))
            return self._postprocess_depth(depth)
        except Exception as e:
            self.get_logger().warn(f'BPU 深度推理失败: {e}', throttle_duration_sec=5.0)
            return self._estimate_dummy(frame)

    def _estimate_with_dnn(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        try:
            self.net.setInput(self._preprocess_onnx(frame))
            raw = self.net.forward()
        except Exception as e:
            self.get_logger().warn(f'深度推理失败: {e}', throttle_duration_sec=5.0)
            return self._estimate_dummy(frame)

        depth = raw.squeeze()
        depth = cv2.resize(depth, (w, h))
        return self._postprocess_depth(depth)

    def _postprocess_depth(self, depth: np.ndarray) -> np.ndarray:
        """统一后处理：缩放、偏移、裁剪到合理范围"""
        depth = depth * self.depth_scale + self.depth_offset
        depth = np.clip(depth, self.min_depth, self.max_depth)
        return depth.astype(np.float32)

    def _estimate_dummy(self, frame: np.ndarray) -> np.ndarray:
        """测试模式：生成模拟深度图（中心近、四周远）"""
        h, w = frame.shape[:2]
        y, x = np.ogrid[:h, :w]
        cx, cy = w // 2, h // 2
        dist = np.sqrt((x - cx) ** 2 + (y - cy) ** 2)
        depth = 0.2 + dist / (max(w, h) * 0.5)
        depth = np.clip(depth, self.min_depth, self.max_depth)
        return depth.astype(np.float32)

    def _process_depth(self, depth_map: np.ndarray, header):
        h, w = depth_map.shape

        # 发布深度热力图（仅在有订阅者时计算）
        if self.depth_pub.get_subscription_count() > 0:
            depth_uint8 = normalize_depth_for_display(depth_map, self.min_depth, self.max_depth)
            depth_color = cv2.applyColorMap(depth_uint8, cv2.COLORMAP_JET)
            depth_msg = self.bridge.cv2_to_imgmsg(depth_color, encoding='bgr8')
            depth_msg.header = header
            self.depth_pub.publish(depth_msg)

        # ROI 提取（车前核心区域）
        y1 = int(h * self.roi_top)
        y2 = int(h * self.roi_bottom)
        y1, y2 = max(0, y1), min(h, y2)
        if y2 <= y1:
            y1, y2 = 0, h
        roi = depth_map[y1:y2, :]

        # 可通行区域掩码：深度大于安全距离的区域
        passable_mask = (roi > self.safe_distance).astype(np.uint8) * 255
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        passable_mask = cv2.morphologyEx(passable_mask, cv2.MORPH_OPEN, kernel)
        passable_mask = cv2.morphologyEx(passable_mask, cv2.MORPH_CLOSE, kernel)

        full_mask = np.zeros((h, w), dtype=np.uint8)
        full_mask[y1:y2, :] = passable_mask
        mask_msg = self.bridge.cv2_to_imgmsg(full_mask, encoding='mono8')
        mask_msg.header = header
        self.mask_pub.publish(mask_msg)

        # 最近障碍距离（仅考虑危险距离以上的 ROI 内像素，避免地面噪点）
        obstacle_mask = (roi < self.safe_distance) & (roi > self.danger_distance)
        if obstacle_mask.any():
            nearest_obstacle = float(np.percentile(roi[obstacle_mask], 5))
        else:
            nearest_obstacle = float(self.max_depth)
        self.obstacle_dist_pub.publish(Float32(data=nearest_obstacle))

        # 可通行区域中心偏移
        offset = compute_passable_offset(passable_mask, w, self.passable_width)
        self.passable_offset_pub.publish(Int32(data=int(offset)))

        self.get_logger().debug(
            f'最近障碍: {nearest_obstacle:.2f}m, 可通行偏移: {offset}px'
        )

    def _on_parameter_change(self, params: List[Parameter]) -> SetParametersResult:
        """支持运行中动态修改深度范围、安全距离、ROI 等参数"""
        unsupported = []
        for param in params:
            name = param.name
            value = param.value
            if name == 'depth.min_depth':
                self.min_depth = float(value)
            elif name == 'depth.max_depth':
                self.max_depth = float(value)
            elif name == 'depth.scale':
                self.depth_scale = float(value)
            elif name == 'depth.offset':
                self.depth_offset = float(value)
            elif name == 'obstacle_avoidance.safe_distance':
                self.safe_distance = float(value)
            elif name == 'obstacle_avoidance.danger_distance':
                self.danger_distance = float(value)
            elif name == 'obstacle_avoidance.roi_top':
                self.roi_top = float(value)
            elif name == 'obstacle_avoidance.roi_bottom':
                self.roi_bottom = float(value)
            elif name == 'obstacle_avoidance.passable_width':
                self.passable_width = int(value)
            else:
                unsupported.append(name)
        if unsupported:
            return SetParametersResult(successful=False, reason=f'不支持的参数: {", ".join(unsupported)}')
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = DepthEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
