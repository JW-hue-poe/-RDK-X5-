#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
body_detector_node.py
人体检测与测距节点
- 订阅相机图像
- 优先使用 RDK BPU 运行时加载自定义 YOLOv8 / YOLO11 bin 模型
- 否则使用 OpenCV DNN 加载 YOLOv8 ONNX 模型
- 计算人体像素坐标、置信度和单目测距距离
- 发布目标列表用于导航决策
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import os
import json
import traceback
from typing import List, Tuple, Dict

from rescue_robot.utils import compute_iou, estimate_human_distance
from rescue_robot.bpu_yolov8_detector import create_bpu_detector


class BodyDetectorNode(Node):
    def __init__(self):
        super().__init__('body_detector_node')

        self._declare_params()
        self._load_params()

        # 目标跟踪
        self.tracks: Dict[int, Dict] = {}
        self.next_track_id = 1

        self.bridge = CvBridge()
        self.net = None
        self.bpu_detector = None
        self._init_model()

        self.image_sub = self.create_subscription(
            Image, '/rescue_robot/image_raw', self.image_callback, 10)
        self.det_pub = self.create_publisher(
            Detection2DArray, '/rescue_robot/body_detections', 10)
        self.nearest_human_pub = self.create_publisher(
            Float32, '/rescue_robot/nearest_human_distance', 10)

        self.add_on_set_parameters_callback(self._on_parameter_change)
        self.get_logger().info('人体检测节点已启动')

    def _declare_params(self):
        """声明参数"""
        self.declare_parameter('body_detection.model_path',
                               '/opt/rescue_robot/models/yolov8n.onnx')
        self.declare_parameter('body_detection.model_type',
                               'yolov8')  # yolov8 / generic
        self.declare_parameter('body_detection.class_id',
                               0)  # COCO 中 person 的 id
        self.declare_parameter('body_detection.use_bpu', False)
        self.declare_parameter('body_detection.conf_threshold', 0.5)
        self.declare_parameter('body_detection.nms_threshold', 0.45)
        self.declare_parameter('body_detection.input_width', 640)
        self.declare_parameter('body_detection.input_height', 640)
        self.declare_parameter('body_detection.max_track_miss', 30)
        self.declare_parameter('body_detection.letterbox', True)
        self.declare_parameter(
            'body_detection.input_format', 'rgb')  # rgb / bgr / nv12

        self.declare_parameter('distance_estimation.average_human_height', 1.7)
        self.declare_parameter('distance_estimation.camera_height', 0.45)
        self.declare_parameter('distance_estimation.camera_pitch', 10.0)

        self.declare_parameter('camera.fx', 609.0)
        self.declare_parameter('camera.fy', 609.0)
        self.declare_parameter('camera.cx', 320.0)
        self.declare_parameter('camera.cy', 240.0)

    def _load_params(self):
        """加载参数"""
        self.model_path = self.get_parameter('body_detection.model_path').value
        self.model_type = self.get_parameter('body_detection.model_type').value
        self.class_id = self.get_parameter('body_detection.class_id').value
        self.use_bpu = self.get_parameter('body_detection.use_bpu').value
        self.conf_threshold = self.get_parameter(
            'body_detection.conf_threshold').value
        self.nms_threshold = self.get_parameter(
            'body_detection.nms_threshold').value
        self.input_width = self.get_parameter(
            'body_detection.input_width').value
        self.input_height = self.get_parameter(
            'body_detection.input_height').value
        self.max_track_miss = self.get_parameter(
            'body_detection.max_track_miss').value
        self.letterbox = self.get_parameter('body_detection.letterbox').value
        self.input_format = self.get_parameter(
            'body_detection.input_format').value

        self.average_human_height = self.get_parameter(
            'distance_estimation.average_human_height').value
        self.camera_height = self.get_parameter(
            'distance_estimation.camera_height').value
        self.camera_pitch = math.radians(self.get_parameter(
            'distance_estimation.camera_pitch').value)
        self.fx = self.get_parameter('camera.fx').value
        self.fy = self.get_parameter('camera.fy').value
        self.cx = self.get_parameter('camera.cx').value
        self.cy = self.get_parameter('camera.cy').value

    def _init_model(self):
        """初始化模型：优先使用 BPU 运行时；否则尝试 OpenCV DNN 加载 ONNX；最后回退测试模式。"""
        if self.use_bpu and os.path.exists(self.model_path):
            self.get_logger().info(
                f"尝试初始化BPU检测器，模型路径：{self.model_path}，输入尺寸(h,w)=({self.input_height},{self.input_width})，输入格式={self.input_format}")
            try:
                self.bpu_detector = create_bpu_detector(
                    self.model_path, (self.input_height, self.input_width),
                    letterbox=self.letterbox, input_format=self.input_format
                )
            except Exception as e:
                self.get_logger().error(f"创建BPU检测器抛出异常：{str(e)}")
                self.get_logger().error(traceback.format_exc())
                self.bpu_detector = None

            if self.bpu_detector is not None:
                self.get_logger().info(
                    f'已加载 BPU 模型: {self.model_path}，{self.bpu_detector.get_info()}'
                )
                self._warmup_model()
                return
            self.get_logger().warn(
                f'BPU 模型加载失败: {self.model_path}，将回退到 OpenCV DNN 或测试模式'
            )

        # OpenCV DNN 回退：尝试与 .bin 同名的 .onnx
        onnx_path = self.model_path.replace('.bin', '.onnx')
        if os.path.exists(onnx_path):
            try:
                self.net = cv2.dnn.readNetFromONNX(onnx_path)
                self.get_logger().info(f'已加载 ONNX 模型: {onnx_path}')
                self._warmup_model()
            except Exception as e:
                self.get_logger().warn(f'ONNX 加载失败: {e}')
        else:
            self.get_logger().warn('未找到 ONNX 模型，人体检测将使用模拟框进行测试')

    def _warmup_model(self):
        """预热模型，减少首次推理延迟"""
        dummy = np.zeros(
            (self.input_height, self.input_width, 3), dtype=np.uint8)
        try:
            if self.bpu_detector is not None:
                self.bpu_detector.infer(dummy)
            elif self.net is not None:
                blob = cv2.dnn.blobFromImage(dummy, 1 / 255.0, (self.input_width, self.input_height),
                                             swapRB=True, crop=False)
                self.net.setInput(blob)
                self.net.forward()
            self.get_logger().debug('模型预热完成')
        except Exception as e:
            self.get_logger().warn(f'模型预热失败: {e}', throttle_duration_sec=5.0)

    def image_callback(self, msg: Image):
        # 无人订阅时跳过检测以节省算力
        if self.det_pub.get_subscription_count() == 0 and self.nearest_human_pub.get_subscription_count() == 0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'图像转换失败: {e}', throttle_duration_sec=5.0)
            return

        detections = self.detect(frame)
        detections = self._track(detections)

        det_array = Detection2DArray()
        det_array.header = msg.header

        nearest_distance = float('inf')
        for det in detections:
            x1, y1, x2, y2, conf, track_id, distance = det
            d = Detection2D()
            d.header = msg.header
            d.bbox.center.position.x = float((x1 + x2) / 2.0)
            d.bbox.center.position.y = float((y1 + y2) / 2.0)
            d.bbox.size_x = float(max(1.0, x2 - x1))
            d.bbox.size_y = float(max(1.0, y2 - y1))
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.score = float(conf)
            d.results.append(hyp)
            # 将 track_id 和 distance 序列化进 source_img，供可视化/上层节点使用
            d.source_img = self._build_extra_info(track_id, distance)
            det_array.detections.append(d)

            if 0 < distance < nearest_distance:
                nearest_distance = float(distance)

        self.det_pub.publish(det_array)

        if nearest_distance != float('inf'):
            self.nearest_human_pub.publish(Float32(data=nearest_distance))

        self.get_logger().debug(
            f'检测到 {len(detections)} 个人体，最近距离: {nearest_distance:.2f}m'
        )

    def detect(self, frame: np.ndarray) -> List[Tuple]:
        """执行人体检测，返回 [(x1, y1, x2, y2, conf, track_id_placeholder, distance), ...]"""
        if self.bpu_detector is not None:
            return self._detect_with_bpu(frame)
        if self.net is not None:
            return self._detect_with_opencv_dnn(frame)
        return self._detect_dummy(frame)

    def _detect_with_opencv_dnn(self, frame: np.ndarray) -> List[Tuple]:
        """使用 OpenCV DNN 执行推理。根据 model_type 选择 YOLOv8 或通用格式。"""
        try:
            if self.model_type == 'yolov8':
                return self._detect_yolov8(frame)
            return self._detect_generic(frame)
        except Exception as e:
            self.get_logger().warn(f'DNN 推理失败: {e}', throttle_duration_sec=5.0)
            return []

    def _detect_yolov8(self, frame: np.ndarray) -> List[Tuple]:
        """YOLOv8 ONNX 推理：输入 (1, 84, 8400) -> 转置 -> 过滤 person -> NMS -> 坐标还原"""
        letter_frame, ratio, pad = self._letterbox(
            frame, (self.input_width, self.input_height))
        blob = cv2.dnn.blobFromImage(
            letter_frame, 1 / 255.0, (self.input_width, self.input_height),
            swapRB=True, crop=False
        )
        self.net.setInput(blob)
        outputs = self.net.forward()
        return self._postprocess_yolov8_output(outputs[0], frame, ratio, pad)

    def _detect_with_bpu(self, frame: np.ndarray) -> List[Tuple]:
        """BPU 运行时推理，根据输出格式自动选择后处理方式"""
        try:
            input_data, ratio, pad = self.bpu_detector.preprocess(frame)
            outputs = self.bpu_detector.forward(input_data)
            
            output_shapes = [o.shape for o in outputs]
            self.get_logger().info(f'BPU 输出格式: {output_shapes}')
            
            if len(outputs) == 1:
                self.get_logger().info(f'BPU 输出数据类型: {type(outputs[0])}, dtype: {outputs[0].dtype}, size: {outputs[0].size}')
                if outputs[0].size == 42000:
                    self.get_logger().info('使用单输出后处理（42000=1*5*8400*1）')
                    return self._postprocess_bpu_single_output(outputs, frame, ratio, pad)
                else:
                    self.get_logger().warn(f'BPU 输出尺寸不匹配: size={outputs[0].size}, 期望42000')
                    return []
            elif len(outputs) == 6:
                self.get_logger().info('使用多头输出后处理')
                return self._postprocess_bpu_yolov8(outputs, frame, ratio, pad)
            else:
                self.get_logger().warn(f'未识别的 BPU 输出格式: {output_shapes}')
                return []
        except Exception as e:
            self.get_logger().warn(f'BPU 推理失败: {e}', throttle_duration_sec=5.0)
            return []

    def _postprocess_yolov8_output(self, output: np.ndarray, frame: np.ndarray,
                                    ratio: float, pad: Tuple[float, float]) -> List[Tuple]:
        """YOLOv8 ONNX 后处理：输出格式 [1, 84, 8400] 或 [8400, 84]，过滤 person 类别"""
        h, w = frame.shape[:2]
        
        if output.ndim == 3:
            output = output[0]
        
        if output.shape[0] == 84:
            output = output.transpose(1, 0)
        
        boxes = []
        confs = []
        
        for row in output:
            if len(row) < 6:
                continue
            if len(row) == 5:
                cx, cy, bw, bh, conf = row[:5]
                cls = 0
            else:
                cx, cy, bw, bh = row[:4]
                conf = row[4 + self.class_id]
                cls = self.class_id
            
            if conf < self.conf_threshold:
                continue
            
            x1 = int((cx - bw / 2) * self.input_width)
            y1 = int((cy - bh / 2) * self.input_height)
            x2 = int((cx + bw / 2) * self.input_width)
            y2 = int((cy + bh / 2) * self.input_height)
            
            x1 -= pad[0]
            y1 -= pad[1]
            x2 -= pad[0]
            y2 -= pad[1]
            x1 /= ratio
            y1 /= ratio
            x2 /= ratio
            y2 /= ratio
            
            x1, y1 = max(0, int(x1)), max(0, int(y1))
            x2, y2 = min(w, int(x2)), min(h, int(y2))
            
            if x2 <= x1 or y2 <= y1:
                continue
            
            boxes.append([x1, y1, x2, y2])
            confs.append(float(conf))
        
        if len(boxes) == 0:
            return []
        
        indices = cv2.dnn.NMSBoxes(boxes, confs, self.conf_threshold, self.nms_threshold)
        results = []
        for i in indices:
            i = i[0] if isinstance(i, (tuple, list, np.ndarray)) else i
            x1, y1, x2, y2 = boxes[i]
            distance = self._estimate_distance(y1, y2)
            results.append((x1, y1, x2, y2, confs[i], -1, distance))
        return results

    def _postprocess_bpu_single_output(self, outputs: List[np.ndarray], frame: np.ndarray,
                                       ratio: float, pad: Tuple[float, float]) -> List[Tuple]:
        """单输出 YOLO 后处理：输出格式 [1, 5, 8400, 1]，单类别检测"""
        h, w = frame.shape[:2]
        pred = outputs[0]
        pred = np.array(pred.buffer) if hasattr(pred, 'buffer') else pred
        pred = pred.reshape(1, 5, 8400, 1)
        
        boxes = pred[0, :4, :, 0].T  # (8400, 4) [x1, y1, x2, y2]
        confs = pred[0, 4, :, 0]      # (8400,)

        self.get_logger().info(f'置信度统计: max={confs.max():.4f}, min={confs.min():.4f}, mean={confs.mean():.4f}, 阈值={self.conf_threshold}')
        self.get_logger().info(f'高于阈值的候选框数量: {(confs >= self.conf_threshold).sum()}')

        mask = confs >= self.conf_threshold
        boxes = boxes[mask]
        confs = confs[mask]

        if len(boxes) == 0:
            return []

        indices = cv2.dnn.NMSBoxes(boxes.tolist(), confs.tolist(),
                                   self.conf_threshold, self.nms_threshold)
        results = []
        for i in indices:
            i = i[0] if isinstance(i, (tuple, list, np.ndarray)) else i
            x1, y1, x2, y2 = boxes[i].astype(int)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(self.input_width, x2), min(self.input_height, y2)
            if x2 <= x1 or y2 <= y1:
                continue
            
            x1 -= pad[0]
            y1 -= pad[1]
            x2 -= pad[0]
            y2 -= pad[1]
            x1 /= ratio
            y1 /= ratio
            x2 /= ratio
            y2 /= ratio
            x1, y1 = max(0, int(x1)), max(0, int(y1))
            x2, y2 = min(w, int(x2)), min(h, int(y2))
            if x2 <= x1 or y2 <= y1:
                continue
            distance = self._estimate_distance(y1, y2)
            results.append((x1, y1, x2, y2, float(confs[i]), -1, distance))
        return results

    def _postprocess_bpu_yolov8(self, outputs: List[np.ndarray], frame: np.ndarray,
                                ratio: float, pad: Tuple[float, float]) -> List[Tuple]:
        """YOLOv8 BPU 多头输出后处理：6 个输出 [cls_80, box_80, cls_40, box_40, cls_20, box_20]"""
        h, w = frame.shape[:2]
        strides = [8, 16, 32]
        reg_max = 16
        proj = np.arange(reg_max, dtype=np.float32)

        all_boxes = []
        all_scores = []
        all_classes = []

        for i in range(3):
            cls = outputs[i * 2][0]      # (H, W, 80)
            box = outputs[i * 2 + 1][0]  # (H, W, 64)
            stride = strides[i]
            H, W = cls.shape[:2]

            # DFL decode: (H, W, 64) -> (H, W, 4, 16) -> softmax -> weighted sum -> (H, W, 4)
            box = box.reshape(H, W, 4, reg_max)
            box = np.exp(box - box.max(axis=-1, keepdims=True))
            box = box / box.sum(axis=-1, keepdims=True)
            box = (box * proj).sum(axis=-1)  # (H, W, 4)

            # sigmoid + decode to xyxy (ltrb format)
            box = 1.0 / (1.0 + np.exp(-box))
            grid_x, grid_y = np.meshgrid(np.arange(W), np.arange(H))
            grid_x = (grid_x + 0.5) * stride
            grid_y = (grid_y + 0.5) * stride

            lt = box[..., :2] * stride
            rb = box[..., 2:4] * stride

            x1 = grid_x - lt[..., 0]
            y1 = grid_y - lt[..., 1]
            x2 = grid_x + rb[..., 0]
            y2 = grid_y + rb[..., 1]

            xyxy = np.stack([x1, y1, x2, y2], axis=-1).reshape(-1, 4)  # (H*W, 4)

            # cls sigmoid
            cls = 1.0 / (1.0 + np.exp(-cls)).reshape(-1, 80)  # (H*W, 80)
            scores = cls.max(axis=-1)  # (H*W,)
            class_ids = cls.argmax(axis=-1)  # (H*W,)

            keep = (class_ids == self.class_id) & (scores >= self.conf_threshold)
            if not keep.any():
                continue

            all_boxes.append(xyxy[keep])
            all_scores.append(scores[keep])
            all_classes.append(class_ids[keep])

        if not all_boxes:
            return []

        all_boxes = np.vstack(all_boxes)
        all_scores = np.concatenate(all_scores)

        # NMS
        indices = cv2.dnn.NMSBoxes(all_boxes.tolist(), all_scores.tolist(),
                                   self.conf_threshold, self.nms_threshold)
        results = []
        for i in indices:
            i = i[0] if isinstance(i, (tuple, list, np.ndarray)) else i
            x1, y1, x2, y2 = all_boxes[i].astype(int)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
            if x2 <= x1 or y2 <= y1:
                continue
            # 坐标还原到原图（letterbox）
            x1 -= pad[0]
            y1 -= pad[1]
            x2 -= pad[0]
            y2 -= pad[1]
            x1 /= ratio
            y1 /= ratio
            x2 /= ratio
            y2 /= ratio
            x1, y1 = max(0, int(x1)), max(0, int(y1))
            x2, y2 = min(w, int(x2)), min(h, int(y2))
            if x2 <= x1 or y2 <= y1:
                continue
            distance = self._estimate_distance(y1, y2)
            results.append((x1, y1, x2, y2, float(all_scores[i]), -1, distance))
        return results

    def _detect_generic(self, frame: np.ndarray) -> List[Tuple]:
        """通用 YOLO 格式后处理：outputs[0] 中每个元素为 [cx, cy, bw, bh, conf, cls, ...]"""
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(
            frame, 1 / 255.0, (self.input_width, self.input_height),
            swapRB=True, crop=False
        )
        self.net.setInput(blob)
        outputs = self.net.forward()

        boxes = []
        confs = []
        for out in outputs[0]:
            if len(out) < 6:
                continue
            cx, cy, bw, bh, conf, cls = out[:6]
            if conf < self.conf_threshold or int(cls) != self.class_id:
                continue
            x1 = int((cx - bw / 2) * w)
            y1 = int((cy - bh / 2) * h)
            x2 = int((cx + bw / 2) * w)
            y2 = int((cy + bh / 2) * h)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)
            if x2 <= x1 or y2 <= y1:
                continue
            boxes.append([x1, y1, x2, y2])
            confs.append(float(conf))

        indices = cv2.dnn.NMSBoxes(boxes, confs, self.conf_threshold, self.nms_threshold)
        results = []
        for i in indices:
            i = i[0] if isinstance(i, (tuple, list, np.ndarray)) else i
            x1, y1, x2, y2 = boxes[i]
            distance = self._estimate_distance(y1, y2)
            results.append((x1, y1, x2, y2, confs[i], -1, distance))
        return results

    @staticmethod
    def _letterbox(img: np.ndarray, new_shape=(640, 640), color=(114, 114, 114)):
        """保持长宽比的 resize + padding"""
        shape = img.shape[:2]
        ratio = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        new_unpad = (int(round(shape[1] * ratio)), int(round(shape[0] * ratio)))
        dw = (new_shape[1] - new_unpad[0]) / 2
        dh = (new_shape[0] - new_unpad[1]) / 2
        if shape[::-1] != new_unpad:
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
        return img, ratio, (dw, dh)

    @staticmethod
    def _scale_coords(img1_shape, coords, img0_shape, ratio_pad=None):
        """将坐标从 letterbox 后的图像还原到原图"""
        if ratio_pad is None:
            gain = min(img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1])
            pad = (img1_shape[1] - img0_shape[1] * gain) / 2, (img1_shape[0] - img0_shape[0] * gain) / 2
        else:
            gain = ratio_pad[0]
            pad = ratio_pad[1]

        coords[:, [0, 2]] -= pad[0]
        coords[:, [1, 3]] -= pad[1]
        coords[:, :4] /= gain
        coords[:, [0, 2]] = np.clip(coords[:, [0, 2]], 0, img0_shape[1])
        coords[:, [1, 3]] = np.clip(coords[:, [1, 3]], 0, img0_shape[0])
        return coords

    def _detect_dummy(self, frame: np.ndarray) -> List[Tuple]:
        """测试模式：在画面中心生成一个模拟人体框。"""
        h, w = frame.shape[:2]
        box_w = int(w * 0.25)
        box_h = int(h * 0.6)
        x1 = (w - box_w) // 2
        y1 = (h - box_h) // 2
        x2 = x1 + box_w
        y2 = y1 + box_h
        distance = self._estimate_distance(y1, y2)
        return [(x1, y1, x2, y2, 0.85, -1, distance)]

    def _estimate_distance(self, y_top: int, y_bottom: int) -> float:
        """基于相似三角形的人体测距"""
        return estimate_human_distance(
            y_bottom - y_top,
            self.fy,
            self.average_human_height,
            self.camera_height,
            self.camera_pitch
        )

    def _track(self, detections: List[Tuple]) -> List[Tuple]:
        """简单 IOU 跟踪，给每个检测框分配稳定 ID。"""
        # 将检测统一转为可变列表 [x1,y1,x2,y2,conf,track_id,distance]
        dets = [list(d) for d in detections]
        updated_tracks: Dict[int, Dict] = {}
        used = set()

        # 按 IOU 匹配已有跟踪
        for track_id, track in self.tracks.items():
            best_iou = 0.3
            best_idx = -1
            for i, det in enumerate(dets):
                if i in used:
                    continue
                iou = compute_iou(track['box'], det[:4])
                if iou > best_iou:
                    best_iou = iou
                    best_idx = i
            if best_idx >= 0:
                dets[best_idx][5] = track_id
                updated_tracks[track_id] = {'box': dets[best_idx][:4], 'miss': 0}
                used.add(best_idx)
            else:
                track['miss'] += 1
                if track['miss'] <= self.max_track_miss:
                    updated_tracks[track_id] = track

        # 未匹配的检测分配新 ID
        for i, det in enumerate(dets):
            if i not in used:
                det[5] = self.next_track_id
                updated_tracks[self.next_track_id] = {'box': det[:4], 'miss': 0}
                self.next_track_id += 1

        self.tracks = updated_tracks
        return [tuple(d) for d in dets]

    def _build_extra_info(self, track_id: int, distance: float) -> Image:
        """把 track_id 和 distance 序列化到 Image 消息"""
        info = {'track_id': track_id, 'distance': round(distance, 3)}
        img = Image()
        img.data = json.dumps(info).encode('utf-8')
        return img

    def _on_parameter_change(self, params: List[Parameter]) -> SetParametersResult:
        """支持运行中动态修改阈值、焦距等参数"""
        unsupported = []
        for param in params:
            name = param.name
            value = param.value
            if name == 'body_detection.conf_threshold':
                self.conf_threshold = float(value)
            elif name == 'body_detection.nms_threshold':
                self.nms_threshold = float(value)
            elif name == 'body_detection.max_track_miss':
                self.max_track_miss = int(value)
            elif name == 'body_detection.letterbox':
                self.letterbox = bool(value)
            elif name == 'body_detection.input_format':
                self.input_format = str(value)
            elif name == 'distance_estimation.average_human_height':
                self.average_human_height = float(value)
            elif name == 'distance_estimation.camera_height':
                self.camera_height = float(value)
            elif name == 'distance_estimation.camera_pitch':
                self.camera_pitch = math.radians(float(value))
            elif name == 'camera.fx':
                self.fx = float(value)
            elif name == 'camera.fy':
                self.fy = float(value)
            elif name == 'camera.cx':
                self.cx = float(value)
            elif name == 'camera.cy':
                self.cy = float(value)
            else:
                unsupported.append(name)
        if unsupported:
            return SetParametersResult(successful=False, reason=f'不支持的参数: {", ".join(unsupported)}')
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = BodyDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
