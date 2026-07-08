#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_yolov8_onnx.py
PC 端 YOLOv8 ONNX 人体检测测试脚本
支持图片文件或摄像头输入，无需 ROS2/BPU。

用法：
    python scripts/test_yolov8_onnx.py
    python scripts/test_yolov8_onnx.py path/to/image.jpg
    python scripts/test_yolov8_onnx.py 0          # 摄像头设备号

按 'q' 退出窗口。
"""

import sys
import os
import argparse
import math
import cv2
import numpy as np


DEFAULT_MODEL = 'yolov8n.onnx'
INPUT_WIDTH = 640
INPUT_HEIGHT = 640
CONF_THRESHOLD = 0.5
NMS_THRESHOLD = 0.45
PERSON_CLASS_ID = 0


def letterbox(img: np.ndarray, new_shape=(640, 640), color=(114, 114, 114)):
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


def scale_coords(img1_shape, coords, img0_shape, ratio_pad=None):
    """将坐标从 letterbox 后的图像还原到原图"""
    if ratio_pad is None:
        gain = min(img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1])
        pad = ((img1_shape[1] - img0_shape[1] * gain) / 2, (img1_shape[0] - img0_shape[0] * gain) / 2)
    else:
        gain = ratio_pad[0]
        pad = ratio_pad[1]

    coords[:, [0, 2]] -= pad[0]
    coords[:, [1, 3]] -= pad[1]
    coords[:, :4] /= gain
    coords[:, [0, 2]] = np.clip(coords[:, [0, 2]], 0, img0_shape[1])
    coords[:, [1, 3]] = np.clip(coords[:, [1, 3]], 0, img0_shape[0])
    return coords


def estimate_distance(pixel_h: float, fy: float, avg_height: float = 1.7,
                      camera_height: float = 0.45, pitch_deg: float = 10.0) -> float:
    """基于相似三角形的简单人体测距"""
    pitch = math.radians(pitch_deg)
    distance = (fy * avg_height) / pixel_h * math.cos(pitch)
    distance -= camera_height * math.sin(pitch)
    return max(0.0, distance)


def detect_yolov8(net: cv2.dnn.Net, frame: np.ndarray) -> list:
    """YOLOv8 ONNX 推理与后处理，返回 [(x1, y1, x2, y2, conf, distance), ...]"""
    h, w = frame.shape[:2]
    letter_frame, ratio, pad = letterbox(frame, (INPUT_WIDTH, INPUT_HEIGHT))

    blob = cv2.dnn.blobFromImage(
        letter_frame, 1 / 255.0, (INPUT_WIDTH, INPUT_HEIGHT),
        swapRB=True, crop=False
    )
    net.setInput(blob)
    outputs = net.forward()

    predictions = outputs[0].transpose(1, 0)
    boxes_xywh = predictions[:, :4]
    scores = predictions[:, 4:]

    class_ids = np.argmax(scores, axis=1)
    max_scores = np.max(scores, axis=1)

    keep = (class_ids == PERSON_CLASS_ID) & (max_scores >= CONF_THRESHOLD)
    boxes_xywh = boxes_xywh[keep]
    confs = max_scores[keep]

    if len(boxes_xywh) == 0:
        return []

    xyxy = np.zeros_like(boxes_xywh)
    xyxy[:, 0] = boxes_xywh[:, 0] - boxes_xywh[:, 2] / 2.0
    xyxy[:, 1] = boxes_xywh[:, 1] - boxes_xywh[:, 3] / 2.0
    xyxy[:, 2] = boxes_xywh[:, 0] + boxes_xywh[:, 2] / 2.0
    xyxy[:, 3] = boxes_xywh[:, 1] + boxes_xywh[:, 3] / 2.0

    xyxy = scale_coords((INPUT_HEIGHT, INPUT_WIDTH), xyxy, (h, w), ratio_pad=(ratio, pad))

    indices = cv2.dnn.NMSBoxes(xyxy.tolist(), confs.tolist(), CONF_THRESHOLD, NMS_THRESHOLD)
    results = []
    for i in indices:
        i = i[0] if isinstance(i, (tuple, list, np.ndarray)) else i
        x1, y1, x2, y2 = xyxy[i].astype(int)
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        if x2 <= x1 or y2 <= y1:
            continue
        pixel_h = y2 - y1
        distance = estimate_distance(pixel_h, 609.0)
        results.append((x1, y1, x2, y2, float(confs[i]), distance))
    return results


def draw_detections(frame: np.ndarray, detections: list) -> np.ndarray:
    """绘制检测框和距离"""
    for x1, y1, x2, y2, conf, dist in detections:
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f'person {conf:.2f} {dist:.2f}m'
        cv2.putText(frame, label, (x1, max(20, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(frame, f'detections: {len(detections)}', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    return frame


def main():
    parser = argparse.ArgumentParser(description='YOLOv8 ONNX 人体检测测试')
    parser.add_argument('source', nargs='?', default='0',
                        help='输入源：图片路径或摄像头设备号（默认 0）')
    parser.add_argument('--model', default=DEFAULT_MODEL, help='ONNX 模型路径')
    args = parser.parse_args()

    if not os.path.exists(args.model):
        print(f'模型未找到: {args.model}')
        print('请先运行: python scripts/export_yolov8_onnx.py')
        sys.exit(1)

    print(f'加载模型: {args.model}')
    net = cv2.dnn.readNetFromONNX(args.model)
    print('模型加载成功')

    # 判断输入源类型
    try:
        device = int(args.source)
        cap = cv2.VideoCapture(device)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        is_camera = True
        print(f'打开摄像头: {device}')
    except ValueError:
        if not os.path.exists(args.source):
            print(f'输入文件不存在: {args.source}')
            sys.exit(1)
        frame = cv2.imread(args.source)
        if frame is None:
            print(f'无法读取图片: {args.source}')
            sys.exit(1)
        is_camera = False
        print(f'读取图片: {args.source}')

    if is_camera:
        print("按 'q' 退出")
        while True:
            ret, frame = cap.read()
            if not ret:
                print('摄像头读取失败')
                break
            detections = detect_yolov8(net, frame)
            frame = draw_detections(frame, detections)
            cv2.imshow('YOLOv8 Body Detection', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()
    else:
        detections = detect_yolov8(net, frame)
        frame = draw_detections(frame, detections)
        output_path = 'yolov8_result.jpg'
        cv2.imwrite(output_path, frame)
        print(f'检测结果已保存: {output_path}')
        cv2.imshow('YOLOv8 Body Detection', frame)
        print("按任意键关闭窗口")
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
