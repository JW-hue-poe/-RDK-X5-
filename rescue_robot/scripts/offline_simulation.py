#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
offline_simulation.py
离线仿真测试：无需 RDK 和 ROS2，直接验证图像处理、深度估计、人体测距和决策逻辑
"""

import cv2
import numpy as np
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'rescue_robot'))


class OfflineCamera:
    def __init__(self, device=0, width=640, height=480):
        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.fx = 609.0
        self.fy = 609.0
        self.cx = 320.0
        self.cy = 240.0

    def read(self):
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame


class OfflineBodyDetector:
    def __init__(self, avg_height=1.7, camera_height=0.45, pitch=10.0):
        self.avg_height = avg_height
        self.camera_height = camera_height
        self.pitch = math.radians(pitch)

    def detect(self, frame):
        # 测试模式：画面中心生成模拟人体框
        h, w = frame.shape[:2]
        bw, bh = int(w * 0.25), int(h * 0.6)
        x1, y1 = (w - bw) // 2, (h - bh) // 2
        x2, y2 = x1 + bw, y1 + bh
        pixel_h = y2 - y1
        distance = (self.fy * self.avg_height) / pixel_h * math.cos(self.pitch)
        distance -= self.camera_height * math.sin(self.pitch)
        return [(x1, y1, x2, y2, 0.85, round(max(0.1, distance), 2))]


class OfflineDepthEstimator:
    def __init__(self, safe_distance=0.8, danger_distance=0.4):
        self.safe_distance = safe_distance
        self.danger_distance = danger_distance

    def estimate(self, frame):
        h, w = frame.shape[:2]
        y, x = np.ogrid[:h, :w]
        cx, cy = w // 2, h // 2
        dist = np.sqrt((x - cx) ** 2 + (y - cy) ** 2)
        depth = 0.2 + dist / (max(w, h) * 0.5)
        return depth.astype(np.float32)

    def analyze(self, depth):
        h, w = depth.shape
        y1, y2 = int(h * 0.45), int(h * 0.85)
        roi = depth[y1:y2, :]
        obstacle_mask = roi < self.safe_distance
        nearest = float(roi[obstacle_mask].min()) if obstacle_mask.any() else 10.0

        passable_mask = (roi > self.safe_distance).astype(np.uint8) * 255
        ys, xs = np.where(passable_mask > 0)
        offset = 0
        if len(xs) > 0:
            offset = int(np.mean(xs[np.argsort(ys)[-max(1, len(ys) // 5):]])) - w // 2
        return nearest, offset


class OfflineNavigator:
    def __init__(self, forward_speed=0.25, turn_speed=0.6, human_stop=1.0):
        self.forward_speed = forward_speed
        self.turn_speed = turn_speed
        self.human_stop = human_stop
        self.state = 'EXPLORING'

    def decide(self, human_dist, obs_dist, offset):
        if human_dist <= self.human_stop:
            self.state = 'HUMAN_PAUSED'
            return 0.0, 0.0
        if obs_dist < 0.4:
            self.state = 'AVOIDING'
            return 0.0, self.turn_speed if offset <= 0 else -self.turn_speed
        self.state = 'EXPLORING'
        angular = -offset * 0.003
        angular = max(-self.turn_speed, min(self.turn_speed, angular))
        return self.forward_speed, angular


def main():
    camera = OfflineCamera()
    detector = OfflineBodyDetector()
    depth_est = OfflineDepthEstimator()
    navigator = OfflineNavigator()

    print("按 'q' 退出")
    while True:
        frame = camera.read()
        if frame is None:
            continue

        dets = detector.detect(frame)
        depth = depth_est.estimate(frame)
        obs_dist, offset = depth_est.analyze(depth)
        human_dist = dets[0][5] if dets else float('inf')
        linear, angular = navigator.decide(human_dist, obs_dist, offset)

        # 可视化
        for x1, y1, x2, y2, conf, dist in dets:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f'{dist:.2f}m', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        depth_color = cv2.applyColorMap(np.clip((depth - 0.1) / 9.9 * 255, 0, 255).astype(np.uint8), cv2.COLORMAP_JET)
        depth_small = cv2.resize(depth_color, (160, 120))
        h, w = frame.shape[:2]
        frame[h - 120:h, w - 160:w] = depth_small

        text = f"State: {navigator.state}  Linear: {linear:.2f}  Angular: {angular:.2f}"
        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow('Offline Rescue Robot Simulation', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    camera.cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
