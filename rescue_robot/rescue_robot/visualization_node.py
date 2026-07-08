#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
visualization_node.py
可视化节点：订阅图像、人体检测结果、深度图、状态信息，
叠加绘制检测框、距离、状态，并在终端/日志中记录搜救统计。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Float32, Int32, String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os
from datetime import datetime
from collections import deque

from rescue_robot.utils import RobotState


class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')

        self._declare_params()
        self._load_params()

        self.bridge = CvBridge()

        self.image = None
        self.depth = None
        self.detections = []
        self.state = 0
        self.nearest_human = float('inf')
        self.nearest_obstacle = float('inf')
        self.start_time = self.get_clock().now()

        self._init_subscribers()
        self._init_publishers()

        self.timer = self.create_timer(self.log_interval, self.log_status)
        self.vis_timer = self.create_timer(0.05, self.draw_loop)

        self.human_count_total = 0
        self.human_count_current = 0
        self.avoid_count = 0
        self._image_times = deque(maxlen=30)
        self._current_fps = 0.0

        self.get_logger().info('可视化节点已启动')

    def _declare_params(self):
        """声明参数"""
        self.declare_parameter('visualization.show_ui', True)
        self.declare_parameter('visualization.publish_overlay', True)
        self.declare_parameter('visualization.log_interval', 5.0)
        self.declare_parameter('visualization.save_path', '')
        self.declare_parameter('obstacle_avoidance.safe_distance', 0.8)

    def _load_params(self):
        """加载参数"""
        self.show_ui = self.get_parameter('visualization.show_ui').value
        self.publish_overlay = self.get_parameter('visualization.publish_overlay').value
        self.log_interval = self.get_parameter('visualization.log_interval').value
        self.save_path = self.get_parameter('visualization.save_path').value
        self._safe_distance = self.get_parameter('obstacle_avoidance.safe_distance').value

    def _init_subscribers(self):
        """初始化订阅"""
        self.image_sub = self.create_subscription(Image, '/rescue_robot/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/rescue_robot/depth_image', self.depth_callback, 10)
        self.det_sub = self.create_subscription(Detection2DArray, '/rescue_robot/body_detections', self.det_callback, 10)
        self.state_sub = self.create_subscription(Int32, '/rescue_robot/state', self.state_callback, 10)
        self.human_dist_sub = self.create_subscription(Float32, '/rescue_robot/nearest_human_distance', self.human_dist_callback, 10)
        self.obs_dist_sub = self.create_subscription(Float32, '/rescue_robot/nearest_obstacle_distance', self.obs_dist_callback, 10)

    def _init_publishers(self):
        """初始化发布"""
        self.overlay_pub = self.create_publisher(Image, '/rescue_robot/overlay_image', 10)
        self.log_pub = self.create_publisher(String, '/rescue_robot/status_log', 10)

    def image_callback(self, msg: Image):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._image_times.append(self.get_clock().now().nanoseconds / 1e9)
            if len(self._image_times) > 1:
                self._current_fps = (len(self._image_times) - 1) / max(
                    1e-6, self._image_times[-1] - self._image_times[0]
                )
        except Exception as e:
            self.get_logger().warn(f'图像转换失败: {e}', throttle_duration_sec=5.0)

    def depth_callback(self, msg: Image):
        try:
            encoding = msg.encoding.lower()
            if encoding in ('bgr8', 'rgb8'):
                self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            elif encoding == 'mono8':
                self.depth = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8'), cv2.COLOR_GRAY2BGR)
            elif encoding == '32fc1':
                depth_float = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
                depth_norm = cv2.normalize(depth_float, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                self.depth = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
            else:
                self.get_logger().warn(f'暂不支持的深度图编码: {msg.encoding}', throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().warn(f'深度图转换失败: {e}', throttle_duration_sec=5.0)

    def det_callback(self, msg: Detection2DArray):
        self.detections = msg.detections
        self.human_count_current = len(msg.detections)
        # 累计检测到的人体总数（按 track_id 去重）
        current_ids = set()
        for d in msg.detections:
            try:
                extra = json.loads(d.source_img.data.decode('utf-8')) if d.source_img.data else {}
                current_ids.add(extra.get('track_id', 0))
            except Exception:
                pass
        self.human_count_total = max(self.human_count_total, len(current_ids))

    def state_callback(self, msg: Int32):
        self.state = msg.data if 0 <= msg.data <= int(RobotState.STOPPED) else 0

    def human_dist_callback(self, msg: Float32):
        self.nearest_human = msg.data

    def obs_dist_callback(self, msg: Float32):
        self.nearest_obstacle = msg.data

    @property
    def safe_distance(self):
        return self._safe_distance

    def draw_loop(self):
        if self.image is None:
            return
        canvas = self.image.copy()

        self._draw_detections(canvas)
        self._draw_status(canvas)
        self._draw_depth_inset(canvas)

        if self.publish_overlay:
            self._publish_overlay(canvas)

        if self.show_ui:
            cv2.imshow('Rescue Robot - Overlay', canvas)
            cv2.waitKey(1)

        self._save_detection_image(canvas)

        # 状态转移统计避障次数
        prev_state = getattr(self, '_prev_state', RobotState.EXPLORING)
        if self.state in (int(RobotState.AVOIDING), int(RobotState.U_TURN)) and prev_state not in (
                int(RobotState.AVOIDING), int(RobotState.U_TURN)):
            self.avoid_count += 1
        self._prev_state = self.state

    def _draw_detections(self, canvas):
        """绘制人体检测框"""
        for d in self.detections:
            x = int(d.bbox.center.position.x - d.bbox.size_x / 2)
            y = int(d.bbox.center.position.y - d.bbox.size_y / 2)
            w = int(d.bbox.size_x)
            h = int(d.bbox.size_y)
            extra = json.loads(d.source_img.data.decode('utf-8')) if d.source_img.data else {}
            distance = extra.get('distance', 0.0)
            track_id = extra.get('track_id', 0)
            cv2.rectangle(canvas, (x, y), (x + w, y + h), (0, 255, 0), 2)
            label = f'ID:{track_id} {distance:.2f}m'
            cv2.putText(canvas, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    def _draw_status(self, canvas):
        """叠加状态信息"""
        state_text = RobotState(self.state).name if 0 <= self.state <= int(RobotState.STOPPED) else 'UNKNOWN'
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        lines = [
            f'STATE: {state_text}',
            f'Human: {self.human_count_current} (nearest {self.nearest_human:.2f}m)',
            f'Obstacle: {self.nearest_obstacle:.2f}m',
            f'Time: {elapsed:.1f}s | Avoid: {self.avoid_count}',
            f'FPS: {self._current_fps:.1f}',
        ]
        for i, line in enumerate(lines):
            cv2.putText(canvas, line, (10, 30 + i * 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    def _draw_depth_inset(self, canvas):
        """在右下角绘制深度图小窗"""
        if self.depth is None:
            return
        try:
            dh, dw = 120, 160
            depth_small = cv2.resize(self.depth, (dw, dh))
            canvas_h, canvas_w = canvas.shape[:2]
            canvas[canvas_h - dh:canvas_h, canvas_w - dw:canvas_w] = depth_small
        except Exception as e:
            self.get_logger().warn(f'深度图叠加失败: {e}', throttle_duration_sec=5.0)

    def _publish_overlay(self, canvas):
        """发布叠加图像"""
        try:
            overlay_msg = self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
            overlay_msg.header.stamp = self.get_clock().now().to_msg()
            overlay_msg.header.frame_id = 'camera_link'
            self.overlay_pub.publish(overlay_msg)
        except Exception as e:
            self.get_logger().warn(f'叠加图像发布失败: {e}', throttle_duration_sec=5.0)

    def _save_detection_image(self, canvas):
        """检测到人体时保存图像，最小间隔 0.5 秒避免 IO 爆炸"""
        if not self.save_path or self.human_count_current <= 0:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now - getattr(self, '_last_save_time', 0) < 0.5:
            return
        try:
            os.makedirs(self.save_path, exist_ok=True)
            fname = os.path.join(self.save_path, f"det_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.jpg")
            cv2.imwrite(fname, canvas)
            self._last_save_time = now
        except Exception as e:
            self.get_logger().warn(f'图像保存失败: {e}', throttle_duration_sec=5.0)

    def log_status(self):
        """定时发布状态日志"""
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        status = {
            'elapsed': round(elapsed, 1),
            'human_count_current': self.human_count_current,
            'human_count_total': self.human_count_total,
            'avoid_count': self.avoid_count,
            'nearest_human': round(self.nearest_human, 2),
            'nearest_obstacle': round(self.nearest_obstacle, 2),
            'state': self.state,
        }
        log_str = json.dumps(status, ensure_ascii=False)
        self.log_pub.publish(String(data=log_str))
        self.get_logger().info(f'搜救日志: {log_str}')


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.show_ui:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
