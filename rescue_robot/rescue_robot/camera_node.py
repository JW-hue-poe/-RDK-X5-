#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera_node.py
基于RDK单目视觉的救援寻路机器人 - 图像采集节点
支持 USB 摄像头或 MIPI 相机转发
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # 参数声明
        self.declare_parameter('camera.camera_type', 'mipi')
        self.declare_parameter('camera.simulate', False)
        self.declare_parameter('camera.usb_device', 0)
        self.declare_parameter('camera.width', 640)
        self.declare_parameter('camera.height', 480)
        self.declare_parameter('camera.fps', 30)
        self.declare_parameter('camera.publish_info', True)
        self.declare_parameter('camera.mipi_topic', '/mipi_cam/image_raw')
        self.declare_parameter('camera.mipi_gdc_enable', False)
        self.declare_parameter('camera.simulation', False)
        self.declare_parameter('camera.fx', 609.0)
        self.declare_parameter('camera.fy', 609.0)
        self.declare_parameter('camera.cx', 320.0)
        self.declare_parameter('camera.cy', 240.0)
        self.declare_parameter('camera.k1', 0.0)
        self.declare_parameter('camera.k2', 0.0)
        self.declare_parameter('camera.p1', 0.0)
        self.declare_parameter('camera.p2', 0.0)
        self.declare_parameter('camera.k3', 0.0)

        self.camera_type = self.get_parameter('camera.camera_type').value
        self.simulate = self.get_parameter('camera.simulate').value
        self.simulation = self.get_parameter('camera.simulation').value
        self.width = self.get_parameter('camera.width').value
        self.height = self.get_parameter('camera.height').value
        self.fps = self.get_parameter('camera.fps').value

        self.publish_info = self.get_parameter('camera.publish_info').value
        self.mipi_gdc_enable = self.get_parameter('camera.mipi_gdc_enable').value
        self._build_camera_info()

        # 发布图像和相机信息
        self.image_pub = self.create_publisher(Image, '/rescue_robot/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/rescue_robot/camera_info', 10) if self.publish_info else None

        self.bridge = CvBridge()
        self.cap = None
        self.mipi_sub = None
        self.timer = None
        self._reconnect_timer = None
        self._frame_count = 0
        self._sim_human_x = 200

        if self.simulation or self.simulate:
            self._init_simulation()
        elif self.camera_type == 'usb':
            self._init_usb_camera()
        elif self.camera_type == 'mipi':
            self._init_mipi_proxy()
        else:
            self.get_logger().error(f'不支持的 camera_type: {self.camera_type}')
            return

        self.get_logger().info(
            f'相机节点已启动：类型={self.camera_type}, '
            f'分辨率={self.width}x{self.height}, 帧率={self.fps}'
        )

    def _build_camera_info(self):
        """根据参数构建 CameraInfo 消息。若 MIPI GDC 已启用，图像已被硬件去畸变，畸变系数置 0。"""
        fx = self.get_parameter('camera.fx').value
        fy = self.get_parameter('camera.fy').value
        cx = self.get_parameter('camera.cx').value
        cy = self.get_parameter('camera.cy').value

        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = 'camera_link'
        self.camera_info.width = self.width
        self.camera_info.height = self.height
        self.camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]

        if self.mipi_gdc_enable:
            # GDC 硬件已完成畸变校正与双目对齐，发布无畸变内参
            self.camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.camera_info.distortion_model = 'plumb_bob'
        else:
            self.camera_info.d = [
                self.get_parameter('camera.k1').value,
                self.get_parameter('camera.k2').value,
                self.get_parameter('camera.p1').value,
                self.get_parameter('camera.p2').value,
                self.get_parameter('camera.k3').value,
            ]
            self.camera_info.distortion_model = 'plumb_bob'

        self.camera_info.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

    def _init_usb_camera(self):
        device = self.get_parameter('camera.usb_device').value
        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            self.get_logger().error(f'无法打开 USB 摄像头设备: {device}，将尝试重连')
            self._start_reconnect()
            return
        self._configure_capture()
        self.timer = self.create_timer(1.0 / self.fps, self._usb_capture)
        if self._reconnect_timer is not None:
            self._reconnect_timer.cancel()
            self._reconnect_timer = None

    def _configure_capture(self):
        if self.cap is None:
            return
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    def _start_reconnect(self):
        if self._reconnect_timer is not None:
            return
        self._reconnect_timer = self.create_timer(2.0, self._reconnect_usb)

    def _reconnect_usb(self):
        device = self.get_parameter('camera.usb_device').value
        self.get_logger().info(f'尝试重连 USB 摄像头: {device}')
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(device)
        if self.cap.isOpened():
            self.get_logger().info(f'USB 摄像头重连成功: {device}')
            self._configure_capture()
            self.timer = self.create_timer(1.0 / self.fps, self._usb_capture)
            self._reconnect_timer.cancel()
            self._reconnect_timer = None

    def _init_simulation(self):
        self.timer = self.create_timer(1.0 / self.fps, self._simulation_capture)
        self.get_logger().info('已启动仿真模式，生成模拟图像')

    def _init_mipi_proxy(self):
        mipi_topic = self.get_parameter('camera.mipi_topic').value
        self.mipi_sub = self.create_subscription(Image, mipi_topic, self._mipi_callback, 10)
        if self.mipi_gdc_enable:
            self.get_logger().info(
                f'已订阅 MIPI 相机话题: {mipi_topic}，GDC 硬件去畸变/行对齐已启用（EEPROM 参数）'
            )
        else:
            self.get_logger().info(f'已订阅 MIPI 相机话题: {mipi_topic}')

    def _usb_capture(self):
        if self.cap is None or not self.cap.isOpened():
            return
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn('USB 摄像头取流失败，启动重连', throttle_duration_sec=5.0)
            if self.timer is not None:
                self.timer.cancel()
                self.timer = None
            self._start_reconnect()
            return
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))
        self._publish(frame)

    def _simulation_capture(self):
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        self._sim_human_x = (self._sim_human_x + 2) % self.width
        cv2.rectangle(frame, (self._sim_human_x - 40, 100), (self._sim_human_x + 40, 350), (0, 255, 0), -1)
        self._publish(frame)

    def _mipi_callback(self, msg: Image):
        encoding = msg.encoding.lower()
        try:
            if encoding in ('bgr8', 'rgb8'):
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            elif encoding == 'nv12':
                frame = self._nv12_to_bgr(msg)
            else:
                self.get_logger().warn(f'暂不支持的图像编码: {msg.encoding}', throttle_duration_sec=5.0)
                return
        except Exception as e:
            self.get_logger().warn(f'cv_bridge 转换失败: {e}', throttle_duration_sec=5.0)
            return
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))
        self._publish(frame)

    def _nv12_to_bgr(self, msg: Image):
        """将 NV12 编码的 Image 消息转换为 BGR 格式"""
        # NV12: Y 平面 size = width * height, UV 平面 size = width * height / 2
        # 总字节数 = width * height * 1.5
        expected = int(msg.width * msg.height * 1.5)
        if len(msg.data) != expected:
            self.get_logger().warn(
                f'NV12 数据长度不匹配: 实际 {len(msg.data)}, 期望 {expected}',
                throttle_duration_sec=5.0
            )
            # 尝试按最小有效数据转换
        yuv = np.frombuffer(msg.data, dtype=np.uint8, count=expected)
        yuv = yuv.reshape((int(msg.height * 1.5), msg.width))
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)

    def _publish(self, frame):
        now = self.get_clock().now().to_msg()

        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'图像消息转换失败: {e}', throttle_duration_sec=5.0)
            return

        img_msg.header.stamp = now
        img_msg.header.frame_id = 'camera_link'
        self.image_pub.publish(img_msg)
        self._frame_count += 1

        if self.info_pub is not None:
            self.camera_info.header.stamp = now
            self.info_pub.publish(self.camera_info)

        # 每 300 帧输出一次统计信息
        if self._frame_count % 300 == 0:
            self.get_logger().info(f'相机节点已发布 {self._frame_count} 帧图像')

    def destroy_node(self):
        for t in (self.timer, self._reconnect_timer):
            if t is not None:
                t.cancel()
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
