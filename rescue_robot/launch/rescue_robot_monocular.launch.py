#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rescue_robot_monocular.launch.py
RDK GS130W 单目摄像头 + 纯视觉感知节点一键启动

启动内容：
  - mipi_cam_gs130w.launch.py : TROS MIPI 相机驱动（单目模式）
  - rescue_robot.launch.py      : 纯视觉感知节点（相机转发 / 人体检测 / 单目深度 / 可视化 / TF）
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('rescue_robot')
    launch_dir = os.path.join(pkg_share, 'launch')
    config_dir = os.path.join(pkg_share, 'config')
    robot_params = os.path.join(config_dir, 'robot_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'mipi_cam_gs130w.launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'rescue_robot.launch.py')
            )
        ),

        # 舵机云台节点：订阅人体检测结果，自动将最大人体框居中（Pan 单轴）
        Node(
            package='rescue_robot',
            executable='camera_servo_node',
            name='camera_servo_node',
            output='screen',
            parameters=[robot_params],
        ),
    ])
