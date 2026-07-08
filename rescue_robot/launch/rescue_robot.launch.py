#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rescue_robot.launch.py
纯视觉方案一键启动：仅包含视觉感知节点与基础 TF
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('rescue_robot')
    config_dir = os.path.join(pkg_share, 'config')
    camera_params = os.path.join(config_dir, 'camera_params.yaml')
    robot_params = os.path.join(config_dir, 'robot_params.yaml')

    urdf_path = os.path.join(pkg_share, 'urdf', 'rescue_robot.urdf')
    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    return LaunchDescription([
        # 相机节点（rescue_robot 内部处理）
        # 注意：MIPI 相机驱动需要先单独启动：ros2 launch rescue_robot mipi_cam_gs130w.launch.py
        Node(
            package='rescue_robot',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[camera_params],
        ),

        # 人体检测节点（使用 ExecuteProcess 确保环境变量正确传递）
        ExecuteProcess(
            cmd=[
                '/bin/sh', '-c',
                'export LD_LIBRARY_PATH="/usr/hobot/lib:/app/lib:/app/pub/lib:/middleware/lib:/middleware/pub/lib:/usr/hobot/lib/sensor:/system/lib:/system/usr/lib:/lib:$LD_LIBRARY_PATH" && export PATH="/usr/hobot/bin:$PATH" && ros2 run rescue_robot body_detector_node --ros-args --params-file ' + camera_params + ' --params-file ' + robot_params
            ],
            name='body_detector_node',
            output='screen',
        ),

        # 深度估计节点（单目深度）
        Node(
            package='rescue_robot',
            executable='depth_estimator_node',
            name='depth_estimator_node',
            output='screen',
            parameters=[robot_params],
        ),

        # 舵机云台节点（默认启用模拟模式，接入 PCA9685 后改 simulation: false）
        Node(
            package='rescue_robot',
            executable='camera_servo_node',
            name='camera_servo_node',
            output='screen',
            parameters=[robot_params],
        ),

        # 可视化节点（SSH无桌面时show_ui设为false即可，overlay图像仍会发布）
        Node(
            package='rescue_robot',
            executable='visualization_node',
            name='visualization_node',
            output='screen',
            parameters=[robot_params],
            additional_env={'QT_QPA_PLATFORM': 'minimal'}
        ),

        # 机器人模型与静态 TF（base_link -> camera_link 由 URDF 定义）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
    ])
