#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera_calibration.py
棋盘格相机标定脚本

使用方法：
  1. 打印 A4 棋盘格，建议使用 9x6 或 8x6 内角点
  2. 用 USB 摄像头采集 15-30 张不同角度棋盘格照片
  3. python3 camera_calibration.py --device 0 --square_size 0.025
  4. 将生成的 camera_params.yaml 复制到 config/camera_params.yaml

注意：RDK GS130W（SC132GS 双目模组）的标定参数存储在模组 EEPROM 中，
      使用 mipi_cam 并开启 mipi_gdc_enable=True 时，GDC 会自动读取 EEPROM
      完成硬件去畸变与双目行对齐，无需运行本脚本。
      本脚本主要用于第三方 USB 摄像头或未启用 GDC 的场景。
"""

import argparse
import cv2
import numpy as np
import os
import yaml
import glob
from datetime import datetime


def collect_images(device: int, output_dir: str, num_images: int = 20,
                   pattern_size: tuple = (9, 6), auto_detect: bool = True):
    """实时采集棋盘格照片，支持自动检测有效角点"""
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        raise RuntimeError(f"无法打开摄像头设备: {device}")

    os.makedirs(output_dir, exist_ok=True)
    count = 0
    hint = "自动保存检测到棋盘格的帧" if auto_detect else "按 'c' 采集照片"
    print(f"{hint}，按 'q' 结束，目标 {num_images} 张")
    while count < num_images:
        ret, frame = cap.read()
        if not ret:
            continue

        display = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        if found:
            cv2.drawChessboardCorners(display, pattern_size, corners, found)
            status = "[OK] 检测到角点"
            color = (0, 255, 0)
        else:
            status = "[--] 未检测到角点"
            color = (0, 0, 255)
        cv2.putText(display, f"{status} | {count}/{num_images}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.imshow('Calibration', display)

        key = cv2.waitKey(1) & 0xFF
        save = False
        if auto_detect and found:
            save = True
        elif not auto_detect and key == ord('c'):
            save = True
        if save:
            fname = os.path.join(output_dir, f"calib_{count:03d}.jpg")
            cv2.imwrite(fname, frame)
            print(f"已保存: {fname}")
            count += 1
        elif key == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
    if count < num_images:
        print(f"警告: 仅采集到 {count}/{num_images} 张有效图片")


def calibrate(image_dir: str, pattern_size: tuple, square_size: float, max_reproj_error: float = 1.0):
    """执行标定，并过滤重投影误差过大的异常图像"""
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size

    obj_points = []
    img_points = []
    image_size = None

    images = sorted(glob.glob(os.path.join(image_dir, "*.jpg")) +
                    glob.glob(os.path.join(image_dir, "*.png")))
    if not images:
        raise RuntimeError(f"未找到图像: {image_dir}")

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = gray.shape[::-1]
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        if ret:
            obj_points.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            img_points.append(corners2)
            cv2.drawChessboardCorners(img, pattern_size, corners2, ret)
            cv2.imshow('Corners', img)
            cv2.waitKey(50)

    cv2.destroyAllWindows()

    if len(obj_points) < 5:
        raise RuntimeError(f"有效棋盘格图像不足（仅 {len(obj_points)} 张），请检查图片质量")

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, image_size, None, None)

    # 计算每张图的重投影误差并过滤异常值
    errors = []
    for i in range(len(obj_points)):
        proj, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], mtx, dist)
        err = np.linalg.norm(img_points[i].reshape(-1, 2) - proj.reshape(-1, 2), axis=1).mean()
        errors.append(err)
    errors = np.array(errors)
    keep = errors < max_reproj_error
    if keep.sum() < len(keep) * 0.5:
        print(f"警告: 超过一半图像重投影误差大于 {max_reproj_error}，保留全部结果")
    elif not keep.all():
        print(f"过滤 {(~keep).sum()} 张异常图像后重新标定")
        obj_points = [obj_points[i] for i in range(len(obj_points)) if keep[i]]
        img_points = [img_points[i] for i in range(len(img_points)) if keep[i]]
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, image_size, None, None)

    print("\n标定成功!")
    print("相机内参 K:")
    print(mtx)
    print("畸变系数 D:")
    print(dist.ravel())
    print(f"重投影误差: {ret:.4f}")

    return mtx, dist.ravel(), image_size


def save_yaml(mtx, dist, output_path: str, width: int, height: int):
    """保存标定参数到 ROS2 参数 YAML"""
    cam_params = {
        'camera': {
            'camera_type': 'usb',
            'usb_device': 0,
            'mipi_topic': '/mipi_cam/image_raw',
            'width': int(width),
            'height': int(height),
            'fps': 30,
            'publish_info': True,
            'fx': float(mtx[0, 0]),
            'fy': float(mtx[1, 1]),
            'cx': float(mtx[0, 2]),
            'cy': float(mtx[1, 2]),
            'k1': float(dist[0]) if len(dist) > 0 else 0.0,
            'k2': float(dist[1]) if len(dist) > 1 else 0.0,
            'p1': float(dist[2]) if len(dist) > 2 else 0.0,
            'p2': float(dist[3]) if len(dist) > 3 else 0.0,
            'k3': float(dist[4]) if len(dist) > 4 else 0.0,
        }
    }
    data = {
        'camera_node': {'ros__parameters': cam_params},
        'body_detector_node': {'ros__parameters': {'camera': {
            'fx': float(mtx[0, 0]),
            'fy': float(mtx[1, 1]),
            'cx': float(mtx[0, 2]),
            'cy': float(mtx[1, 2]),
        }}},
    }
    with open(output_path, 'w', encoding='utf-8') as f:
        yaml.dump(data, f, allow_unicode=True, sort_keys=False)
    print(f"\n标定参数已保存: {output_path}")


def update_config_yaml(config_path: str, mtx, dist, width: int, height: int):
    """将标定结果写回项目 config/camera_params.yaml，保留其他字段"""
    if os.path.exists(config_path):
        with open(config_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
    else:
        data = {}

    data.setdefault('camera_node', {}).setdefault('ros__parameters', {}).setdefault('camera', {})
    data.setdefault('body_detector_node', {}).setdefault('ros__parameters', {}).setdefault('camera', {})

    data['camera_node']['ros__parameters']['camera'].update({
        'fx': float(mtx[0, 0]),
        'fy': float(mtx[1, 1]),
        'cx': float(mtx[0, 2]),
        'cy': float(mtx[1, 2]),
        'k1': float(dist[0]) if len(dist) > 0 else 0.0,
        'k2': float(dist[1]) if len(dist) > 1 else 0.0,
        'p1': float(dist[2]) if len(dist) > 2 else 0.0,
        'p2': float(dist[3]) if len(dist) > 3 else 0.0,
        'k3': float(dist[4]) if len(dist) > 4 else 0.0,
        'width': int(width),
        'height': int(height),
    })
    data['body_detector_node']['ros__parameters']['camera'].update({
        'fx': float(mtx[0, 0]),
        'fy': float(mtx[1, 1]),
        'cx': float(mtx[0, 2]),
        'cy': float(mtx[1, 2]),
    })

    with open(config_path, 'w', encoding='utf-8') as f:
        yaml.dump(data, f, allow_unicode=True, sort_keys=False)
    print(f"已更新配置文件: {config_path}")


def main():
    parser = argparse.ArgumentParser(description='RDK 单目摄像头标定')
    parser.add_argument('--device', type=int, default=0, help='USB 摄像头设备号')
    parser.add_argument('--collect', action='store_true', help='是否先采集图像')
    parser.add_argument('--manual', action='store_true', help='采集时手动按 c 保存（默认自动保存检测到角点的帧）')
    parser.add_argument('--image_dir', type=str, default='./calib_images', help='图像目录')
    parser.add_argument('--pattern_width', type=int, default=9, help='棋盘格内角点宽度')
    parser.add_argument('--pattern_height', type=int, default=6, help='棋盘格内角点高度')
    parser.add_argument('--square_size', type=float, default=0.025, help='棋盘格方格边长（米）')
    parser.add_argument('--width', type=int, default=640, help='图像宽度')
    parser.add_argument('--height', type=int, default=480, help='图像高度')
    parser.add_argument('--max_reproj_error', type=float, default=1.0, help='最大单图重投影误差（像素）')
    parser.add_argument('--output', type=str, default='camera_params.yaml', help='输出 YAML 文件')
    parser.add_argument('--update_config', action='store_true', help='同时写回 config/camera_params.yaml')
    args = parser.parse_args()

    if args.collect:
        collect_images(args.device, args.image_dir,
                       pattern_size=(args.pattern_width, args.pattern_height),
                       auto_detect=not args.manual)

    mtx, dist, image_size = calibrate(args.image_dir,
                                      (args.pattern_width, args.pattern_height),
                                      args.square_size,
                                      max_reproj_error=args.max_reproj_error)
    width, height = image_size
    save_yaml(mtx, dist, args.output, width, height)

    if args.update_config:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(script_dir, '..', 'config', 'camera_params.yaml')
        update_config_yaml(config_path, mtx, dist, width, height)


if __name__ == '__main__':
    main()
